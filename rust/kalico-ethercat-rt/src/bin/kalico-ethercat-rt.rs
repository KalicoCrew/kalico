//! Usage: kalico-ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rt-cpu N] [--rt-prio N]
#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::atomic::{AtomicBool, Ordering};

use kalico_ethercat_rt::claim::{eval_wkc, single_slave_reply, wait_for_claim, WkcDecision};
use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT, NUM_AXES};
use kalico_ethercat_rt::ffi;
use kalico_ethercat_rt::scale::CountMap;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED,
};
use kalico_ethercat_rt::wire::{
    claim_handshake_reply_frame, identify_response_frame, push_pieces_response_frame,
    runtime_caps_response_frame, set_torque_response_frame, status_heartbeat_frame, Command,
};
use kalico_protocol::messages::SlaveState;

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigterm(_: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::Release);
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let ifname = args.get(1).cloned().unwrap_or_else(|| "eth0".into());
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let cycle_us: i64 = arg_val(&args, "--cycle-us")
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);
    let counts_per_mm: f64 = arg_val(&args, "--counts-per-mm")
        .and_then(|s| s.parse().ok())
        .unwrap_or(3276.8);
    let rt_cpu: i32 = arg_val(&args, "--rt-cpu")
        .and_then(|s| s.parse().ok())
        .unwrap_or(3);
    let rt_prio: i32 = arg_val(&args, "--rt-prio")
        .and_then(|s| s.parse().ok())
        .unwrap_or(80);
    let cycle_ns = cycle_us * 1000;
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let mut ring = AxisRing::new();
    let mut cmap: Option<CountMap> = None;
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt: socket {socket}, cycle {cycle_us}us, counts/mm {counts_per_mm}");

    // SAFETY: on_sigterm only touches a static AtomicBool; SA_RESTART (and no
    // SA_RESETHAND) keeps a second SIGTERM on the clean-shutdown path too.
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");
    let rc = unsafe { ffi::ec_rt_bringup(cif.as_ptr(), cycle_ns, rt_cpu, rt_prio) };
    if rc != 0 {
        eprintln!("ec-rt: bringup failed rc={rc}, sending handshake-fail then exiting");
        let claim_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        if let Some(cid) = wait_for_claim(&mut server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt") {
            let reply = single_slave_reply(
                1,
                SlaveState::Offline,
                u16::try_from(rc.unsigned_abs()).unwrap_or(u16::MAX),
            );
            server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
            eprintln!("ec-rt: sent offline handshake reply, exiting");
        } else {
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; aborting");
        }
        std::process::exit(1);
    }
    eprintln!("ec-rt: drive parked (Ready-to-Switch-On, no torque)");

    match wait_for_claim(
        &mut server,
        std::time::Instant::now() + std::time::Duration::from_secs(5),
        &SIGTERM_RECEIVED,
        "ec-rt",
    ) {
        Some(cid) => {
            server.respond(&claim_handshake_reply_frame(
                cid,
                &single_slave_reply(1, SlaveState::Ok, 0),
            ));
        }
        None => {
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; aborting");
            unsafe {
                ffi::ec_rt_disable();
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
    eprintln!("ec-rt: handshake ok, entering DC loop");

    let mut gate = TorqueGate::new();
    let mut prdiv = 0u64;
    let mut wkc_consecutive = 0u8;
    'dc: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt: SIGTERM received — disabling drive and exiting");
            break;
        }
        if server.session_ended() {
            eprintln!("ec-rt: bridge disconnected — disabling drive and exiting");
            break;
        }

        for cmd in server.poll_commands() {
            match cmd {
                Command::Identify {
                    correlation_id,
                    proto_version,
                } => {
                    server.respond(&identify_response_frame(correlation_id, proto_version));
                }
                Command::PushPieces {
                    correlation_id,
                    msg,
                } => {
                    let front_start_time = if msg.piece_count > 0 && msg.pieces_bytes.len() >= 8 {
                        u64::from_le_bytes(msg.pieces_bytes[0..8].try_into().unwrap_or([0; 8]))
                    } else {
                        0
                    };
                    let pushed = ring.push_from_bytes(msg.piece_count, &msg.pieces_bytes);
                    let now_ns = monotonic_ns();
                    #[allow(clippy::cast_precision_loss)]
                    let delta_ms = (now_ns as i64 - front_start_time as i64) as f64 / 1_000_000.0;
                    eprintln!(
                        "ec-rt: PushPieces axis={} pieces={} pushed={} head={} \
                         now_ns={} front_start_ns={} delta_ms={:.3}",
                        msg.axis_idx,
                        msg.piece_count,
                        pushed,
                        msg.new_head,
                        now_ns,
                        front_start_time,
                        delta_ms
                    );
                    let arrival_clock = now_ns;
                    let result = if pushed == msg.piece_count {
                        0i32
                    } else {
                        -309
                    };
                    server.respond(&push_pieces_response_frame(
                        correlation_id,
                        result,
                        arrival_clock,
                        front_start_time,
                    ));
                }
                Command::QueryRuntimeCaps { correlation_id } => {
                    let total: u32 = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                    server.respond(&runtime_caps_response_frame(correlation_id, total));
                }
                Command::SetTorque {
                    correlation_id,
                    msg,
                } => {
                    let now_ns = monotonic_ns();
                    match gate.on_set_torque(msg.value != 0, msg.execute_at_ns, now_ns) {
                        CommandAction::Enable => {
                            let rc = unsafe { ffi::ec_rt_enable() };
                            gate.enable_finished(rc == 0);
                            if rc == 0 {
                                eprintln!("ec-rt: torque enabled (CiA402 operation enabled)");
                                server.respond(&set_torque_response_frame(correlation_id, 0));
                            } else {
                                eprintln!(
                                    "ec-rt: CiA402 enable failed rc={rc} — disabling and exiting"
                                );
                                server.respond(&set_torque_response_frame(
                                    correlation_id,
                                    ERR_ENABLE_FAILED,
                                ));
                                unsafe {
                                    ffi::ec_rt_disable();
                                    ffi::ec_rt_shutdown();
                                }
                                std::process::exit(1);
                            }
                        }
                        CommandAction::ScheduleDisable => {
                            eprintln!(
                                "ec-rt: torque disable scheduled at {} (now {now_ns})",
                                msg.execute_at_ns
                            );
                            server.respond(&set_torque_response_frame(correlation_id, 0));
                        }
                        CommandAction::Reject { code } => {
                            eprintln!(
                                "ec-rt: SetTorque rejected code={code} \
                                 (value={} execute_at={} now={now_ns}) — exiting",
                                msg.value, msg.execute_at_ns
                            );
                            server.respond(&set_torque_response_frame(correlation_id, code));
                            unsafe {
                                ffi::ec_rt_disable();
                                ffi::ec_rt_shutdown();
                            }
                            std::process::exit(1);
                        }
                    }
                }
                Command::ClaimHandshake { .. } => {
                    eprintln!(
                        "ec-rt: protocol violation: ClaimHandshake after handshake \
                         — ending session"
                    );
                    break 'dc;
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let now = monotonic_ns();

        match gate.on_tick(now, ring.is_empty()) {
            TickAction::None => {}
            TickAction::ExecuteDisable => {
                eprintln!("ec-rt: scheduled torque disable executing");
                unsafe { ffi::ec_rt_disable() };
                gate.disable_finished();
                cmap = None;
            }
            TickAction::Fault { code } => {
                eprintln!(
                    "ec-rt: torque-gate fault code={code} — pieces present without torque, exiting"
                );
                server.respond(&status_heartbeat_frame(
                    ENGINE_STATE_FAULT,
                    &[ring.retired_count()],
                ));
                unsafe {
                    ffi::ec_rt_disable();
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }
        }

        if gate.state() == TorqueState::Enabled {
            if let Some((pos_mm, _vel_mm_s)) = ring.sample(now) {
                let map = cmap.get_or_insert_with(|| {
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, f64::from(pos_mm))
                });
                let counts = map.target_counts(f64::from(pos_mm));
                unsafe { ffi::ec_rt_set_target_position(counts) };
            } else {
                cmap = None;
            }
        }

        if let Some(fault_val) = ring.take_fault() {
            let fault_code_u16 = (fault_val & 0xFFFF) as u16;
            eprintln!(
                "ec-rt: FAULT latched fault_val=0x{fault_val:08x} code=0x{fault_code_u16:04x} \
                 — notifying host via heartbeat"
            );
            let current_retired = ring.retired_count();
            server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                &[current_retired],
            ));

            #[cfg(feature = "hw")]
            {
                eprintln!("ec-rt: disabling drive (hw safety backstop)");
                unsafe {
                    ffi::ec_rt_disable();
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }

            #[cfg(not(feature = "hw"))]
            {
                last_sent_retired = current_retired;
                heartbeat_sent = true;
            }
        }

        let mut toff = 0i64;
        let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };

        match eval_wkc(wkc, 3, &mut wkc_consecutive) {
            WkcDecision::Good => {}
            WkcDecision::Warn(n) => {
                eprintln!(
                    "ec-rt: WARNING — working counter {wkc} (expected 3), \
                     consecutive_bad={n}; tolerating (USB-NIC frame loss); \
                     halt threshold={}",
                    kalico_ethercat_rt::claim::WKC_CONSECUTIVE_LOSS_LIMIT
                );
            }
            WkcDecision::Halt => {
                eprintln!(
                    "ec-rt: working counter {wkc} (expected 3), \
                     consecutive_bad={wkc_consecutive} — bus lost after \
                     {} consecutive bad cycles, halting",
                    kalico_ethercat_rt::claim::WKC_CONSECUTIVE_LOSS_LIMIT
                );
                break;
            }
        }

        let current_retired = ring.retired_count();
        let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(engine_state, &[current_retired]));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt: heartbeat retired_count={current_retired}");
            }
        }

        prdiv += 1;
        if prdiv >= telemetry_period {
            prdiv = 0;
            let (sw, err, pos, ferr) = unsafe {
                (
                    ffi::ec_rt_get_statusword(),
                    ffi::ec_rt_get_error_code(),
                    ffi::ec_rt_get_position_actual(),
                    ffi::ec_rt_get_following_error(),
                )
            };
            eprintln!(
                "ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{err:04x} pos={pos} ferr={ferr} toff={toff} \
                 ring_len={} retired={}",
                ring.is_empty() as u8 ^ 1,
                current_retired,
            );
            if err != 0 {
                eprintln!("ec-rt: DRIVE FAULT err=0x{err:04x}, disabling");
                break;
            }
        }
    }

    unsafe {
        ffi::ec_rt_disable();
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}
