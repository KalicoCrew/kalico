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
use kalico_ethercat_rt::sdo::{execute_sdo_read, execute_sdo_write, SdoBus};
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use kalico_ethercat_rt::wire::{
    claim_handshake_reply_frame, identify_response_frame, push_pieces_response_frame,
    restore_drive_limits_response_frame, runtime_caps_response_frame, sdo_read_response_frame,
    sdo_write_response_frame, set_drive_limits_response_frame, set_torque_response_frame,
    status_heartbeat_frame, stop_response_frame, Command,
};
use kalico_protocol::messages::{SlaveState, ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE};

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigterm(_: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::Release);
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

struct FfiSdoBus;

fn ffi_sdo_error(abort: u32) -> i32 {
    if abort == 0 {
        return ERR_SDO_TRANSPORT;
    }
    debug_assert!(
        abort < 0x8000_0000,
        "CoE abort code 0x{abort:08x} would collide with local error codes as i32"
    );
    abort as i32
}

impl SdoBus for FfiSdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        let mut buf = [0u8; 8];
        let mut size: std::os::raw::c_int = buf.len() as std::os::raw::c_int;
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_read(index, subindex, buf.as_mut_ptr(), &mut size, &mut abort)
        };
        if rc != 0 {
            return Err(ffi_sdo_error(abort));
        }
        if !(1..=4).contains(&size) {
            return Err(ERR_SDO_UNSUPPORTED_SIZE);
        }
        let mut data = [0u8; 4];
        data[..size as usize].copy_from_slice(&buf[..size as usize]);
        Ok((size as u8, data))
    }

    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_write(
                index,
                subindex,
                bytes.as_ptr(),
                bytes.len() as std::os::raw::c_int,
                &mut abort,
            )
        };
        if rc != 0 {
            return Err(ffi_sdo_error(abort));
        }
        Ok(())
    }
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

    let run_limits: (u32, u16) = {
        let mut ferr = 0u32;
        let mut tmo = 0u16;
        let mut tq = 0u16;
        let rc = unsafe { ffi::ec_rt_read_limits(&mut ferr, &mut tmo, &mut tq) };
        if rc != 0 {
            eprintln!("ec-rt: SDO read of protection limits failed rc={rc} — aborting bringup");
            unsafe {
                ffi::ec_rt_disable();
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
        eprintln!("ec-rt: drive limits at bringup: 6065h={ferr} counts, 6066h={tmo} ms, 6072h={tq} (0.1%)");
        let cli_ferr: Option<u32> =
            arg_val(&args, "--following-error-counts").and_then(|s| s.parse().ok());
        let cli_tq: Option<u16> =
            arg_val(&args, "--max-torque-tenth-pct").and_then(|s| s.parse().ok());
        let run = (cli_ferr.unwrap_or(ferr), cli_tq.unwrap_or(tq));
        if cli_ferr.is_some() || cli_tq.is_some() {
            let rc = unsafe { ffi::ec_rt_write_limits(run.0, run.1) };
            if rc != 0 {
                eprintln!("ec-rt: SDO write of session limits failed rc={rc} — aborting bringup");
                unsafe {
                    ffi::ec_rt_disable();
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }
            eprintln!(
                "ec-rt: session limits applied: 6065h={} 6072h={}",
                run.0, run.1
            );
        }
        run
    };

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
    let mut sdo_bus = FfiSdoBus;
    let mut prdiv = 0u64;
    let mut wkc_consecutive = 0u8;
    let mut latched_drive_err: u16 = 0;
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
                    let now_ns = monotonic_ns();
                    if gate.state() == TorqueState::Faulted {
                        server.respond(&push_pieces_response_frame(
                            correlation_id,
                            ERR_PIECES_WHILE_FAULTED,
                            now_ns,
                            0,
                        ));
                    } else {
                        let front_start_time = if msg.piece_count > 0 && msg.pieces_bytes.len() >= 8
                        {
                            u64::from_le_bytes(msg.pieces_bytes[0..8].try_into().unwrap_or([0; 8]))
                        } else {
                            0
                        };
                        let pushed = ring.push_from_bytes(msg.piece_count, &msg.pieces_bytes);
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
                }
                Command::QueryRuntimeCaps { correlation_id } => {
                    let total: u32 = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                    server.respond(&runtime_caps_response_frame(correlation_id, total));
                }
                Command::SetTorque {
                    correlation_id,
                    msg,
                } => match gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
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
                            "ec-rt: torque disable scheduled at {} (now {})",
                            msg.execute_at_ns,
                            monotonic_ns()
                        );
                        server.respond(&set_torque_response_frame(correlation_id, 0));
                    }
                    CommandAction::Reject { code } => {
                        eprintln!(
                            "ec-rt: SetTorque rejected code={code} \
                                 (value={} execute_at={} now={}) — exiting",
                            msg.value,
                            msg.execute_at_ns,
                            monotonic_ns()
                        );
                        server.respond(&set_torque_response_frame(correlation_id, code));
                        unsafe {
                            ffi::ec_rt_disable();
                            ffi::ec_rt_shutdown();
                        }
                        std::process::exit(1);
                    }
                },
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    ring.reset();
                    cmap = None;
                    eprintln!("ec-rt: Stop — ring discarded, discard_clock={now_ns}");
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
                Command::ClaimHandshake { .. } => {
                    eprintln!(
                        "ec-rt: protocol violation: ClaimHandshake after handshake \
                         — ending session"
                    );
                    break 'dc;
                }
                Command::SetDriveLimits {
                    correlation_id,
                    msg,
                } => {
                    let rc = unsafe {
                        ffi::ec_rt_write_limits(
                            msg.following_error_counts,
                            msg.max_torque_tenth_pct,
                        )
                    };
                    if rc != 0 {
                        eprintln!(
                            "ec-rt: SetDriveLimits SDO write failed rc={rc} \
                             ferr={} tq={}",
                            msg.following_error_counts, msg.max_torque_tenth_pct
                        );
                    } else {
                        eprintln!(
                            "ec-rt: SetDriveLimits applied ferr={} tq={}",
                            msg.following_error_counts, msg.max_torque_tenth_pct
                        );
                    }
                    server.respond(&set_drive_limits_response_frame(correlation_id, rc));
                }
                Command::RestoreDriveLimits { correlation_id } => {
                    let rc = unsafe { ffi::ec_rt_write_limits(run_limits.0, run_limits.1) };
                    if rc != 0 {
                        eprintln!(
                            "ec-rt: RestoreDriveLimits SDO write failed rc={rc} \
                             ferr={} tq={}",
                            run_limits.0, run_limits.1
                        );
                    } else {
                        eprintln!(
                            "ec-rt: RestoreDriveLimits applied ferr={} tq={}",
                            run_limits.0, run_limits.1
                        );
                    }
                    server.respond(&restore_drive_limits_response_frame(correlation_id, rc));
                }
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_read(&mut sdo_bus, &msg);
                    if resp.result != 0 {
                        eprintln!(
                            "ec-rt: SdoRead 0x{:04x}.{} failed result={}",
                            msg.index, msg.subindex, resp.result
                        );
                    }
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_write(&mut sdo_bus, &msg);
                    if resp.result != 0 {
                        eprintln!(
                            "ec-rt: SdoWrite 0x{:04x}.{} value={} size={} failed result={}",
                            msg.index, msg.subindex, msg.value, msg.size, resp.result
                        );
                    }
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
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
                    0,
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
                (fault_val & 0xFFFF) as u16,
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

        let drive_err = unsafe { ffi::ec_rt_get_error_code() };
        if drive_err != 0 && gate.state() != TorqueState::Faulted {
            eprintln!(
                "ec-rt: DRIVE FAULT err=0x{drive_err:04x} — parking, reporting via heartbeat"
            );
            gate.on_drive_fault();
            ring.reset();
            cmap = None;
            latched_drive_err = drive_err;
            server.respond(&status_heartbeat_frame(
                0,
                drive_err,
                &[ring.retired_count()],
            ));
            last_sent_retired = ring.retired_count();
            heartbeat_sent = true;
        }

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
            server.respond(&status_heartbeat_frame(engine_state, 0, &[current_retired]));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt: heartbeat retired_count={current_retired}");
            }
        }

        prdiv += 1;
        if prdiv >= telemetry_period {
            prdiv = 0;
            let (sw, pos, ferr) = unsafe {
                (
                    ffi::ec_rt_get_statusword(),
                    ffi::ec_rt_get_position_actual(),
                    ffi::ec_rt_get_following_error(),
                )
            };
            eprintln!(
                "ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{drive_err:04x} pos={pos} ferr={ferr} toff={toff} \
                 ring_len={} retired={}",
                ring.is_empty() as u8 ^ 1,
                current_retired,
            );
            if gate.state() == TorqueState::Faulted {
                server.respond(&status_heartbeat_frame(
                    0,
                    latched_drive_err,
                    &[current_retired],
                ));
            }
        }
    }

    unsafe {
        ffi::ec_rt_disable();
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}
