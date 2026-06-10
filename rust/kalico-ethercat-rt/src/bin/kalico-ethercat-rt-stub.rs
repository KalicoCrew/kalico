#![allow(unsafe_code)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::Duration;

use kalico_ethercat_rt::claim::{parse_fail_bringup, single_slave_reply, wait_for_claim};
use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT, NUM_AXES};
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use kalico_ethercat_rt::wire::{
    claim_handshake_reply_frame, identify_response_frame, push_pieces_response_frame,
    restore_drive_limits_response_frame, runtime_caps_response_frame,
    set_drive_limits_response_frame, set_torque_response_frame, status_heartbeat_frame,
    stop_response_frame, Command,
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
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());

    let fail_slave: Option<u8> = match parse_fail_bringup(&args) {
        Ok(v) => v,
        Err(msg) => {
            eprintln!("ec-rt-stub: {msg}");
            eprintln!("Usage: kalico-ethercat-rt-stub [--socket PATH] [--fail-bringup slave=N]");
            std::process::exit(2);
        }
    };

    let fail_enable = args.iter().any(|a| a == "--fail-enable");
    let drive_fault_after: Option<u32> =
        arg_val(&args, "--drive-fault-after-pieces").and_then(|s| s.parse().ok());

    let mut ring = AxisRing::new();
    let mut gate = TorqueGate::new();
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;
    let mut sampled_pieces: u32 = 0;
    let mut drive_fault_fired = false;
    let mut stored_limits: Option<(u32, u16)> = None;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket} (NO HARDWARE)");

    // SAFETY: on_sigterm only touches a static AtomicBool; SA_RESTART (and no
    // SA_RESETHAND) keeps a second SIGTERM on the clean-shutdown path too.
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    let claim_deadline = std::time::Instant::now() + Duration::from_secs(10);
    let cid = match wait_for_claim(&mut server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt-stub") {
        Some(id) => id,
        None => {
            eprintln!("ec-rt-stub: bridge did not send ClaimHandshake within 10 s; aborting");
            std::process::exit(1);
        }
    };

    if let Some(slave_idx) = fail_slave {
        let reply = single_slave_reply(slave_idx, SlaveState::Offline, 0);
        server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
        eprintln!("ec-rt-stub: --fail-bringup: sent Offline for slave {slave_idx}, exiting");
        std::process::exit(1);
    }

    server.respond(&claim_handshake_reply_frame(
        cid,
        &single_slave_reply(1, SlaveState::Ok, 0),
    ));
    eprintln!("ec-rt-stub: handshake ok, entering stub loop");

    'session: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt-stub: SIGTERM received — exiting");
            break;
        }
        if server.session_ended() {
            eprintln!("ec-rt-stub: bridge disconnected — exiting");
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
                        #[allow(clippy::cast_precision_loss)]
                        let delta_ms =
                            (now_ns as i64 - front_start_time as i64) as f64 / 1_000_000.0;
                        eprintln!(
                            "ec-rt-stub: PushPieces axis={} pieces={} pushed={} head={} \
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
                }
                Command::QueryRuntimeCaps { correlation_id } => {
                    let total: u32 = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                    server.respond(&runtime_caps_response_frame(correlation_id, total));
                }
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    ring.reset();
                    eprintln!("ec-rt-stub: Stop — ring discarded, discard_clock={now_ns}");
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
                Command::ClaimHandshake { .. } => {
                    eprintln!(
                        "ec-rt-stub: protocol violation: ClaimHandshake after handshake \
                         — ending session"
                    );
                    break 'session;
                }
                Command::SetTorque {
                    correlation_id,
                    msg,
                } => {
                    let now_ns = monotonic_ns();
                    match gate.on_set_torque(msg.value != 0, msg.execute_at_ns, now_ns) {
                        CommandAction::Enable => {
                            let ok = !fail_enable;
                            gate.enable_finished(ok);
                            if ok {
                                eprintln!("ec-rt-stub: torque enabled (simulated)");
                                server.respond(&set_torque_response_frame(correlation_id, 0));
                            } else {
                                eprintln!("ec-rt-stub: simulated enable failure — exiting");
                                server.respond(&set_torque_response_frame(
                                    correlation_id,
                                    ERR_ENABLE_FAILED,
                                ));
                                std::process::exit(1);
                            }
                        }
                        CommandAction::ScheduleDisable => {
                            eprintln!(
                                "ec-rt-stub: torque disable scheduled at {} (now {now_ns})",
                                msg.execute_at_ns
                            );
                            server.respond(&set_torque_response_frame(correlation_id, 0));
                        }
                        CommandAction::Reject { code } => {
                            eprintln!("ec-rt-stub: SetTorque rejected code={code} — exiting");
                            server.respond(&set_torque_response_frame(correlation_id, code));
                            std::process::exit(1);
                        }
                    }
                }
                Command::SetDriveLimits {
                    correlation_id,
                    msg,
                } => {
                    stored_limits = Some((msg.following_error_counts, msg.max_torque_tenth_pct));
                    eprintln!(
                        "ec-rt-stub: SetDriveLimits ferr={} tq={}",
                        msg.following_error_counts, msg.max_torque_tenth_pct
                    );
                    server.respond(&set_drive_limits_response_frame(correlation_id, 0));
                }
                Command::RestoreDriveLimits { correlation_id } => {
                    eprintln!("ec-rt-stub: RestoreDriveLimits stored={stored_limits:?}");
                    server.respond(&restore_drive_limits_response_frame(correlation_id, 0));
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt-stub: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let now = monotonic_ns();

        match gate.on_tick(now, ring.is_empty()) {
            TickAction::None => {}
            TickAction::ExecuteDisable => {
                eprintln!("ec-rt-stub: scheduled torque disable executed");
                gate.disable_finished();
            }
            TickAction::Fault { code } => {
                eprintln!("ec-rt-stub: torque-gate fault code={code} — exiting");
                server.respond(&status_heartbeat_frame(
                    ENGINE_STATE_FAULT,
                    0,
                    &[ring.retired_count()],
                ));
                std::process::exit(1);
            }
        }
        if gate.state() == TorqueState::Enabled && ring.sample(now).is_some() {
            sampled_pieces += 1;
            if !drive_fault_fired {
                if let Some(threshold) = drive_fault_after {
                    if sampled_pieces >= threshold {
                        drive_fault_fired = true;
                        gate.on_drive_fault();
                        ring.reset();
                        eprintln!(
                            "ec-rt-stub: drive fault simulated after {sampled_pieces} pieces"
                        );
                        server.respond(&status_heartbeat_frame(0, 0x8611, &[ring.retired_count()]));
                        last_sent_retired = ring.retired_count();
                        heartbeat_sent = true;
                    }
                }
            }
        }

        if let Some(fault_val) = ring.take_fault() {
            if !drive_fault_fired {
                if let Some(threshold) = drive_fault_after {
                    sampled_pieces += 1;
                    if sampled_pieces >= threshold {
                        drive_fault_fired = true;
                        gate.on_drive_fault();
                        ring.reset();
                        eprintln!("ec-rt-stub: drive fault simulated after {sampled_pieces} pieces (ring fault path)");
                        server.respond(&status_heartbeat_frame(0, 0x8611, &[ring.retired_count()]));
                        last_sent_retired = ring.retired_count();
                        heartbeat_sent = true;
                        continue 'session;
                    }
                }
            }
            let fault_code_u16 = (fault_val & 0xFFFF) as u16;
            eprintln!(
                "ec-rt-stub: FAULT latched fault_val=0x{fault_val:08x} code=0x{fault_code_u16:04x} \
                 — propagating to host via heartbeat, host must shut down"
            );
            let current_retired = ring.retired_count();
            server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                (fault_val & 0xFFFF) as u16,
                &[current_retired],
            ));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
        }

        let current_retired = ring.retired_count();
        let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(engine_state, 0, &[current_retired]));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt-stub: heartbeat retired_count={current_retired}");
            }
        }

        sleep(Duration::from_millis(1));
    }
}
