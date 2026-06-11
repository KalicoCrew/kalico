//! Usage: kalico-ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rt-cpu N] [--rt-prio N]
//!        [--velocity-ff] [--dynamics-profile PATH] [--torque-clamp-pct F]
#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::atomic::{AtomicBool, Ordering};

use kalico_ethercat_rt::capture::{
    Capture, CaptureConfig, CaptureRecord, DriveSample, PendingStart, PendingStop,
    FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED,
};
use kalico_ethercat_rt::claim::{
    eval_wkc, single_slave_reply, wait_for_claim, wait_for_claim_pumping, WkcDecision,
};
use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT, NUM_AXES};
use kalico_ethercat_rt::dynamics::{clamp_torque, DynamicsModel};
use kalico_ethercat_rt::ffi;
use kalico_ethercat_rt::mailbox::{MailboxReply, MailboxRequest, MailboxWorker};
use kalico_ethercat_rt::scale::CountMap;
use kalico_ethercat_rt::sdo::SdoBus;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use kalico_ethercat_rt::wire::{
    claim_handshake_reply_frame, identify_response_frame, push_pieces_response_frame,
    restore_drive_limits_response_frame, resume_stream_response_frame, runtime_caps_response_frame,
    sdo_read_response_frame, sdo_write_response_frame, set_drive_limits_response_frame,
    set_torque_response_frame, start_capture_response_frame, status_heartbeat_frame,
    stop_capture_response_frame, stop_response_frame, Command,
};
use kalico_protocol::messages::{
    SlaveState, StopCaptureResponse, ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE,
};

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
    let velocity_ff = args.iter().any(|a| a == "--velocity-ff");
    let torque_clamp_tenths: i16 = arg_val(&args, "--torque-clamp-pct")
        .and_then(|s| s.parse::<f64>().ok())
        .map(|pct| {
            if !(pct > 0.0 && pct <= 400.0) {
                eprintln!("ec-rt: --torque-clamp-pct {pct} outside (0, 400]");
                std::process::exit(1);
            }
            (pct * 10.0) as i16
        })
        .unwrap_or(300);
    let dynamics = arg_val(&args, "--dynamics-profile").map(|path| {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path}: {e}");
            std::process::exit(1);
        });
        let model = DynamicsModel::from_toml_str(&text).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path} invalid: {e:?}");
            std::process::exit(1);
        });
        if model.n != NUM_AXES {
            eprintln!(
                "ec-rt: dynamics profile {path} has {} axes, endpoint drives {NUM_AXES}",
                model.n
            );
            std::process::exit(1);
        }
        model
    });
    let cycle_ns = cycle_us * 1000;
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let mut ring = AxisRing::new();
    let mut cmap: Option<CountMap> = None;
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!(
        "ec-rt: socket {socket}, cycle {cycle_us}us, counts/mm {counts_per_mm} \
         velocity_ff={velocity_ff} dynamics={} clamp={torque_clamp_tenths}",
        dynamics.is_some()
    );

    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    fn bringup_fail(server: &mut FrameServer, rc: i32) -> ! {
        eprintln!("ec-rt: bringup failed rc={rc}, sending handshake-fail then exiting");
        let claim_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        if let Some(cid) = wait_for_claim(server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt") {
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

    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");
    let rc = unsafe { ffi::ec_rt_bringup_preop(cif.as_ptr(), cycle_ns, rt_cpu, rt_prio) };
    if rc != 0 {
        bringup_fail(&mut server, rc);
    }

    let run_limits: (u32, u16) = {
        let mut ferr = 0u32;
        let mut tmo = 0u16;
        let mut tq = 0u16;
        let rc = unsafe { ffi::ec_rt_read_limits(&mut ferr, &mut tmo, &mut tq) };
        if rc != 0 {
            eprintln!("ec-rt: SDO read of protection limits failed rc={rc} — aborting bringup");
            unsafe { ffi::ec_rt_shutdown() };
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
                unsafe { ffi::ec_rt_shutdown() };
                std::process::exit(1);
            }
            eprintln!(
                "ec-rt: session limits applied: 6065h={} 6072h={}",
                run.0, run.1
            );
        }
        run
    };

    let rc = unsafe { ffi::ec_rt_bringup_finish() };
    if rc != 0 {
        bringup_fail(&mut server, rc);
    }
    eprintln!("ec-rt: drive parked (Ready-to-Switch-On, no torque)");

    match wait_for_claim_pumping(
        &mut server,
        std::time::Instant::now() + std::time::Duration::from_secs(5),
        &SIGTERM_RECEIVED,
        "ec-rt",
        &mut || {
            let mut toff = 0i64;
            unsafe { ffi::ec_rt_park_cycle(&mut toff) };
        },
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
    let mut capture = Capture::new();
    let mut cycle_index: u64 = 0;
    let mailbox = MailboxWorker::spawn(FfiSdoBus, |ferr_counts, torque_tenth_pct| unsafe {
        ffi::ec_rt_write_limits(ferr_counts, torque_tenth_pct)
    });
    let mut pending_starts: Vec<(u32, String, PendingStart)> = Vec::new();
    let mut pending_stops: Vec<(u32, PendingStop)> = Vec::new();
    let mut prdiv = 0u64;
    let mut ff_saturation = 0u32;
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
                Command::StartCapture {
                    correlation_id,
                    msg,
                } => {
                    let pending = capture.start_async(CaptureConfig {
                        path: msg.path.clone(),
                        started_utc: msg.started_utc.clone(),
                        drive_name: msg.drive_name.clone(),
                        cycle_ns,
                        counts_per_mm,
                        started_mono_ns: monotonic_ns(),
                    });
                    pending_starts.push((correlation_id, msg.path, pending));
                }
                Command::StopCapture { correlation_id } => {
                    pending_stops.push((correlation_id, capture.stop_async()));
                }
                Command::ResumeStream { correlation_id } => {
                    server.respond(&resume_stream_response_frame(correlation_id, 0));
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
                    mailbox.submit(MailboxRequest::WriteLimits {
                        correlation_id,
                        ferr_counts: msg.following_error_counts,
                        torque_tenth_pct: msg.max_torque_tenth_pct,
                        restore: false,
                    });
                }
                Command::RestoreDriveLimits { correlation_id } => {
                    mailbox.submit(MailboxRequest::WriteLimits {
                        correlation_id,
                        ferr_counts: run_limits.0,
                        torque_tenth_pct: run_limits.1,
                        restore: true,
                    });
                }
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    mailbox.submit(MailboxRequest::SdoRead {
                        correlation_id,
                        msg,
                    });
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    mailbox.submit(MailboxRequest::SdoWrite {
                        correlation_id,
                        msg,
                    });
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let mut start_idx = 0;
        while start_idx < pending_starts.len() {
            match pending_starts[start_idx].2.try_take() {
                Some(rc) => {
                    let (correlation_id, path, pending) = pending_starts.remove(start_idx);
                    if rc != 0 && pending.claimed() {
                        capture.clear_failed_start();
                    }
                    eprintln!("ec-rt: StartCapture path={path} rc={rc}");
                    server.respond(&start_capture_response_frame(correlation_id, rc));
                }
                None => start_idx += 1,
            }
        }

        let mut stop_idx = 0;
        while stop_idx < pending_stops.len() {
            match pending_stops[stop_idx].1.try_take() {
                Some(out) => {
                    let (correlation_id, _) = pending_stops.remove(stop_idx);
                    eprintln!(
                        "ec-rt: StopCapture result={} samples={} overflow={:?}",
                        out.result, out.samples, out.overflow_cycle
                    );
                    server.respond(&stop_capture_response_frame(
                        correlation_id,
                        out.result,
                        out.samples,
                        out.overflow_cycle
                            .unwrap_or(StopCaptureResponse::NO_OVERFLOW),
                    ));
                }
                None => stop_idx += 1,
            }
        }

        while let Some(reply) = mailbox.try_recv() {
            match reply {
                MailboxReply::SdoRead {
                    correlation_id,
                    resp,
                } => {
                    if resp.result != 0 {
                        eprintln!("ec-rt: SdoRead failed result={}", resp.result);
                    }
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
                }
                MailboxReply::SdoWrite {
                    correlation_id,
                    resp,
                } => {
                    if resp.result != 0 {
                        eprintln!("ec-rt: SdoWrite failed result={}", resp.result);
                    }
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
                }
                MailboxReply::WriteLimits {
                    correlation_id,
                    rc,
                    ferr_counts,
                    torque_tenth_pct,
                    restore,
                } => {
                    let what = if restore {
                        "RestoreDriveLimits"
                    } else {
                        "SetDriveLimits"
                    };
                    if rc != 0 {
                        eprintln!(
                            "ec-rt: {what} SDO write failed rc={rc} \
                             ferr={ferr_counts} tq={torque_tenth_pct}"
                        );
                    } else {
                        eprintln!("ec-rt: {what} applied ferr={ferr_counts} tq={torque_tenth_pct}");
                    }
                    let frame = if restore {
                        restore_drive_limits_response_frame(correlation_id, rc)
                    } else {
                        set_drive_limits_response_frame(correlation_id, rc)
                    };
                    server.respond(&frame);
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
                    ff_saturation,
                ));
                unsafe {
                    ffi::ec_rt_disable();
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }
        }

        let mut motion_active = false;
        if gate.state() == TorqueState::Enabled {
            if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = ring.sample(now) {
                let map = cmap.get_or_insert_with(|| {
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, f64::from(pos_mm))
                });
                let counts = map.target_counts(f64::from(pos_mm));
                let vel_offset = if velocity_ff {
                    (f64::from(vel_mm_s) * counts_per_mm).round() as i32
                } else {
                    0
                };
                let torque_offset = match &dynamics {
                    Some(model) => {
                        let raw = model.torque_ff(0, &[acc_mm_s2], &[vel_mm_s]);
                        if !raw.is_finite() {
                            eprintln!(
                                "ec-rt: FAULT non-finite torque FF (acc={acc_mm_s2} vel={vel_mm_s}) — disabling"
                            );
                            server.respond(&status_heartbeat_frame(
                                ENGINE_STATE_FAULT,
                                0,
                                &[ring.retired_count()],
                                ff_saturation,
                            ));
                            unsafe {
                                ffi::ec_rt_disable();
                                ffi::ec_rt_shutdown();
                            }
                            std::process::exit(1);
                        }
                        clamp_torque(raw, torque_clamp_tenths, &mut ff_saturation)
                    }
                    None => 0,
                };
                unsafe {
                    ffi::ec_rt_set_target_position(counts);
                    ffi::ec_rt_set_velocity_offset(vel_offset);
                    ffi::ec_rt_set_torque_offset(torque_offset);
                }
                motion_active = true;
            } else {
                cmap = None;
                unsafe {
                    ffi::ec_rt_set_velocity_offset(0);
                    ffi::ec_rt_set_torque_offset(0);
                }
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
                ff_saturation,
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
                ff_saturation,
            ));
            last_sent_retired = ring.retired_count();
            heartbeat_sent = true;
        }

        cycle_index += 1;
        if capture.is_active() {
            let mut t = ffi::EcTelemetry::default();
            unsafe { ffi::ec_rt_get_telemetry(&mut t) };
            let mut flags = 0u8;
            if gate.state() == TorqueState::Enabled {
                flags |= FLAG_TORQUE_ENABLED;
            }
            if motion_active {
                flags |= FLAG_MOTION_ACTIVE;
            }
            capture.push(CaptureRecord {
                cycle_index,
                flags,
                drive: DriveSample {
                    target_counts: t.target_position,
                    position_demand: t.position_demand,
                    position_actual: t.position_actual,
                    following_error: t.following_error,
                    torque_actual: t.torque_actual,
                    statusword: t.statusword,
                    error_code: t.error_code,
                },
            });
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
                let mut al_state = 0u16;
                let mut al_code = 0u16;
                unsafe { ffi::ec_rt_al_status(&mut al_state, &mut al_code) };
                eprintln!(
                    "ec-rt: working counter {wkc} (expected 3), \
                     consecutive_bad={wkc_consecutive} — bus lost after \
                     {} consecutive bad cycles, halting \
                     (slave AL state=0x{al_state:02x} status_code=0x{al_code:04x}; \
                     0x001b=SM watchdog, 0x001a/0x002c/0x0030=DC sync)",
                    kalico_ethercat_rt::claim::WKC_CONSECUTIVE_LOSS_LIMIT
                );
                unsafe { ffi::ec_rt_dump_al_state() };
                break;
            }
        }

        let current_retired = ring.retired_count();
        let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(
                engine_state,
                0,
                &[current_retired],
                ff_saturation,
            ));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt: heartbeat retired_count={current_retired}");
            }
        }

        prdiv += 1;
        if prdiv >= telemetry_period {
            prdiv = 0;
            let (sw, pos, ferr, tq_act) = unsafe {
                (
                    ffi::ec_rt_get_statusword(),
                    ffi::ec_rt_get_position_actual(),
                    ffi::ec_rt_get_following_error(),
                    ffi::ec_rt_get_torque_actual(),
                )
            };
            eprintln!(
                "ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{drive_err:04x} pos={pos} ferr={ferr} toff={toff} \
                 ring_len={} retired={} tq_act={tq_act} ff_sat={ff_saturation}",
                ring.is_empty() as u8 ^ 1,
                current_retired,
            );
            if gate.state() == TorqueState::Faulted {
                server.respond(&status_heartbeat_frame(
                    0,
                    latched_drive_err,
                    &[current_retired],
                    ff_saturation,
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
