//! Usage: kalico-ethercat-rt-stub [--socket PATH]

use std::thread::sleep;
use std::time::Duration;

use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT, NUM_AXES};
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    identify_response_frame, push_pieces_response_frame, runtime_caps_response_frame,
    status_heartbeat_frame, Command,
};

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());

    let mut ring = AxisRing::new();
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket} (NO HARDWARE)");

    loop {
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
                    eprintln!(
                        "ec-rt-stub: PushPieces axis={} pieces={} pushed={} head={}",
                        msg.axis_idx, msg.piece_count, pushed, msg.new_head
                    );
                    let arrival_clock = monotonic_ns();
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
                Command::ClaimHandshake { .. } => {
                    eprintln!("ec-rt-stub: ClaimHandshake not yet implemented in stub");
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt-stub: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let now = monotonic_ns();

        let _ = ring.sample(now);

        if let Some(fault_val) = ring.take_fault() {
            let fault_code_u16 = (fault_val & 0xFFFF) as u16;
            eprintln!(
                "ec-rt-stub: FAULT latched fault_val=0x{fault_val:08x} code=0x{fault_code_u16:04x} \
                 — propagating to host via heartbeat, host must shut down"
            );
            let current_retired = ring.retired_count();
            server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                &[current_retired],
            ));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
        }

        let current_retired = ring.retired_count();
        let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(engine_state, &[current_retired]));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt-stub: heartbeat retired_count={current_retired}");
            }
        }

        sleep(Duration::from_millis(1));
    }
}
