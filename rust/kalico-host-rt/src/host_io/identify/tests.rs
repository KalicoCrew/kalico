use std::collections::VecDeque;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use super::{IdentifyOutcome, wait_for_klipper_frame};
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::host_io::wire::build_frame;

struct ScriptedPort {
    rx: VecDeque<Vec<u8>>,
}

impl ScriptedPort {
    fn boxed(chunks: Vec<Vec<u8>>) -> Box<dyn serialport::SerialPort> {
        Box::new(Self { rx: chunks.into() })
    }
}

impl Read for ScriptedPort {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match self.rx.pop_front() {
            Some(chunk) => {
                buf[..chunk.len()].copy_from_slice(&chunk);
                Ok(chunk.len())
            }
            None => Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "scripted rx exhausted",
            )),
        }
    }
}

impl Write for ScriptedPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl serialport::SerialPort for ScriptedPort {
    fn name(&self) -> Option<String> {
        None
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(250_000)
    }
    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        Ok(serialport::DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        Ok(serialport::FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<serialport::Parity> {
        Ok(serialport::Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        Ok(serialport::StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(0)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "scripted port cannot clone",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

fn identify_response_frame(seq: u8, offset: u8, data: &[u8]) -> Vec<u8> {
    let mut payload = vec![0u8, offset, data.len() as u8];
    payload.extend_from_slice(data);
    build_frame(&payload, seq)
}

fn unsolicited_stats_frame(seq: u8) -> Vec<u8> {
    build_frame(&[0x81, 0x05, 0x11, 0x84, 0x91], seq)
}

fn empty_ack_frame(seq: u8) -> Vec<u8> {
    build_frame(&[], seq)
}

fn run(chunks: Vec<Vec<u8>>, sent_seq_nibble: u8) -> IdentifyOutcome {
    let mut io = SerialFrameIo::new(ScriptedPort::boxed(chunks));
    let mut mcu_recv_abs = 0u64;
    wait_for_klipper_frame(
        &mut io,
        Instant::now() + Duration::from_millis(50),
        &mut mcu_recv_abs,
        Some(sent_seq_nibble),
    )
    .expect("scripted port never errors fatally")
}

#[test]
fn response_behind_unsolicited_frame_in_same_batch_is_found() {
    let mut batch = unsolicited_stats_frame(2);
    batch.extend_from_slice(&identify_response_frame(2, 0, b"x"));
    let outcome = run(vec![batch], 0);
    assert!(matches!(outcome, IdentifyOutcome::Response(_)));
}

#[test]
fn unsolicited_content_frame_alone_is_not_a_nak() {
    let outcome = run(vec![unsolicited_stats_frame(2)], 0);
    assert!(matches!(outcome, IdentifyOutcome::Timeout));
}

#[test]
fn empty_frame_with_foreign_seq_is_a_nak() {
    let outcome = run(vec![empty_ack_frame(2)], 0);
    assert!(matches!(outcome, IdentifyOutcome::Nak));
}

#[test]
fn empty_frame_matching_sent_seq_is_a_stale_ack_not_a_nak() {
    let outcome = run(vec![empty_ack_frame(3)], 3);
    assert!(matches!(outcome, IdentifyOutcome::Timeout));
}

#[test]
fn nak_seq_state_is_adopted_from_batch() {
    let mut io = SerialFrameIo::new(ScriptedPort::boxed(vec![empty_ack_frame(7)]));
    let mut mcu_recv_abs = 0u64;
    let outcome = wait_for_klipper_frame(
        &mut io,
        Instant::now() + Duration::from_millis(50),
        &mut mcu_recv_abs,
        Some(0),
    )
    .expect("scripted port never errors fatally");
    assert!(matches!(outcome, IdentifyOutcome::Nak));
    assert_eq!(mcu_recv_abs, 7);
}
