use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::transport::TransportError;

pub struct TcpSerialPort {
    stream: TcpStream,
    name: String,
    timeout: Duration,
}

impl TcpSerialPort {
    pub fn connect(addr: &str) -> Result<Self, TransportError> {
        let stream = TcpStream::connect(addr).map_err(|e| {
            TransportError::Io(io::Error::other(format!(
                "TcpSerialPort::connect({addr}): {e}"
            )))
        })?;
        // Not setting TCP_NODELAY: Renode's USART2 TCP bridge overruns when Nagle is off.
        let default_timeout = Duration::from_millis(100);
        stream
            .set_read_timeout(Some(default_timeout))
            .map_err(|e| {
                TransportError::Io(io::Error::other(format!(
                    "TcpSerialPort: set_read_timeout: {e}"
                )))
            })?;
        // 5 s write timeout: Renode stalls under heavy load but never refuses inbound; 100 ms aborted mid-frame.
        stream
            .set_write_timeout(Some(Duration::from_secs(5)))
            .map_err(|e| {
                TransportError::Io(io::Error::other(format!(
                    "TcpSerialPort: set_write_timeout: {e}"
                )))
            })?;
        Ok(Self {
            stream,
            name: format!("tcp://{addr}"),
            timeout: default_timeout,
        })
    }
}

impl Read for TcpSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.stream.read(buf)
    }
}

impl Write for TcpSerialPort {
    // Throttled chunk-write: Renode 1.16 STM32F7_USART raises ORE on every byte after the first
    // when a large frame arrives faster than USART2_IRQHandler drains RDR. OVRDIS (CR3 bit 12)
    // would suppress ORE but Renode 1.16 ignores it. Chunk/delay tunable via
    // KALICO_TCP_WRITE_CHUNK / KALICO_TCP_WRITE_DELAY_US.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        use std::sync::OnceLock;
        static CHUNK: OnceLock<usize> = OnceLock::new();
        static DELAY: OnceLock<Duration> = OnceLock::new();
        let chunk = *CHUNK.get_or_init(|| {
            std::env::var("KALICO_TCP_WRITE_CHUNK")
                .ok()
                .and_then(|s| s.parse::<usize>().ok())
                .filter(|&n| n > 0)
                .unwrap_or(1)
        });
        let delay = *DELAY.get_or_init(|| {
            let us: u64 = std::env::var("KALICO_TCP_WRITE_DELAY_US")
                .ok()
                .and_then(|s| s.parse::<u64>().ok())
                .unwrap_or(100);
            Duration::from_micros(us)
        });

        if buf.len() <= chunk {
            return self.stream.write(buf);
        }
        for piece in buf.chunks(chunk) {
            self.stream.write_all(piece)?;
            if delay > Duration::ZERO {
                std::thread::sleep(delay);
            }
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

fn no_support(method: &'static str) -> serialport::Error {
    serialport::Error::new(
        serialport::ErrorKind::Io(io::ErrorKind::Unsupported),
        format!("TcpSerialPort: {method} is not supported"),
    )
}

impl SerialPort for TcpSerialPort {
    fn name(&self) -> Option<String> {
        Some(self.name.clone())
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(250_000)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _data_bits: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _flow_control: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _parity: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _stop_bits: StopBits) -> serialport::Result<()> {
        Ok(())
    }

    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        // 100 ms floor: macOS SO_RCVTIMEO rejects sub-10 ms values under syscall pressure (EINVAL).
        let effective = timeout.max(Duration::from_millis(100));
        if effective == self.timeout {
            return Ok(());
        }
        self.stream.set_read_timeout(Some(effective)).map_err(|e| {
            serialport::Error::new(
                serialport::ErrorKind::Io(e.kind()),
                format!("TcpSerialPort: set_read_timeout: {e}"),
            )
        })?;
        self.timeout = effective;
        Ok(())
    }

    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        Err(no_support("write_request_to_send"))
    }
    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        Err(no_support("write_data_terminal_ready"))
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_clear_to_send"))
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_data_set_ready"))
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_ring_indicator"))
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_carrier_detect"))
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }

    fn clear(&self, _buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        let cloned = self.stream.try_clone().map_err(|e| {
            serialport::Error::new(
                serialport::ErrorKind::Io(e.kind()),
                format!("TcpSerialPort: try_clone: {e}"),
            )
        })?;
        Ok(Box::new(TcpSerialPort {
            stream: cloned,
            name: self.name.clone(),
            timeout: self.timeout,
        }))
    }

    fn set_break(&self) -> serialport::Result<()> {
        Err(no_support("set_break"))
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Err(no_support("clear_break"))
    }
}
