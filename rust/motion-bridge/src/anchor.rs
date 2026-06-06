const CONTIGUITY_EPS: f64 = 1e-6;
const DEFAULT_LEAD_SECS: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct SegmentLate {
    pub scheduled_host: f64,
    pub host_now: f64,
    pub gap_s: f64,
    pub seg_t_start: f64,
}

pub struct Anchor {
    t0: Option<f64>,
    last_t_end: f64,
    lead_secs: f64,
}

impl Anchor {
    pub fn new() -> Self {
        Self {
            t0: None,
            last_t_end: 0.0,
            lead_secs: DEFAULT_LEAD_SECS,
        }
    }

    pub fn anchor_segment(
        &mut self,
        seg_t_start: f64,
        seg_t_end: f64,
        host_now: f64,
    ) -> Result<(f64, bool), SegmentLate> {
        let reanchor = match self.t0 {
            None => true,
            Some(t0) => {
                let timeline_reset = seg_t_start + CONTIGUITY_EPS < self.last_t_end;
                let starvation = t0 + seg_t_start < host_now;

                if starvation && !timeline_reset {
                    let scheduled_host = t0 + seg_t_start;
                    let gap_s = host_now - scheduled_host;
                    return Err(SegmentLate {
                        scheduled_host,
                        host_now,
                        gap_s,
                        seg_t_start,
                    });
                }

                timeline_reset
            }
        };

        if reanchor {
            let condition = match self.t0 {
                None => "first",
                Some(_) => "backward-jump",
            };
            self.t0 = Some(host_now + self.lead_secs - seg_t_start);
            let t0 = self.t0.unwrap();
            tracing::info!(host_now, t0, seg_t_start, condition, "[anchor-decision]");
        }
        self.last_t_end = seg_t_end;
        Ok((self.t0.unwrap(), reanchor))
    }
}

#[cfg(test)]
mod tests;
