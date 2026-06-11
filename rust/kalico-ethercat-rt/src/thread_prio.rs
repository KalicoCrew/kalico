//! New threads inherit the creator's SCHED_FIFO policy, priority, and CPU
//! pin — and go_realtime runs on the thread that later spawns every helper.
//! Two placement rules follow:
//!
//! * Helpers that never touch the SOEM socket (file I/O, channel drains) call
//!   [`demote_to_normal_scheduling`]: SCHED_OTHER on the housekeeping cores,
//!   so the FIFO DC thread preempts them unconditionally and they stay off
//!   the isolated (`isolcpus`/`nohz_full`) cores.
//!
//! * Helpers that share the SOEM socket with the DC loop (CoE mailbox
//!   traffic) call [`assume_companion_rt_scheduling`]. SOEM stashes frames it
//!   receives on behalf of another thread; a SCHED_OTHER helper descheduled
//!   between the kernel read and that stash traps the DC thread's process
//!   data frame for a whole scheduling latency, the cycle reads WKC -1, and
//!   two such cycles halt the endpoint (bench 2026-06-11: every ErC1.1 that
//!   evening had a mailbox SDO in flight). SCHED_FIFO below the DC priority
//!   on an isolated companion core closes the window: nothing ordinary can
//!   deschedule the helper, and it can never starve the DC core.

#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
pub fn demote_to_normal_scheduling() {
    unsafe {
        let param = libc::sched_param { sched_priority: 0 };
        let rc = libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_OTHER, &param);
        if rc != 0 {
            panic!("ec-rt helper thread: SCHED_OTHER demotion failed (errno {rc})");
        }
        let isolated = isolated_cpus();
        let mut cpus: libc::cpu_set_t = std::mem::zeroed();
        let mut any = false;
        for cpu in 0..(8 * std::mem::size_of::<libc::cpu_set_t>()) {
            if !isolated.contains(&cpu) {
                libc::CPU_SET(cpu, &mut cpus);
                any = true;
            }
        }
        if !any {
            for cpu in 0..(8 * std::mem::size_of::<libc::cpu_set_t>()) {
                libc::CPU_SET(cpu, &mut cpus);
            }
        }
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &cpus);
    }
}

/// SCHED_FIFO at `priority` pinned to `cpu`. For helper threads that share
/// the SOEM socket with the DC loop: `priority` must sit below the DC
/// thread's (so it can never starve the cycle) and below the threaded NIC
/// IRQ's (so frame delivery preempts the helper's busy-poll), and `cpu`
/// should be an isolated core the DC thread does not own.
///
/// Panics on failure: the endpoint only reaches this after go_realtime
/// succeeded, so the capabilities are already proven present.
#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
pub fn assume_companion_rt_scheduling(cpu: usize, priority: i32) {
    unsafe {
        let mut cpus: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(cpu, &mut cpus);
        if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &cpus) != 0 {
            panic!(
                "ec-rt companion thread: pin to CPU {cpu} failed (errno {})",
                std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
            );
        }
        let param = libc::sched_param {
            sched_priority: priority,
        };
        let rc = libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param);
        if rc != 0 {
            panic!("ec-rt companion thread: SCHED_FIFO({priority}) failed (errno {rc})");
        }
    }
}

#[cfg(target_os = "linux")]
fn isolated_cpus() -> Vec<usize> {
    parse_cpu_list(
        std::fs::read_to_string("/sys/devices/system/cpu/isolated")
            .unwrap_or_default()
            .trim(),
    )
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn parse_cpu_list(list: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in list.split(',').filter(|p| !p.is_empty()) {
        match part.split_once('-') {
            Some((lo, hi)) => {
                if let (Ok(lo), Ok(hi)) = (lo.trim().parse::<usize>(), hi.trim().parse::<usize>()) {
                    cpus.extend(lo..=hi);
                }
            }
            None => {
                if let Ok(cpu) = part.trim().parse() {
                    cpus.push(cpu);
                }
            }
        }
    }
    cpus
}

#[cfg(not(target_os = "linux"))]
pub fn demote_to_normal_scheduling() {}

#[cfg(not(target_os = "linux"))]
pub fn assume_companion_rt_scheduling(_cpu: usize, _priority: i32) {}

#[cfg(test)]
mod tests;
