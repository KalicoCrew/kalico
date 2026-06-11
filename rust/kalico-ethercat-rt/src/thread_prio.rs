//! New threads inherit the creator's SCHED_FIFO policy, priority, and CPU
//! pin — and go_realtime runs on the thread that later spawns every helper.
//! Equal-priority FIFO threads never preempt each other, so an inherited-RT
//! helper that burns or blocks CPU on the DC core starves the cycle and the
//! slave latches ErC1.1 (AL 0x001a/0x001b). Every helper thread calls this
//! first: SCHED_OTHER is preempted by the FIFO DC thread unconditionally.

#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
pub fn demote_to_normal_scheduling() {
    unsafe {
        let param = libc::sched_param { sched_priority: 0 };
        let rc = libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_OTHER, &param);
        if rc != 0 {
            panic!("ec-rt helper thread: SCHED_OTHER demotion failed (errno {rc})");
        }
        let mut cpus: libc::cpu_set_t = std::mem::zeroed();
        for cpu in 0..(8 * std::mem::size_of::<libc::cpu_set_t>()) {
            libc::CPU_SET(cpu, &mut cpus);
        }
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &cpus);
    }
}

#[cfg(not(target_os = "linux"))]
pub fn demote_to_normal_scheduling() {}
