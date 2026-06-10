use crate::model::{PhysicalParams, COULOMB_DEADBAND_MM_S};

pub fn render_profile(p: &PhysicalParams, axes: &[&str], rms_residual: &[f64]) -> String {
    let fmt_vec = |v: &[f64]| {
        let inner: Vec<String> = v.iter().map(|x| format!("{x}")).collect();
        format!("[{}]", inner.join(", "))
    };
    let mass_rows: Vec<String> = p.mass.iter().map(|row| fmt_vec(row)).collect();
    let axes_q: Vec<String> = axes.iter().map(|a| format!("\"{a}\"")).collect();
    format!(
        "version = 1\naxes = [{}]\nmass = [{}]\nviscous = {}\ncoulomb_fwd = {}\ncoulomb_rev = {}\ncoulomb_deadband_mm_s = {COULOMB_DEADBAND_MM_S}\nfit_rms_residual = {}\n",
        axes_q.join(", "),
        mass_rows.join(", "),
        fmt_vec(&p.viscous),
        fmt_vec(&p.coulomb_fwd),
        fmt_vec(&p.coulomb_rev),
        fmt_vec(rms_residual),
    )
}

/// `m_diag`: fitted diagonal mass entry, units (0.1% rated) / (mm/s²).
/// `rated_torque_nm`: motor rated torque in N·m.
/// `rot_dist_mm`: linear distance per revolution in mm/rev.
/// `rotor_inertia_kgm2`: rotor moment of inertia in kg·m².
///
/// Returns the drive load-inertia-ratio C00.06 in percent:
/// `(J_total - J_rotor) / J_rotor * 100`.
pub fn c0006_recommendation(
    m_diag: f64,
    rated_torque_nm: f64,
    rot_dist_mm: f64,
    rotor_inertia_kgm2: f64,
) -> f64 {
    let j_total = m_diag * (rated_torque_nm / 1000.0) * rot_dist_mm / (2.0 * std::f64::consts::PI);
    (j_total - rotor_inertia_kgm2) / rotor_inertia_kgm2 * 100.0
}
