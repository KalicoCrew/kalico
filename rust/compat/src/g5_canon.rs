//! G5 canonicalization: resolve implicit I/J from the RS274NGC modal chain rule.
//!
//! When a G5 immediately follows another G5 with I and J both omitted, the
//! spec defaults them to −(`prev_P`, `prev_Q`) for C¹ continuity across the
//! junction.  This module encapsulates that logic so the main converter loop
//! stays clean.

/// Resolve the I, J, P, Q parameters for a G5 command.
///
/// # Parameters
/// - `params` — the raw `Params` block from the lexer for this G5 line.
/// - `prev_pq` — the `[P, Q]` offsets from the immediately-preceding G5 move,
///   if it exists and was not broken by an intervening motion command.
///
/// # Returns
/// `Ok((i, j, p, q))` on success, or `Err(&'static str)` describing the
/// violation.
///
/// # RS274NGC rules implemented
/// | I present | J present | `prev_pq` | outcome |
/// |-----------|-----------|---------|---------|
/// | yes       | yes       | any     | use I, J directly |
/// | no        | no        | Some    | I = −`prev_P`, J = −`prev_Q` |
/// | no        | no        | None    | Err — chain broken, I/J required |
/// | mixed     | —         | any     | Err — I and J must come together |
///
/// P and Q are always required regardless of the I/J case.
pub fn canonicalize_g5(
    params: &gcode::Params,
    prev_pq: Option<[f64; 2]>,
) -> Result<(f64, f64, f64, f64), &'static str> {
    let i_opt = params.i();
    let j_opt = params.j();

    let (i, j) = match (i_opt, j_opt) {
        (Some(i), Some(j)) => (i, j),
        (None, None) => match prev_pq {
            Some([prev_p, prev_q]) => (-prev_p, -prev_q),
            None => {
                return Err("G5: I/J omitted with no previous G5 in chain");
            }
        },
        _ => {
            return Err("G5: I and J must both be present or both omitted");
        }
    };

    let p = params.p().ok_or("G5: P is required")?;
    let q = params.q().ok_or("G5: Q is required")?;

    Ok((i, j, p, q))
}
