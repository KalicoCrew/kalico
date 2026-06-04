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
