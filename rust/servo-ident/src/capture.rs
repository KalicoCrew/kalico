#[derive(Debug)]
pub struct Capture {
    pub t: Vec<f64>,
    pub acc: Vec<Vec<f64>>,
    pub vel: Vec<Vec<f64>>,
    pub torque: Vec<Vec<f64>>,
}

#[derive(Debug)]
pub enum CaptureError {
    MissingColumn(String),
    Malformed { line: usize, what: String },
    TooShort,
}

pub fn parse_capture_csv(text: &str, axes: &[&str]) -> Result<Capture, CaptureError> {
    let mut lines = text.lines().enumerate();
    let (_, header) = lines.next().ok_or(CaptureError::TooShort)?;
    let cols: Vec<&str> = header.split(',').map(str::trim).collect();

    let col = |name: &str| {
        cols.iter()
            .position(|c| *c == name)
            .ok_or_else(|| CaptureError::MissingColumn(name.to_string()))
    };

    let t_col = col("t")?;
    let target_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("target_{a}")))
        .collect::<Result<_, _>>()?;
    let torque_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("torque_{a}")))
        .collect::<Result<_, _>>()?;

    let mut t: Vec<f64> = Vec::new();
    let mut target: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];
    let mut torque: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];

    for (lineno, line) in lines {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        let num = |idx: usize| -> Result<f64, CaptureError> {
            fields
                .get(idx)
                .and_then(|f| f.parse().ok())
                .ok_or_else(|| CaptureError::Malformed {
                    line: lineno + 1,
                    what: format!("column {idx}"),
                })
        };
        t.push(num(t_col)?);
        for (a, (&tc, &qc)) in target_cols.iter().zip(&torque_cols).enumerate() {
            target[a].push(num(tc)?);
            torque[a].push(num(qc)?);
        }
    }

    let n = t.len();
    if n < 5 {
        return Err(CaptureError::TooShort);
    }

    let diff = |x: &[f64]| -> Vec<f64> {
        let mut d = vec![0.0; n];
        for k in 1..n - 1 {
            let dt = t[k + 1] - t[k - 1];
            d[k] = if dt > 0.0 { (x[k + 1] - x[k - 1]) / dt } else { 0.0 };
        }
        d[0] = d[1];
        d[n - 1] = d[n - 2];
        d
    };

    let vel: Vec<Vec<f64>> = target.iter().map(|x| diff(x)).collect();
    let acc: Vec<Vec<f64>> = vel.iter().map(|v| diff(v)).collect();

    Ok(Capture { t, acc, vel, torque })
}
