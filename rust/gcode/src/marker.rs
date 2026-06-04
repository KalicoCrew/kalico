#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum MarkerKind {
    LayerChange { layer: Option<u32> },
    LayerType { name: Box<str> },
    EndOfPrint,
}

#[must_use]
pub fn match_comment(comment_line: &str) -> Option<MarkerKind> {
    let body = comment_line.strip_prefix(';')?.trim();

    if let Some(rest) = body.strip_prefix("LAYER:") {
        if let Ok(n) = rest.trim().parse::<u32>() {
            return Some(MarkerKind::LayerChange { layer: Some(n) });
        }
    }

    if body == "LAYER_CHANGE" {
        return Some(MarkerKind::LayerChange { layer: None });
    }

    if let Some(rest) = body.strip_prefix("TYPE:") {
        return Some(MarkerKind::LayerType {
            name: rest.trim().to_string().into_boxed_str(),
        });
    }

    let upper = body.to_ascii_uppercase();
    if upper == "END_OF_PRINT" || upper == "END OF PRINT" || upper.starts_with("END_GCODE") {
        return Some(MarkerKind::EndOfPrint);
    }

    None
}

#[cfg(test)]
mod tests;
