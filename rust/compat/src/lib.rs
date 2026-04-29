//! Offline G-code compatibility layer.
//!
//! Converts legacy G0/G1/G2/G3/G5.1 mixed G-code to G5-only output consumable
//! by the kalico live pipeline. This is a pure G-code-text → G-code-text
//! transform; it does not import geometry/nurbs planner types.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod arc;
pub mod collinear;
pub mod converter;
pub mod corner;
pub mod degree_elev;
pub mod emit;
pub mod fitter;
pub mod g5_canon;
pub mod hausdorff;
pub mod modal;
pub mod run;
