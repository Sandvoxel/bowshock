//! Reusable UI widgets of the Bowshock kit (design/components). Screens compose these; none of them knows about the
//! rocket model. The kit's widgets are consumed screen by screen, so unused ones are allowed here.
#![allow(dead_code)]

pub mod button;
pub mod chip;
pub mod dashed;
pub mod field;
pub mod gauge;
pub mod glyph;
pub mod ground_track;
pub mod icons;
pub mod pinch;
pub mod plot;
pub mod segmented;
pub mod spinner;
pub mod status;
pub mod unit_input;
