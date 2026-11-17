use super::plan::{lift_key, lift_nodes, nodes, table_key, Plan, OUT_OF_RANGE};
use bowshock_model::{CfdLiftTable, CfdTable};
use std::time::{SystemTime, UNIX_EPOCH};

/// How a stored 3D stability solve relates to the rocket and the Mach range on screen.
pub fn lift_status(plan: &Result<Plan, String>, lift: Option<&CfdLiftTable>, mach: (f64, f64)) -> Status {
    let plan = match plan {
        Ok(p) => p,
        Err(why) => return Status::Unsupported(why.clone()),
    };
    let Some(nodes) = lift_nodes(mach) else {
        return Status::Unsupported(format!("the 3D solve covers Mach up to 3; the Mach range is {:.2} to {:.2}", mach.0, mach.1));
    };
    match lift {
        None => Status::Missing,
        Some(l) if l.key == lift_key(plan.geometry, &nodes) => Status::Fresh,
        Some(_) => Status::Stale,
    }
}

/// How a stored table relates to the rocket and the conditions on screen.
#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    /// The geometry cannot be solved; the reason.
    Unsupported(String),
    /// There is no table for this stack.
    Missing,
    /// The table was solved for these geometry and conditions.
    Fresh,
    /// A table exists but was solved for other geometry or conditions.
    Stale,
}

/// Compares the stored `table` with what the rocket and the conditions give now.
pub fn status(plan: &Result<Plan, String>, table: Option<&CfdTable>, altitude: f64, mach: (f64, f64)) -> Status {
    let plan = match plan {
        Ok(p) => p,
        Err(why) => return Status::Unsupported(why.clone()),
    };
    let Some(nodes) = nodes(mach) else {
        return Status::Unsupported(format!("{OUT_OF_RANGE}; the Mach range is {:.2} to {:.2}", mach.0, mach.1));
    };
    match table {
        None => Status::Missing,
        Some(t) if t.key == table_key(plan.geometry, altitude, &nodes) => Status::Fresh,
        Some(_) => Status::Stale,
    }
}

/// Whether `table` covers any of the Mach `range` (a stale table over other Mach numbers cannot stand in for the plots).
pub fn covers(table: &CfdTable, range: (f64, f64)) -> bool {
    let (lo, hi) = (range.0.min(range.1), range.0.max(range.1));
    match (table.points.first(), table.points.last()) {
        (Some(a), Some(b)) => hi.min(b.mach) - lo.max(a.mach) >= 0.05,
        _ => false,
    }
}

/// Seconds since the Unix epoch.
pub fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// `just now`, `12 s ago`, `6 min ago`, `3 h ago`, `2 d ago` for a table solved at `solved_at`.
pub fn ago(solved_at: u64) -> String {
    let s = now().saturating_sub(solved_at);
    match s {
        0..=4 => "just now".to_string(),
        5..=59 => format!("{s} s ago"),
        60..=3599 => format!("{} min ago", s / 60),
        3600..=86_399 => format!("{} h ago", s / 3600),
        _ => format!("{} d ago", s / 86_400),
    }
}
