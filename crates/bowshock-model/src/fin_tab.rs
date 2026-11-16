//! Through-the-wall fin tabs: where a fin's tab can reach, what it can be set to by default, and what is wrong with a tab as set.
//!
//! A fin glued to the outside of the tube only (fillets) and a fin whose root plate goes through a slot in the wall to be glued to
//! the motor mount inside (*through-the-wall*, TTW) have the same outside shape, so a tab changes the fin's mass and CG
//! ([`crate::mass`]) and nothing in the aerodynamics. For flutter, Martin's estimate (NACA TN 4197) already assumes a clamped root,
//! so a tab earns no bonus there either.

use crate::geometry::FinTab;
use crate::internal::InternalKind;
use crate::rocket::Rocket;
use crate::PartId;

/// Clearance, metres, below which two edges count as touching.
const EPS: f64 = 1e-6;

/// The motor mount under a fin's root and the tab that reaches it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TabTarget {
    /// The motor mount (inner tube).
    pub mount: PartId,
    /// Its outer radius, m.
    pub outer_radius: f64,
    /// Tab height that just reaches its outer surface: the body's outer radius under the fin less [`TabTarget::outer_radius`], m.
    pub height: f64,
    /// The tab to set when the fin has none: along the part of the root that lies over the mount.
    pub tab: FinTab,
}

/// What is wrong with a fin's tab (see [`Rocket::fin_tab_issues`]).
#[derive(Clone, Debug, PartialEq)]
pub enum TabIssue {
    /// The tab is longer than the root chord (`tab`, `root`, m).
    LongerThanRoot(f64, f64),
    /// The tab starts ahead of the root's leading edge or ends behind its trailing edge.
    PastRootEnds,
    /// The tab reaches past the body's axis.
    PastAxis,
    /// The tab passes through a centering ring (its name) instead of a slot.
    HitsRing(String),
    /// The tab reaches into the motor mount (its name) by this much (m).
    IntoMount(String, f64),
}

impl TabIssue {
    /// A sentence for the health list.
    pub fn message(&self) -> String {
        let mm = |v: f64| format!("{:.1} mm", v * 1000.0);
        match self {
            TabIssue::LongerThanRoot(tab, root) => format!("Fin tab is {} long, longer than the {} root chord", mm(*tab), mm(*root)),
            TabIssue::PastRootEnds => "Fin tab runs past the ends of the root chord".to_string(),
            TabIssue::PastAxis => "Fin tab is deeper than the body's radius".to_string(),
            TabIssue::HitsRing(ring) => format!("Fin tab hits {ring}: slot the ring or move the tab"),
            TabIssue::IntoMount(tube, by) => format!("Fin tab is {} deeper than the surface of {tube}", mm(*by)),
        }
    }
}

impl Rocket {
    /// The tube under fin group `id` that a through-the-wall tab is glued to, and the tab that reaches it: the first tube a tab meets
    /// going in from the wall. That is the widest inner tube of the fin's body tube that is at least as wide as the motor mount under
    /// the root (the one overlapping the root most) and covers at least as much of the root, which is the mount itself or a motor tube
    /// it sits in (as in HYPERLOC's `.ork`). `None` for a part that is not a fin, a fin with no mount under its root, or a tube as wide as
    /// the body.
    pub fn fin_tab_target(&self, id: PartId) -> Option<TabTarget> {
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.fin_tab_target(id);
        }
        let fin = self.radial(id).filter(|r| r.kind.is_fin())?;
        let (root_fore, root_aft) = self.part_span(id)?;
        let base = self.radial_base_radius(fin)?;
        // Every inner tube under the root: (id, outer radius, fore, aft, overlap with the root, holds a motor).
        let tubes: Vec<(PartId, f64, f64, f64, f64, bool)> = self
            .internals_of(fin.parent)
            .filter_map(|p| match p.kind {
                InternalKind::InnerTube { motor_mount, outer_diameter, length, .. } => {
                    let fore = self.part_station(p.id)?;
                    let overlap = (fore + length).min(root_aft) - fore.max(root_fore);
                    (overlap > EPS).then_some((p.id, outer_diameter / 2.0, fore, fore + length, overlap, motor_mount))
                }
                _ => None,
            })
            .collect();
        let mount = tubes.iter().filter(|t| t.5).max_by(|a, b| a.4.total_cmp(&b.4))?;
        let (tube, outer_radius, fore, aft, ..) = *tubes
            .iter()
            .filter(|t| t.1 >= mount.1 - EPS && t.4 >= mount.4 - EPS)
            .max_by(|a, b| a.1.total_cmp(&b.1).then(a.4.total_cmp(&b.4)))
            .unwrap_or(mount);
        let height = base - outer_radius;
        (height > EPS).then_some(())?;
        let (from, to) = (fore.max(root_fore), aft.min(root_aft));
        Some(TabTarget { mount: tube, outer_radius, height, tab: FinTab { height, length: to - from, position: from - root_fore } })
    }

    /// Sets the through-the-wall tab of fin group `id` (`None` removes it). Returns whether it was applied; a tab of a non-finite or
    /// negative size is refused, and only fins have one.
    pub fn set_fin_tab(&mut self, id: PartId, tab: Option<FinTab>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_fin_tab(id, tab)).unwrap_or(false);
        }
        if tab.is_some_and(|t| ![t.height, t.length, t.position].iter().all(|v| v.is_finite()) || t.height < 0.0 || t.length < 0.0 || t.position < 0.0) {
            return false;
        }
        match self.radials.iter_mut().find(|r| r.id == id && r.kind.is_fin()) {
            Some(r) => {
                r.tab = tab;
                true
            }
            None => false,
        }
    }

    /// Reaches fin group `id`'s tab to the motor mount under it ([`fin_tab_target`](Self::fin_tab_target)): an existing tab keeps its
    /// length and position and gets the height that just meets the mount; a fin without one gets the whole of the root that lies over
    /// the mount. Returns whether a mount was found.
    pub fn set_fin_tab_to_motor_mount(&mut self, id: PartId) -> bool {
        let Some(target) = self.fin_tab_target(id) else {
            return false;
        };
        let tab = match self.radial(id).and_then(|r| r.tab) {
            Some(own) => FinTab { height: target.height, ..own },
            None => target.tab,
        };
        self.set_fin_tab(id, Some(tab))
    }

    /// Gives fin group `id` a through-the-wall tab if it has none: onto the motor mount under it when there is one
    /// ([`TabTarget::tab`]), else a quarter of the body radius deep and half the root long, centred on it. Returns whether the fin has
    /// a tab afterwards.
    pub fn add_fin_tab(&mut self, id: PartId) -> bool {
        if self.radial(id).is_some_and(|r| r.kind.is_fin() && r.tab.is_some()) {
            return true;
        }
        let tab = self.fin_tab_target(id).map(|t| t.tab).or_else(|| {
            let fin = self.radial(id).filter(|r| r.kind.is_fin())?;
            let (base, root) = (self.radial_base_radius(fin)?, fin.kind.extent());
            Some(FinTab { height: 0.25 * base, length: 0.5 * root, position: 0.25 * root })
        });
        tab.is_some_and(|t| self.set_fin_tab(id, Some(t)))
    }

    /// What is wrong with fin group `id`'s tab, if it has one: longer than the root, beyond its ends, past the axis, through a
    /// centering ring, or deeper than the surface of an inner tube it is over.
    pub fn fin_tab_issues(&self, id: PartId) -> Vec<TabIssue> {
        if let Some((_, sub)) = self.scratch_of(id) {
            return sub.fin_tab_issues(id);
        }
        let mut out = Vec::new();
        let Some((fin, tab)) = self.radial(id).and_then(|r| r.tab.map(|t| (r, t))) else {
            return out;
        };
        let (Some((root_fore, _)), Some(base)) = (self.part_span(id), self.radial_base_radius(fin)) else {
            return out;
        };
        let root = fin.kind.extent();
        if tab.length > root + EPS {
            out.push(TabIssue::LongerThanRoot(tab.length, root));
        } else if tab.position < -EPS || tab.position + tab.length > root + EPS {
            out.push(TabIssue::PastRootEnds);
        }
        if tab.height >= base - EPS {
            out.push(TabIssue::PastAxis);
        }
        let (fore, aft) = tab.span(root_fore);
        let bottom = base - tab.height;
        for p in self.internals_of(fin.parent) {
            let Some(p_fore) = self.part_station(p.id) else {
                continue;
            };
            if aft.min(p_fore + p.kind.extent()) - fore.max(p_fore) <= EPS {
                continue;
            }
            let Some(size) = self.resolved(p.id) else {
                continue;
            };
            match p.kind {
                InternalKind::CenteringRing { .. } if bottom < size.outer_diameter / 2.0 - EPS && base > size.inner_diameter / 2.0 + EPS => {
                    out.push(TabIssue::HitsRing(p.display_name()));
                }
                InternalKind::InnerTube { outer_diameter, .. } if bottom < outer_diameter / 2.0 - EPS => {
                    out.push(TabIssue::IntoMount(p.display_name(), outer_diameter / 2.0 - bottom));
                }
                _ => {}
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{PartKind, RadialKind};
    use crate::material::MaterialId;

    /// A 100 mm tube (3 mm wall) of 400 mm with a 54 mm motor mount of 300 mm at its aft end, 3 fins 100 mm from the tail.
    fn rocket() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        let tube = r.insert(0, PartKind::BodyTube { diameter: 0.1, length: 0.4 }).unwrap();
        let mount = r
            .attach_internal(tube, 0.1, InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.002, length: 0.3, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let fins =
            r.attach_radial(tube, 0.2, 0.0, 3, RadialKind::FinSet { root_chord: 0.2, tip_chord: 0.08, span: 0.08, sweep: 0.08, thickness: 0.004 }).unwrap();
        (r, fins, mount)
    }

    #[test]
    fn to_motor_mount_reaches_the_mount_from_the_body_surface() {
        let (mut r, fins, mount) = rocket();
        let target = r.fin_tab_target(fins).unwrap();
        assert_eq!(target.mount, mount);
        // Body radius 50 mm less the mount's 27 mm.
        assert!((target.height - 0.023).abs() < 1e-9, "{}", target.height);
        // The mount starts 100 mm into the tube, the root 200 mm: the root lies wholly over the mount.
        assert!((target.tab.length - 0.2).abs() < 1e-9 && target.tab.position.abs() < 1e-9);
        assert!(r.set_fin_tab_to_motor_mount(fins));
        assert_eq!(r.radial(fins).unwrap().tab, Some(target.tab));
        assert!(r.fin_tab_issues(fins).is_empty());
        // An existing tab keeps its length and position and takes the height of the mount.
        assert!(r.set_fin_tab(fins, Some(FinTab { height: 0.01, length: 0.05, position: 0.03 })));
        assert!(r.set_fin_tab_to_motor_mount(fins));
        let tab = r.radial(fins).unwrap().tab.unwrap();
        assert!((tab.height - 0.023).abs() < 1e-12 && tab.length == 0.05 && tab.position == 0.03);
    }

    #[test]
    fn a_wider_motor_tube_around_the_mount_is_what_the_tab_reaches() {
        // As in HYPERLOC's file: the 54 mm mount (offset 100 mm, 300 mm long) sits in a 57.4 mm tube that covers the whole root.
        let (mut r, fins, mount) = rocket();
        let tube = r.parent_of(fins).unwrap();
        let sleeve = r
            .attach_internal(tube, 0.05, InternalKind::InnerTube { outer_diameter: 0.0574, thickness: 0.0015, length: 0.35, motor_mount: false, overhang: 0.0 })
            .unwrap();
        let target = r.fin_tab_target(fins).unwrap();
        assert_eq!(target.mount, sleeve);
        assert!((target.height - (0.05 - 0.0287)).abs() < 1e-12 && (target.outer_radius - 0.0287).abs() < 1e-12);
        assert!(r.set_fin_tab_to_motor_mount(fins) && r.fin_tab_issues(fins).is_empty());
        // A sleeve that covers less of the root than the mount does is not the target.
        r.set_internal_kind(sleeve, InternalKind::InnerTube { outer_diameter: 0.0574, thickness: 0.0015, length: 0.05, motor_mount: false, overhang: 0.0 });
        assert_eq!(r.fin_tab_target(fins).unwrap().mount, mount);
    }

    #[test]
    fn a_fin_with_no_mount_under_it_has_no_target() {
        let (mut r, fins, mount) = rocket();
        r.remove_internal(mount).unwrap();
        assert!(r.fin_tab_target(fins).is_none() && !r.set_fin_tab_to_motor_mount(fins));
        assert!(r.radial(fins).unwrap().tab.is_none());
        // Still a tab can be added: a quarter of the 50 mm body radius deep, half the 200 mm root long, centred.
        assert!(r.add_fin_tab(fins));
        let tab = r.radial(fins).unwrap().tab.unwrap();
        assert!((tab.height - 0.0125).abs() < 1e-12 && (tab.length - 0.1).abs() < 1e-12 && (tab.position - 0.05).abs() < 1e-12);
        // A fin that has one keeps it; one over a mount gets the mount's.
        assert!(r.add_fin_tab(fins) && r.radial(fins).unwrap().tab == Some(tab));
        let (mut r, fins, _) = rocket();
        assert!(r.add_fin_tab(fins) && r.radial(fins).unwrap().tab == Some(r.fin_tab_target(fins).unwrap().tab));
        let mount = r.fin_tab_target(fins).unwrap().mount;
        assert!(!r.add_fin_tab(mount));
    }

    #[test]
    fn a_tab_adds_the_mass_of_its_plate_at_its_own_cg() {
        let (mut r, fins, _) = rocket();
        let before = r.component_mass(fins).unwrap();
        assert!(r.set_fin_tab(fins, Some(FinTab { height: 0.023, length: 0.1, position: 0.05 })));
        let after = r.component_mass(fins).unwrap();
        // Three plates of 23 x 100 x 4 mm in birch plywood (630 kg/m^3), CG 100 mm behind the fin set's fore edge.
        let tab = 3.0 * 0.023 * 0.1 * 0.004 * MaterialId::BIRCH_PLYWOOD.density();
        assert!((after.mass - before.mass - tab).abs() < 1e-12, "{} vs {tab}", after.mass - before.mass);
        let cg = (before.mass * before.cg + tab * (r.part_station(fins).unwrap() + 0.1)) / after.mass;
        assert!((after.cg - cg).abs() < 1e-9, "{} vs {cg}", after.cg);
        assert!(r.set_fin_tab(fins, None));
        assert_eq!(r.component_mass(fins).unwrap(), before);
    }

    #[test]
    fn a_bad_tab_is_refused_and_a_tab_is_for_fins_only() {
        let (mut r, fins, mount) = rocket();
        assert!(!r.set_fin_tab(fins, Some(FinTab { height: f64::NAN, length: 0.1, position: 0.0 })));
        assert!(!r.set_fin_tab(fins, Some(FinTab { height: 0.01, length: -0.1, position: 0.0 })));
        assert!(!r.set_fin_tab(mount, Some(FinTab { height: 0.01, length: 0.1, position: 0.0 })));
        assert!(r.radial(fins).unwrap().tab.is_none());
    }

    #[test]
    fn tab_issues_name_what_is_wrong() {
        let (mut r, fins, _) = rocket();
        let set = |r: &mut Rocket, height, length, position| assert!(r.set_fin_tab(fins, Some(FinTab { height, length, position })));
        set(&mut r, 0.023, 0.25, 0.0);
        assert!(matches!(r.fin_tab_issues(fins)[..], [TabIssue::LongerThanRoot(..)]));
        set(&mut r, 0.023, 0.15, 0.1);
        assert!(r.fin_tab_issues(fins).contains(&TabIssue::PastRootEnds));
        // 30 mm deep goes 7 mm into the 54 mm mount.
        set(&mut r, 0.03, 0.1, 0.05);
        assert!(matches!(&r.fin_tab_issues(fins)[..], [TabIssue::IntoMount(_, by)] if (by - 0.007).abs() < 1e-9));
        set(&mut r, 0.06, 0.1, 0.05);
        assert!(r.fin_tab_issues(fins).contains(&TabIssue::PastAxis));
        // A centering ring across the tab: 3 mm thick, 230 mm into the tube, from the mount (27 mm) to the bore (50 mm; the model's
        // tube walls are its default thickness).
        set(&mut r, 0.023, 0.1, 0.05);
        let tube = r.parent_of(fins).unwrap();
        r.attach_internal(tube, 0.25, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: Some(0.054), thickness: 0.003 }).unwrap();
        assert!(matches!(&r.fin_tab_issues(fins)[..], [TabIssue::HitsRing(_)]));
        // Moved ahead of the ring the tab is clear.
        set(&mut r, 0.023, 0.04, 0.0);
        assert!(r.fin_tab_issues(fins).is_empty());
    }
}
