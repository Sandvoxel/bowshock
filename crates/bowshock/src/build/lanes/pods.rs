//! The `Pod ×N` brackets of Station Lanes: every part standing in a pod is bracketed with the others of its pod in its own lane,
//! and the bracket carries a tag (the link glyph and `Pod ×2`). The tag is never dropped: when something is in the way beside
//! the bracket's ends it slides along the bracket into the first free gap, then to the first free row of the lane, and failing
//! that opens a row of its own at the bottom of the lane ([`PodLayout::lanes`] is the scene's lanes with those rows added).
//! Pure (pixels in, pixels out), so the layout is unit-tested.

use bowshock_model::{PartId, System};

use super::canvas::{mono_w, TEXT_SMALL};
use super::geometry::{Axis, LaneBox, LABEL_W};
use super::scene::{Lane, Scene};

/// Height of a tag, px.
pub const TAG_H: f32 = 14.0;
/// Air kept between a tag and what is beside it, px.
const AIR: f32 = 3.0;
/// Width of the link glyph with its padding, and the padding after the text, px.
const GLYPH_W: f32 = 13.0;
const PAD_W: f32 = 6.0;

/// One pod's parts in one lane, as the bracket over them is drawn.
#[derive(Clone, Debug, PartialEq)]
pub struct PodSpan<'a> {
    /// The pod set.
    pub id: PartId,
    /// Its group label (`Pod ×2`).
    pub label: &'a str,
    /// The lane.
    pub system: System,
    /// Left and right edge of the parts, px.
    pub x0: f32,
    /// See `x0`.
    pub x1: f32,
    /// The topmost sub-row they occupy (the bracket runs over it).
    pub row: usize,
    /// Every part has dropped away in the phase on show.
    pub dropped: bool,
}

/// Where a bracket's tag sits: the left edge, the width and the sub-row (which may be one the lane gains for it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PodTag {
    /// Left edge, px.
    pub x: f32,
    /// Width, px.
    pub w: f32,
    /// Sub-row.
    pub row: usize,
}

/// The brackets and their tags for one axis and panel width.
#[derive(Clone, Debug, PartialEq)]
pub struct PodLayout<'a> {
    /// The scene's lanes, with a row more where a tag had nowhere else to go.
    pub lanes: Vec<Lane>,
    /// Each visible bracket with its tag.
    pub pods: Vec<(PodSpan<'a>, PodTag)>,
}

impl PodLayout<'_> {
    /// The pod set whose tag or bracket line is under `p` (the lane boxes are laid out from [`PodLayout::lanes`]): clicking either
    /// selects the pod set, as its `POD ×2` chip in the viewport does.
    pub fn hit(&self, rows: &[LaneBox], p: iced::Point) -> Option<PartId> {
        self.pods.iter().find_map(|(s, tag)| {
            let lane = rows.iter().find(|l| l.lane.system == s.system)?;
            let (y, h) = lane.row(tag.row);
            let mid = y + h / 2.0;
            let on_tag = p.x >= tag.x && p.x <= tag.x + tag.w && (p.y - mid).abs() <= TAG_H / 2.0;
            // The bracket is a 1 px line 2 px over its row's top: give it a few px of grip above, none into the bars below.
            let line = lane.row(s.row).0 - 2.0;
            let on_line = p.x >= s.x0.max(LABEL_W) - 2.0 && p.x <= s.x1 + 2.0 && p.y >= line - 3.0 && p.y <= line + 1.0;
            (on_tag || on_line).then_some(s.id)
        })
    }
}

/// The members of each pod in each lane but Structure (the pod's own bar is there): the span they cover and the row on top.
pub fn spans<'a>(scene: &'a Scene, axis: &Axis) -> Vec<PodSpan<'a>> {
    let mut out: Vec<PodSpan> = Vec::new();
    for bar in scene.bars.iter().filter(|b| !b.alt && !b.shoulder && b.system != System::Structure) {
        let Some(mark) = &bar.pod else { continue };
        let (a, b) = axis.span(bar.start, bar.end);
        match out.iter_mut().find(|s| s.id == mark.id && s.system == bar.system) {
            Some(s) => {
                s.x0 = s.x0.min(a);
                s.x1 = s.x1.max(b);
                s.row = s.row.min(bar.sub);
                s.dropped &= bar.dropped;
            }
            None => out.push(PodSpan { id: mark.id, label: &mark.label, system: bar.system, x0: a, x1: b, row: bar.sub, dropped: bar.dropped }),
        }
    }
    out
}

/// The horizontal extents, px, that something else covers in sub-row `row` of `system`: the bars (with the room their names take
/// beside them when a name does not fit inside) and the tags already placed there.
fn taken(scene: &Scene, axis: &Axis, name_w: &dyn Fn(&str) -> f32, placed: &[(System, PodTag)], system: System, row: usize) -> Vec<(f32, f32)> {
    let bars = scene.bars.iter().filter(|b| !b.alt && b.system == system && b.sub == row).map(|b| {
        let (a, z) = axis.span(b.start, b.end);
        // A block too short for its name has it beside it, so that space is taken too.
        let reach = if b.thin || z - a < name_w(&b.name) + 8.0 { mono_w(b.name.chars().count(), TEXT_SMALL) + 10.0 } else { 0.0 };
        (a, z + reach + if b.warn { 9.0 } else { 0.0 })
    });
    let tags = placed.iter().filter(|(s, t)| *s == system && t.row == row).map(|(_, t)| (t.x, t.x + t.w));
    bars.chain(tags).collect()
}

/// Whether a tag `w` wide with its left edge at `x` clears everything in `taken`.
fn clear(taken: &[(f32, f32)], x: f32, w: f32) -> bool {
    taken.iter().all(|&(a, b)| x + w + 1.0 <= a || b <= x - 1.0)
}

/// Places the tag of every pod bracket in a panel `width` px wide (see the module docs for the order of tries). `name_w` measures
/// a bar's name in the bar font.
pub fn layout<'a>(scene: &'a Scene, axis: &Axis, width: f32, name_w: &dyn Fn(&str) -> f32) -> PodLayout<'a> {
    let mut lanes = scene.lanes.clone();
    let mut spans = spans(scene, axis);
    spans.retain(|s| s.x1 >= LABEL_W && s.x0 <= width);
    spans.sort_by(|a, b| a.system.cmp(&b.system).then(a.x0.total_cmp(&b.x0)));
    let mut placed: Vec<(System, PodTag)> = Vec::new();
    let mut pods = Vec::new();
    for span in spans {
        let w = GLYPH_W + mono_w(span.label.chars().count(), TEXT_SMALL) + PAD_W;
        let Some(lane) = lanes.iter().position(|l| l.system == span.system) else { continue };
        let subs = lanes[lane].subs;
        let on_view = |x: f32| x >= LABEL_W && x + w <= width;

        // Beside the bracket's ends, then in the gaps along it, in its own row first and then in the others.
        let mut rows = std::iter::once(span.row).chain((0..subs).filter(|&r| r != span.row));
        let found = rows.find_map(|row| {
            let busy = taken(scene, axis, name_w, &placed, span.system, row);
            let mut gaps: Vec<f32> = std::iter::once(span.x0.max(LABEL_W)).chain(busy.iter().flat_map(|&(a, b)| [b + AIR, a - AIR - w])).collect();
            gaps.retain(|&x| x + w > span.x0 && x < span.x1);
            gaps.sort_by(f32::total_cmp);
            [span.x0 - AIR - w, span.x1 + AIR].into_iter().chain(gaps).find(|&x| on_view(x) && clear(&busy, x, w)).map(|x| PodTag { x, w, row })
        });
        // Nowhere on the lane: a row of its own below (several tags may share it, side by side). An empty row always has room.
        let tag = match found {
            Some(tag) => tag,
            None => {
                let mut row = subs;
                loop {
                    let busy = taken(scene, axis, name_w, &placed, span.system, row);
                    let first = span.x0.clamp(LABEL_W, (width - w).max(LABEL_W));
                    let gaps = busy.iter().map(|&(_, b)| b + AIR);
                    let fit = std::iter::once(first).chain(gaps).chain(std::iter::once(LABEL_W)).find(|&x| clear(&busy, x, w));
                    if let Some(x) = fit {
                        break PodTag { x, w, row };
                    }
                    row += 1;
                }
            }
        };
        lanes[lane].subs = lanes[lane].subs.max(tag.row + 1);
        placed.push((span.system, tag));
        pods.push((span, tag));
    }
    PodLayout { lanes, pods }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build::lanes::geometry::{self, bar_rect};
    use crate::build::lanes::scene::{BarKey, Inputs};
    use crate::units::UnitSystem;
    use bowshock_model::{InternalKind, MassRole, NoseShape, PartKind, Rocket};
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::Phase;
    use iced::Rectangle;

    fn name_w(text: &str) -> f32 {
        text.chars().count() as f32 * 6.2
    }

    fn build(rocket: &Rocket) -> Scene {
        let db = MotorDb::bundled();
        let stages = rocket.stages();
        Scene::build(&Inputs { rocket, stages: &stages, phase: Phase::Full, motors: &db, units: UnitSystem::MM_G_S, compare: false })
    }

    /// A 0.2 m nose and a 1.2 m core tube (stations 0.2 to 1.4) with a pod set of two at 0.1 m along it, and the pod's tube.
    fn core_with_pod() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let core = r.insert(1, PartKind::BodyTube { length: 1.2, diameter: 0.1 }).unwrap();
        let kind = r.default_pod();
        r.attach_radial(core, 0.1, 0.0, 2, kind).unwrap();
        let tube = r.pods().next().and_then(|p| p.kind.pod_body()).unwrap().parts().last().unwrap().id;
        (r, core, tube)
    }

    fn ballast(r: &mut Rocket, parent: PartId, at: f64) -> PartId {
        r.attach_internal(parent, at, InternalKind::MassComponent { mass: 0.1, length: 0.05, diameter: None, role: MassRole::Ballast }).unwrap()
    }

    /// Nothing but its own pod's bracket touches a tag: no bar (of its lane, in its row) and no other tag.
    fn assert_clear(scene: &Scene, layout: &PodLayout, axis: &Axis, width: f32) {
        let rows = geometry::rows(&layout.lanes, 800.0, 28.0);
        let rect = |system: System, t: &PodTag| {
            let lane = rows.iter().find(|l| l.lane.system == system).expect("the tag's lane");
            assert!(t.row < lane.lane.subs, "the lane has the tag's row");
            let (y, h) = lane.row(t.row);
            Rectangle { x: t.x, y: y + h / 2.0 - TAG_H / 2.0, width: t.w, height: TAG_H }
        };
        for (span, tag) in &layout.pods {
            let r = rect(span.system, tag);
            assert!(r.x >= LABEL_W && r.x + r.width <= width + 0.5 || tag.row >= scene.lanes.iter().find(|l| l.system == span.system).unwrap().subs);
            for bar in scene.bars.iter().filter(|b| !b.alt && b.system == span.system) {
                let b = bar_rect(bar, &rows, axis).unwrap();
                assert!(!b.intersects(&r), "the tag of {} overlaps {}", span.label, bar.name);
            }
            for (other, t) in layout.pods.iter().filter(|(o, _)| o.id != span.id || o.system != span.system) {
                if other.system == span.system {
                    assert!(!rect(other.system, t).intersects(&r), "two tags overlap");
                }
            }
        }
    }

    #[test]
    fn a_free_bracket_keeps_its_tag_beside_its_fore_end_in_the_row_of_the_bracket() {
        let (mut r, _, tube) = core_with_pod();
        let b = ballast(&mut r, tube, 0.1);
        let scene = build(&r);
        let axis = Axis::fit(1100.0, scene.length);
        let layout = layout(&scene, &axis, 1100.0, &name_w);
        let (span, tag) = layout.pods.iter().find(|(s, _)| s.system == System::Avionics).expect("the avionics bracket");
        let bar = scene.part_bar(b).unwrap();
        assert_eq!((span.label, tag.row, bar.sub), ("Pod ×2", bar.sub, 0));
        assert!((tag.x + tag.w + AIR - span.x0).abs() < 1e-3, "left of the bracket");
        assert_eq!(layout.lanes, scene.lanes, "no row added");
        assert_clear(&scene, &layout, &axis, 1100.0);
    }

    #[test]
    fn avionics_parts_hemming_the_pod_in_never_drop_its_tag() {
        let (mut r, core, tube) = core_with_pod();
        let b = ballast(&mut r, tube, 0.1);
        let (a, _) = r.part_span(b).unwrap();
        // A core avionics part touching the pod's ballast on its fore side, and the ballast's own name beside it on the other (too
        // short a bar for it inside): no room beside the bracket, none inside it, one row in the lane.
        let before = ballast(&mut r, core, a - 0.05 - 0.2);
        let scene = build(&r);
        for id in [b, before] {
            assert_eq!((scene.part_bar(id).unwrap().system, scene.part_bar(id).unwrap().sub), (System::Avionics, 0));
        }
        let axis = Axis::fit(1100.0, scene.length);
        let layout = layout(&scene, &axis, 1100.0, &name_w);
        let tags: Vec<_> = layout.pods.iter().filter(|(s, _)| s.system == System::Avionics).collect();
        assert_eq!(tags.len(), 1, "the tag is in the scene");
        assert_eq!(tags[0].1.row, 1, "in a row the lane gained");
        assert_eq!(layout.lanes.iter().find(|l| l.system == System::Avionics).unwrap().subs, 2);
        assert_clear(&scene, &layout, &axis, 1100.0);
        // Squeezed to a quarter of the width, the same holds.
        let axis = Axis::fit(420.0, scene.length);
        let layout = super::layout(&scene, &axis, 420.0, &name_w);
        assert_eq!(layout.pods.iter().filter(|(s, _)| s.system == System::Avionics).count(), 1);
        assert_clear(&scene, &layout, &axis, 420.0);
    }

    #[test]
    fn the_tag_slides_along_the_bracket_into_a_gap_between_its_parts() {
        let (mut r, core, tube) = core_with_pod();
        let first = ballast(&mut r, tube, 0.05);
        let second = ballast(&mut r, tube, 0.35);
        let (a, _) = r.part_span(first).unwrap();
        ballast(&mut r, core, a - 0.05 - 0.2);
        let _ = second;
        let scene = build(&r);
        let axis = Axis::fit(1100.0, scene.length);
        let layout = layout(&scene, &axis, 1100.0, &name_w);
        let (span, tag) = layout.pods.iter().find(|(s, _)| s.system == System::Avionics).unwrap();
        assert_eq!(tag.row, span.row, "still in the bracket's own row");
        let (g0, g1) = (axis.px(r.part_span(first).unwrap().1), axis.px(r.part_span(second).unwrap().0));
        assert!(tag.x >= g0 && tag.x + tag.w <= g1, "between the two parts");
        assert_eq!(layout.lanes, scene.lanes);
        assert_clear(&scene, &layout, &axis, 1100.0);
    }

    #[test]
    fn a_propulsion_bracket_with_a_motor_keeps_its_tag_too() {
        // The pod's motor mount and its motor bar sit in one bracket in the Propulsion lane.
        let (mut r, _, tube) = core_with_pod();
        let mount = r
            .attach_internal(tube, 0.05, InternalKind::InnerTube { outer_diameter: 0.03, thickness: 0.002, length: 0.4, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let config = r.active_config_id();
        r.set_motor(config, mount, Some(bowshock_model::MotorRef::new("H128W", "Aerotech"))).unwrap();
        let scene = build(&r);
        assert!(scene.bars.iter().any(|b| matches!(b.key, BarKey::Motor { .. }) && b.pod.is_some()));
        let axis = Axis::fit(1100.0, scene.length);
        let layout = layout(&scene, &axis, 1100.0, &name_w);
        assert_eq!(layout.pods.iter().filter(|(s, _)| s.system == System::Propulsion).count(), 1);
        assert_clear(&scene, &layout, &axis, 1100.0);
    }

    #[test]
    fn clicking_a_pod_tag_or_its_bracket_hits_the_pod_set() {
        let (mut r, _, tube) = core_with_pod();
        let b = ballast(&mut r, tube, 0.1);
        let pod = r.pods().next().unwrap().id;
        let scene = build(&r);
        let axis = Axis::fit(1100.0, scene.length);
        let layout = layout(&scene, &axis, 1100.0, &name_w);
        let rows = geometry::rows(&layout.lanes, 800.0, 28.0);
        let (span, tag) = layout.pods.iter().find(|(s, _)| s.system == System::Avionics).unwrap();
        let lane = rows.iter().find(|l| l.lane.system == System::Avionics).unwrap();
        let (y, h) = lane.row(tag.row);
        assert_eq!(layout.hit(&rows, iced::Point::new(tag.x + tag.w / 2.0, y + h / 2.0)), Some(pod), "the tag");
        let line = lane.row(span.row).0 - 2.0;
        assert_eq!(layout.hit(&rows, iced::Point::new((span.x0 + span.x1) / 2.0, line)), Some(pod), "the bracket");
        // The middle of the pod's own part is the part's, not the pod's.
        let bar = bar_rect(scene.part_bar(b).unwrap(), &rows, &axis).unwrap();
        assert_eq!(layout.hit(&rows, bar.center()), None);
    }
}
