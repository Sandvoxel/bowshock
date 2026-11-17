use roxmltree::Node;

use crate::geometry::FinTab;
use crate::material::{Finish, MaterialId};

pub(super) fn child<'a, 'i>(n: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    n.children().find(|c| c.is_element() && c.tag_name().name() == tag)
}

pub(super) fn children<'a, 'i>(n: Node<'a, 'i>, tag: &'a str) -> impl Iterator<Item = Node<'a, 'i>> {
    n.children().filter(move |c| c.is_element() && c.tag_name().name() == tag)
}

pub(super) fn text<'a>(n: Node<'a, '_>, tag: &str) -> Option<&'a str> {
    child(n, tag).and_then(|c| c.text()).map(str::trim)
}

pub(super) fn num(n: Node, tag: &str) -> Option<f64> {
    text(n, tag).and_then(|t| t.parse::<f64>().ok()).filter(|v| v.is_finite())
}

pub(super) fn name_of(n: Node) -> Option<String> {
    text(n, "name").filter(|s| !s.is_empty()).map(str::to_string)
}

/// A dimension that may be `auto` (OpenRocket writes `auto <computed value>`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Dim {
    Auto(Option<f64>),
    Val(f64),
}

impl Dim {
    /// The number if there is one (an explicit value, or the value OpenRocket computed for `auto`).
    pub(super) fn value(self) -> Option<f64> {
        match self {
            Dim::Auto(v) => v,
            Dim::Val(v) => Some(v),
        }
    }

    /// The explicit value, or `None` for `auto`.
    pub(super) fn explicit(self) -> Option<f64> {
        match self {
            Dim::Val(v) => Some(v),
            Dim::Auto(_) => None,
        }
    }
}

pub(super) fn dim(n: Node, tag: &str) -> Option<Dim> {
    let t = text(n, tag)?;
    if let Some(rest) = t.strip_prefix("auto") {
        return Some(Dim::Auto(rest.trim().parse::<f64>().ok().filter(|v| v.is_finite())));
    }
    t.parse::<f64>().ok().filter(|v| v.is_finite()).map(Dim::Val)
}

/// `(method, value)` of an axial offset: `<axialoffset method="top">` (or the older `<position type=…>`).
pub(super) fn axial_offset(n: Node) -> (String, f64) {
    for tag in ["axialoffset", "position"] {
        if let Some(c) = child(n, tag) {
            let method = c.attribute("method").or_else(|| c.attribute("type")).unwrap_or("top").to_string();
            return (method, c.text().and_then(|t| t.trim().parse().ok()).unwrap_or(0.0));
        }
    }
    ("top".to_string(), 0.0)
}

pub(super) fn finish_of(n: Node) -> Finish {
    match text(n, "finish") {
        Some("rough") => Finish::Rough,
        Some("unfinished") => Finish::Unfinished,
        Some("smooth") => Finish::SmoothPaint,
        Some("polished") => Finish::Polished,
        _ => Finish::RegularPaint,
    }
}

pub(super) fn deg(v: f64) -> f64 {
    v.to_radians()
}

/// The model materials an OpenRocket material of this name can be: its family by the words in the name (`Aircraft plywood (LOC)` is
/// plywood whatever its density, so it takes a plywood's shear modulus, not that of the nearest-density cardboard). `None` for a name
/// that says nothing about what it is made of (the nearest density then decides).
pub(super) fn family_of(name: &str) -> Option<&'static [MaterialId]> {
    const WOODS: &[MaterialId] = &[MaterialId::BASSWOOD, MaterialId::BALSA, MaterialId::BIRCH_PLYWOOD, MaterialId::AIRCRAFT_PLYWOOD];
    const FAMILIES: [(&[&str], &[MaterialId]); 16] = [
        (&["aircraft ply"], &[MaterialId::AIRCRAFT_PLYWOOD]),
        (&["plywood", "birch ply"], &[MaterialId::BIRCH_PLYWOOD, MaterialId::AIRCRAFT_PLYWOOD]),
        (&["balsa"], &[MaterialId::BALSA]),
        (&["basswood"], &[MaterialId::BASSWOOD]),
        (&["fiberglass", "fibreglass", "g10", "fr4", "glass"], &[MaterialId::FIBERGLASS]),
        (&["carbon"], &[MaterialId::CARBON_FIBER]),
        (&["aluminum", "aluminium"], &[MaterialId::ALUMINUM]),
        (&["steel"], &[MaterialId::STEEL]),
        (&["brass"], &[MaterialId::BRASS]),
        (&["lead"], &[MaterialId::LEAD]),
        (&["epoxy"], &[MaterialId::EPOXY]),
        (&["polycarbonate"], &[MaterialId::POLYCARBONATE]),
        (&["polystyrene"], &[MaterialId::POLYSTYRENE]),
        (&["blue tube", "quantum"], &[MaterialId::BLUE_TUBE]),
        (&["phenolic"], &[MaterialId::KRAFT_PHENOLIC]),
        (&["cardboard", "paper"], &[MaterialId::CARDBOARD]),
    ];
    let lower = name.to_lowercase();
    if let Some((_, ids)) = FAMILIES.iter().find(|(words, _)| words.iter().any(|w| lower.contains(w))) {
        return Some(ids);
    }
    ["birch", "maple", "spruce", "pine", "oak", "poplar", "wood", "lumber"].iter().any(|w| lower.contains(w)).then_some(WOODS)
}

/// The through-the-wall tab of an OpenRocket fin set `n` whose root chord is `root`: `tabheight`, `tablength` and `tabposition`, which
/// is measured from the root's leading edge (`relativeto="front"`), from the middle of the root (`center`) or from its trailing edge
/// (`end`, the tab's aft edge that far ahead of it). `None` without a tab.
pub(super) fn fin_tab_of(n: Node, root: f64) -> Option<FinTab> {
    let (height, length) = (num(n, "tabheight")?, num(n, "tablength")?);
    if !(height > 0.0 && length > 0.0) {
        return None;
    }
    let at = |rel: &str| children(n, "tabposition").find(|c| c.attribute("relativeto") == Some(rel)).and_then(|c| c.text()?.trim().parse::<f64>().ok());
    let position = if let Some(v) = at("front") {
        v
    } else if let Some(v) = at("center") {
        (root - length) / 2.0 + v
    } else if let Some(v) = at("end") {
        root - length - v
    } else {
        children(n, "tabposition").find_map(|c| c.text()?.trim().parse::<f64>().ok()).unwrap_or(0.0)
    };
    Some(FinTab { height, length, position: position.max(0.0) })
}
