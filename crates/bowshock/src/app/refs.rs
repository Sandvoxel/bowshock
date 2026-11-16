//! References between fields (design/components/UnitInput, *References*): the app-level pick mode and the helpers that turn a
//! field reference into what the user sees.
//!
//! A field that gets a `{` is in pick mode: [`Ui::picking`] holds it as the `target` (`Picking { target }`), every other
//! numeric field of the inspector draws as a pick target, and the viewport and Station Lanes pick parts: a click on a part does
//! not select it but opens [`PartMenu`], a small list of that part's values (outer and inner Ø, length, wall, offset…).
//! Whatever is picked (a field, or a row of the menu) arrives as `inspector::Message::PickField` and becomes an [`Insert`],
//! which the target field's widget turns into a chip.
//!
//! The formulas themselves live in the document (`bowshock_model::Formula`), so undo covers them.

use bowshock_model::{Dim, FieldRef, PartId, Rocket};
use iced::widget::{button, column, container, mouse_area, row, text, Space};
use iced::{Color, Element, Fill, Point, Widget};

use crate::app::{self, Ctx};
use crate::build::inspector;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, TEXT_BODY, TEXT_LABEL};
use crate::units::{self, Quantity};
use crate::widgets::unit_input::Chip;

/// Width of the part menu.
const MENU_W: f32 = 236.0;

/// The field in pick mode.
#[derive(Debug, Clone)]
pub struct Picking {
    /// The field that is being edited and will receive the reference (`Picking { target: FieldId }` of the kit).
    pub target: FieldRef,
    /// The part that was clicked in the viewport or the lanes, with the cursor position, while its menu is open.
    pub menu: Option<PartMenu>,
}

/// The menu of one part's values, opened by clicking the part during a pick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PartMenu {
    /// The part.
    pub part: PartId,
    /// Where the menu opens, in window coordinates.
    pub at: Point,
}

/// A reference for the target field to take: `text` is `{name#id.key}`; each insert has its own `token`.
#[derive(Debug, Clone)]
pub struct Insert {
    /// The field that receives it.
    pub target: FieldRef,
    /// Distinguishes this insert from the last one (the widget inserts once per token).
    pub token: u64,
    /// `{body_tube#3.od}`.
    pub text: String,
}

/// Everything the references UI keeps besides the formulas.
#[derive(Debug, Default)]
pub struct Ui {
    /// The field in pick mode, if any.
    pub picking: Option<Picking>,
    /// The latest pick, for the target field's widget to take.
    pub insert: Option<Insert>,
    /// A refused formula: the field and why ("Circular: Root chord already uses Tip chord…").
    pub error: Option<(FieldRef, String)>,
    /// A scrub was tried on this formula field: *Replace formula with a value?* is showing for it.
    pub confirm: Option<FieldRef>,
    /// The source field whose dependants are listed under it.
    pub dependants: Option<FieldRef>,
    counter: u64,
}

impl Ui {
    /// Whether a field is in pick mode.
    pub fn is_picking(&self) -> bool {
        self.picking.is_some()
    }

    /// Whether `field` is the one being edited in pick mode.
    pub fn is_target(&self, field: FieldRef) -> bool {
        self.picking.as_ref().is_some_and(|p| p.target == field)
    }

    /// Pick mode starts for `target`.
    pub fn start(&mut self, target: FieldRef) {
        self.picking = Some(Picking { target, menu: None });
        self.error = None;
    }

    /// Pick mode ends.
    pub fn end(&mut self) {
        self.picking = None;
    }

    /// A part was clicked while picking: open its menu at `at`.
    pub fn open_menu(&mut self, part: PartId, at: Point) {
        if let Some(p) = &mut self.picking {
            p.menu = Some(PartMenu { part, at });
        }
    }

    /// Closes the part menu.
    pub fn close_menu(&mut self) {
        if let Some(p) = &mut self.picking {
            p.menu = None;
        }
    }

    /// The target takes `reference`: pick mode ends, and the target's widget will insert it.
    pub fn pick(&mut self, rocket: &Rocket, reference: FieldRef) {
        let Some(picking) = self.picking.take() else { return };
        self.counter += 1;
        let name = rocket.kind_of(reference.part).map(|p| p.name()).unwrap_or_default();
        self.insert = Some(Insert { target: picking.target, token: self.counter, text: format!("{{{}}}", reference.token(&name)) });
    }
}

/// The short name of a dimension on a chip: `Ø`, `L`, `Root`.
pub fn dim_short(dim: Dim) -> String {
    match dim {
        Dim::Diameter => "Ø".into(),
        Dim::ForeDiameter => "Fore Ø".into(),
        Dim::InnerDiameter => "Inner Ø".into(),
        Dim::Length | Dim::ForeLength => "L".into(),
        Dim::Wall => "Wall".into(),
        Dim::Position => "Offset".into(),
        Dim::Span | Dim::Height => "Height".into(),
        Dim::RootChord => "Root".into(),
        Dim::TipChord => "Tip".into(),
        Dim::Sweep => "Sweep".into(),
        Dim::Thickness => "Thickness".into(),
        Dim::Cant => "Cant".into(),
        Dim::Angle => "Rotation".into(),
        Dim::Gap => "Gap".into(),
        Dim::Shape => "Shape".into(),
        Dim::Overhang => "Overhang".into(),
        Dim::PackedLength => "Packed".into(),
        Dim::ShoulderLength => "Shoulder L".into(),
        Dim::ShoulderDiameter => "Shoulder Ø".into(),
        Dim::ShoulderThickness => "Shoulder wall".into(),
        Dim::ForeShoulderLength => "Fore shoulder L".into(),
        Dim::ForeShoulderDiameter => "Fore shoulder Ø".into(),
        Dim::ForeShoulderThickness => "Fore shoulder wall".into(),
        Dim::FilletRadius => "Fillet".into(),
        Dim::Vertex(i) => format!("Vertex {}", i + 1),
    }
}

/// A dimension as the inspector's label spells it, in sentence case: `Root chord`, `Outer Ø`.
pub fn dim_name(rocket: &Rocket, part: PartId, dim: Dim) -> String {
    match dim {
        Dim::Diameter if rocket.part(part).is_some_and(|p| matches!(p.kind, bowshock_model::PartKind::NoseCone { .. })) => "Base Ø".into(),
        Dim::Diameter => "Outer Ø".into(),
        Dim::InnerDiameter => "Inner Ø".into(),
        Dim::ForeDiameter => "Fore Ø".into(),
        Dim::Position => "Offset".into(),
        Dim::Span | Dim::Height => "Height".into(),
        other => {
            let label = other.label().to_lowercase();
            let mut chars = label.chars();
            chars.next().map(|c| c.to_uppercase().chain(chars).collect()).unwrap_or(label)
        }
    }
}

/// The quantity a dimension is shown in.
pub fn dim_quantity(dim: Dim) -> Quantity {
    match bowshock_model::dim_kind(dim) {
        bowshock_model::Kind::Length => Quantity::Length,
        bowshock_model::Kind::Angle => Quantity::Angle,
        bowshock_model::Kind::Scalar => Quantity::Scalar,
    }
}

/// The value a reference token names, in SI (`None` if the part or its dimension is gone): the widget's resolver.
pub fn resolve(rocket: &Rocket, token: &str) -> Option<f64> {
    let (r, _) = FieldRef::from_token(token)?;
    rocket.dim(r.part, r.dim)
}

/// How a reference token shows as a chip: `Body tube · Ø` with the part's system colour, red and struck through when the
/// part has been deleted (its last known name is remembered by the formula). `None` for text that is not a reference.
pub fn chip(rocket: &Rocket, t: &crate::tokens::Tokens, token: &str) -> Option<Chip> {
    let (r, slug) = FieldRef::from_token(token)?;
    let short = dim_short(r.dim);
    match (rocket.kind_of(r.part), rocket.dim(r.part, r.dim)) {
        (Some(part), Some(_)) => Some(Chip { label: format!("{} · {short}", part.name()), color: system_color(t, part.system()), broken: false }),
        _ => {
            let remembered = rocket.formulas().iter().find_map(|f| f.names.get(&r.part).cloned());
            let name = remembered.or_else(|| rocket.kind_of(r.part).map(|p| p.name())).unwrap_or(slug);
            Some(Chip { label: format!("{name} · {short}"), color: t.ink_muted, broken: true })
        }
    }
}

/// The name of the first deleted part a broken formula refers to.
pub fn broken_name(rocket: &Rocket, part: PartId, dim: Dim) -> Option<String> {
    let bowshock_model::FormulaStatus::Broken(gone) = rocket.formula_status(part, dim) else { return None };
    let f = rocket.formula(part, dim)?;
    gone.first().map(|r| rocket.formula_name(f, r.part).unwrap_or_else(|| "A part".to_string()))
}

/// A formula text with its tokens written as the chips show them and the operators as maths: `2 × [Body tube · Ø]`.
pub fn pretty(rocket: &Rocket, t: &crate::tokens::Tokens, text: &str) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(open) = rest.find('{') {
        out.push_str(&rest[..open]);
        let Some(len) = rest[open..].find('}') else {
            out.push_str(&rest[open..]);
            rest = "";
            break;
        };
        let token = &rest[open + 1..open + len];
        match chip(rocket, t, token) {
            Some(c) => out.push_str(&format!("[{}]", c.label)),
            None => out.push_str(&rest[open..=open + len]),
        }
        rest = &rest[open + len + 1..];
    }
    out.push_str(rest);
    out.replace(" * ", " × ").replace(" / ", " ÷ ").replace(" - ", " − ")
}

/// The values of `part` a formula can use, in menu order: the dimension, its name and value.
pub fn values(rocket: &Rocket, part: PartId) -> Vec<(FieldRef, String, f64)> {
    const ORDER: [Dim; 16] = [
        Dim::Diameter,
        Dim::ForeDiameter,
        Dim::InnerDiameter,
        Dim::Length,
        Dim::Position,
        Dim::Wall,
        Dim::Span,
        Dim::Height,
        Dim::RootChord,
        Dim::TipChord,
        Dim::Sweep,
        Dim::Thickness,
        Dim::PackedLength,
        Dim::Overhang,
        Dim::Cant,
        Dim::Angle,
    ];
    ORDER.iter().filter_map(|&d| Some((FieldRef::new(part, d), dim_name(rocket, part, d), rocket.dim(part, d)?))).collect()
}

/// The sentence of a refused circular formula: `Root chord already uses Tip chord.`
pub fn circular_message(rocket: &Rocket, target: FieldRef, referenced: FieldRef) -> String {
    let name = |r: FieldRef| {
        let field = dim_name(rocket, r.part, r.dim);
        if r.part == target.part {
            field
        } else {
            format!("{} · {field}", rocket.kind_of(r.part).map(|p| p.name()).unwrap_or_default())
        }
    };
    format!("{} already uses {}.", name(referenced), name(target))
}

/// The colour of a system's parts (the square on a chip and in the part menu).
fn system_color(t: &crate::tokens::Tokens, system: bowshock_model::System) -> Color {
    use bowshock_model::System;
    match system {
        System::Structure => t.part_body,
        System::Aero => t.sys_aero,
        System::Propulsion => t.sys_propulsion,
        System::Recovery => t.sys_recovery,
        System::Avionics => t.sys_avionics,
    }
}

/// The part menu while a part is clicked in a pick, as an overlay of the whole window (`None` when it is not open). It opens at
/// the click and slides back inside the window when it would overhang an edge.
pub fn menu_view(ctx: &Ctx) -> Option<Element<'_, app::Message>> {
    const ROW_H: f32 = 28.0;
    const HEAD_H: f32 = 36.0;
    let menu = ctx.refs.picking.as_ref()?.menu?;
    let rocket = ctx.doc.rocket();
    let part = rocket.kind_of(menu.part)?;
    let color = system_color(ctx.tokens(), part.system());
    let name = part.name().to_uppercase();
    let rows: Vec<(FieldRef, String, String)> = values(rocket, menu.part)
        .into_iter()
        .map(|(reference, label, value)| (reference, label, units::format(value, dim_quantity(reference.dim), &ctx.units).0))
        .collect();
    let height = HEAD_H + ROW_H * rows.len() as f32;
    Some(
        iced::widget::responsive(move |window| {
            let square = container(Space::new().width(8).height(8))
                .style(move |_: &iced::Theme| container::Style { background: Some(color.into()), ..container::Style::default() });
            let head = row![
                square,
                text(name.clone()).font(MONO).size(TEXT_LABEL).style(theme::muted),
                Space::new().width(Fill),
                text("click part").font(MONO).size(TEXT_LABEL).style(theme::faint),
            ]
            .spacing(SPACE_2)
            .align_y(iced::Center);
            let mut cells: Vec<Element<'_, app::Message>> = vec![container(head).padding([SPACE_2, 12.0]).width(Fill).boxed()];
            for (reference, label, shown) in &rows {
                let line = row![
                    text(label.clone()).size(TEXT_BODY).style(theme::ink),
                    Space::new().width(Fill),
                    text(shown.clone()).font(MONO).size(TEXT_BODY).style(theme::muted)
                ]
                .align_y(iced::Center);
                cells.push(
                    button(line)
                        .width(Fill)
                        .padding([5.0, 12.0])
                        .style(theme::ghost)
                        .on_press(app::Message::Inspector(inspector::Message::PickField(*reference)))
                        .boxed(),
                );
            }
            let card = container(column(cells)).width(MENU_W).style(theme::overlay);
            let left = (menu.at.x + 6.0).min(window.width - MENU_W - 8.0).max(8.0);
            let top = (menu.at.y + 6.0).min(window.height - height - 8.0).max(8.0);
            let placed = container(card).padding(iced::Padding { top, left, right: 0.0, bottom: 0.0 });
            let backdrop = mouse_area(Space::new().width(Fill).height(Fill))
                .interaction(iced::mouse::Interaction::Idle)
                .on_press(app::Message::Inspector(inspector::Message::PickMenuClose));
            iced::widget::stack![backdrop, placed].boxed()
        })
        .boxed(),
    )
}
