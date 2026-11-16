//! The Scale overlay: multiply the whole rocket (or the selected parts) by one factor, as a single undo step. It opens from the
//! rocket Inspector's More menu ("Scale rocket…") and from the part context menu ("Scale…"). For the whole rocket a *Target
//! length* field is the other way to give the factor (target / current length).
//!
//! The overlay is app-level state in [`Ctx::scale`]; Escape or Cancel closes it, Apply runs [`Rocket::scale`] inside one
//! `doc.edit`.

use bowshock_model::{PartId, Rocket, ScaleScope};
use iced::widget::{column, container, mouse_area, row, text, text_input, Space, Stack};
use iced::{Background, Color, Element, Fill, Widget};

use crate::app::{Ctx, Message as AppMessage};
use crate::theme::{self, MONO, SANS_MEDIUM};
use crate::tokens::{CONTROL_H, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::button::{button, Weight};
use crate::widgets::field::{checkbox, labeled};

/// Text-input id of the factor field, so the overlay can focus it when it opens.
pub const FACTOR_ID: &str = "scale-factor";

/// The factor range [`Rocket::scale`] accepts.
const RANGE: (f64, f64) = (0.01, 100.0);

/// What the overlay is editing.
#[derive(Debug, Clone, PartialEq)]
pub struct Dialog {
    /// The parts to scale; `None` scales the whole rocket.
    pub parts: Option<Vec<PartId>>,
    /// The factor field's text.
    pub factor: String,
    /// The target-length field's text (whole rocket only), in the length unit.
    pub target: String,
    /// Also scale mass overrides (by the cube) and CG overrides.
    pub masses: bool,
}

impl Dialog {
    /// The overlay for the whole rocket, at factor 1.
    pub fn whole(ctx: &Ctx) -> Dialog {
        let unit = Quantity::Length.unit_for(ctx.units);
        Dialog { parts: None, factor: "1".into(), target: crate::units::format_in(ctx.doc.rocket().length(), unit), masses: true }
    }

    /// The overlay for the given parts, at factor 1.
    pub fn parts(parts: Vec<PartId>) -> Dialog {
        Dialog { parts: Some(parts), factor: "1".into(), target: String::new(), masses: true }
    }

    /// The factor typed, if it is a number in range.
    pub fn value(&self) -> Option<f64> {
        let v: f64 = self.factor.trim().parse().ok()?;
        (v.is_finite() && (RANGE.0..=RANGE.1).contains(&v)).then_some(v)
    }

    fn scope(&self) -> ScaleScope {
        match &self.parts {
            None => ScaleScope::Whole,
            Some(ids) => ScaleScope::Parts(ids.clone()),
        }
    }
}

/// Messages of the overlay.
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    /// Open it for the whole rocket.
    OpenWhole,
    /// Open it for the selected parts.
    OpenParts,
    /// The factor text changed.
    Factor(String),
    /// The target length text changed.
    Target(String),
    /// The "Scale mass overrides" checkbox.
    Masses(bool),
    /// Apply and close.
    Apply,
    /// Close without applying.
    Close,
}

/// Carries out a message of the overlay.
pub fn update(ctx: &mut Ctx, message: Message) -> iced::Task<AppMessage> {
    let unit = Quantity::Length.unit_for(ctx.units);
    let length = ctx.doc.rocket().length();
    match message {
        Message::OpenWhole => {
            ctx.scale = Some(Dialog::whole(ctx));
            return focus_factor();
        }
        Message::OpenParts => {
            let parts = ctx.selected_parts();
            if !parts.is_empty() {
                ctx.scale = Some(Dialog::parts(parts));
                return focus_factor();
            }
        }
        Message::Factor(text) => {
            if let Some(d) = &mut ctx.scale {
                d.factor = text;
                if let (None, Some(f)) = (&d.parts, d.value()) {
                    d.target = crate::units::format_in(length * f, unit);
                }
            }
        }
        Message::Target(text) => {
            if let Some(d) = &mut ctx.scale {
                d.target = text;
                if d.parts.is_none() && length > 0.0 {
                    if let Ok(shown) = d.target.trim().parse::<f64>() {
                        let f = unit.to_si(shown) / length;
                        if f.is_finite() && f > 0.0 {
                            d.factor = format!("{f:.4}").trim_end_matches('0').trim_end_matches('.').to_string();
                        }
                    }
                }
            }
        }
        Message::Masses(on) => {
            if let Some(d) = &mut ctx.scale {
                d.masses = on;
            }
        }
        Message::Close => ctx.scale = None,
        Message::Apply => {
            let Some(dialog) = ctx.scale.take() else { return iced::Task::none() };
            let Some(factor) = dialog.value() else {
                ctx.scale = Some(dialog);
                return iced::Task::none();
            };
            let whole = dialog.parts.is_none();
            let (scope, masses) = (dialog.scope(), dialog.masses);
            let changed = ctx.doc.edit(&format!("Scale ×{factor:.3}"), |r: &mut Rocket| r.scale(factor, &scope, masses));
            if changed > 0 && whole {
                ctx.camera_fit = true;
            }
            ctx.notify(match (whole, changed) {
                (_, 0) => "Nothing to scale".to_string(),
                (true, _) => format!("Scaled the rocket ×{factor:.3}"),
                (false, n) => format!("Scaled {n} part{} ×{factor:.3}", if n == 1 { "" } else { "s" }),
            });
        }
    }
    iced::Task::none()
}

/// Focuses the factor field with its text selected, so typing replaces it.
fn focus_factor() -> iced::Task<AppMessage> {
    iced::Task::batch([iced::widget::operation::focus(FACTOR_ID), iced::widget::operation::text_input::select_all(FACTOR_ID)])
}

/// The overlay: a dimmed backdrop that takes every click and a centred card; `None` while it is closed.
pub fn view(ctx: &Ctx) -> Option<Element<'_, AppMessage>> {
    let dialog = ctx.scale.as_ref()?;
    let unit = Quantity::Length.unit_for(ctx.units);
    let wrap = AppMessage::Scale;
    let nothing = || AppMessage::Chrome(crate::chrome::Message::Nothing);
    let backdrop = container(Space::new().width(Fill).height(Fill))
        .style(|_| container::Style { background: Some(Background::Color(Color::from_rgba(0.0, 0.0, 0.0, 0.55))), ..container::Style::default() });
    let title = text(heading(dialog)).font(SANS_MEDIUM).size(15);
    let blurb = text(if dialog.parts.is_none() {
        "Multiplies every length by the factor: tubes, fins, internal parts and where they sit. Angles, counts, materials and motors stay."
    } else {
        "Multiplies the selected parts' own dimensions by the factor. Where they sit stays; parts behind them move to follow."
    })
    .size(TEXT_BODY)
    .style(theme::muted);
    fn field<'a>(value: &'a str, on_input: fn(String) -> Message, id: Option<&'static str>) -> Element<'a, AppMessage> {
        let mut input = text_input("", value)
            .on_input(move |s| AppMessage::Scale(on_input(s)))
            .size(TEXT_BODY)
            .font(MONO)
            .padding([CONTROL_H / 2.0 - 10.0, SPACE_2])
            .style(theme::input);
        if let Some(id) = id {
            input = input.id(id).on_submit(AppMessage::Scale(Message::Apply));
        }
        input.boxed()
    }
    let mut fields: Vec<Element<'_, AppMessage>> = vec![labeled("Factor", field(&dialog.factor, Message::Factor, Some(FACTOR_ID)))];
    let valid = dialog.value();
    if dialog.parts.is_none() {
        let label = format!("Length ({})", unit.symbol);
        fields.push(
            row![
                crate::widgets::field::label::<AppMessage>("Target length"),
                field(&dialog.target, Message::Target, None),
                text(label).size(TEXT_BODY).style(theme::faint)
            ]
            .spacing(SPACE_2)
            .align_y(iced::Center)
            .boxed(),
        );
        if let Some(f) = valid {
            let now = ctx.doc.rocket().length();
            let (a, b) = (crate::units::format_in(now, unit), crate::units::format_in(now * f, unit));
            fields.push(text(format!("{a} → {b} {}", unit.symbol)).size(TEXT_BODY).font(MONO).style(theme::muted).boxed());
        }
    }
    fields.push(checkbox("Scale mass overrides", dialog.masses, move |on| wrap(Message::Masses(on))));
    if valid.is_none() {
        fields.push(text(format!("Enter a factor from {} to {}.", RANGE.0, RANGE.1)).size(TEXT_BODY).style(theme::colored(|t| t.err)).boxed());
    }
    let cancel: Element<'_, AppMessage> = button("Cancel", Weight::Secondary).key("Esc").on_press(wrap(Message::Close)).into();
    let mut apply = button("Apply", Weight::Primary).key("↵");
    if valid.is_some() {
        apply = apply.on_press(wrap(Message::Apply));
    }
    let apply: Element<'_, AppMessage> = apply.into();
    let buttons = row![Space::new().width(Fill), cancel, apply].spacing(SPACE_2).align_y(iced::Center);
    let card = container(mouse_area(column![title, blurb, column(fields).spacing(SPACE_2), buttons].spacing(SPACE_3 + 2.0)).on_press(nothing()))
        .width(440)
        .padding(SPACE_3 + 8.0)
        .style(theme::overlay);
    let placed = container(card).center(Fill);
    Some(Stack::with_children([mouse_area(backdrop).interaction(iced::mouse::Interaction::Idle).on_press(nothing()).boxed(), placed.boxed()]).boxed())
}

/// The overlay's title: what is being scaled.
fn heading(dialog: &Dialog) -> String {
    match dialog.parts.as_deref() {
        None => "Scale rocket".to_string(),
        Some([_]) => "Scale part".to_string(),
        Some(ids) => format!("Scale {} parts", ids.len()),
    }
}
