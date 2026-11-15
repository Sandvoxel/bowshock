//! UnitInput (design/components/UnitInput): a numeric field that knows its physical quantity. Label column, scrub grip,
//! mono value, optional badge and a unit chip, with the kit's states: default, focus, drag, pick, formula, auto, override
//! and error.
//!
//! * **Typing**: a number, a unit-suffixed value (`3in`, `76.2mm`, `1' 2"`) or arithmetic (`(120-50)/2`, `2*38`), parsed by
//!   [`crate::units`]. Enter or clicking away commits, Esc reverts. A value that does not parse keeps the last valid one
//!   and explains itself in a hint line under the field.
//! * **Scrub**: drag the grip (or Alt-drag the field) sideways; one pixel is one display step, `Shift` ×0.1, `Ctrl`/`⌘`
//!   ×10. [`UnitInput::on_scrub`] is published live, [`UnitInput::on_commit`] when the mouse is released.
//! * **Unit chip**: a click cycles the display unit within the quantity (mm → cm → m → in → ft). The choice lives in the
//!   widget; [`UnitInput::on_unit`] reports it so the app can make it project-wide.
//! * **References**: text containing `{id}` is a formula. With [`UnitInput::resolver`] it is evaluated, and the raw text is
//!   published through [`UnitInput::on_formula`] so the app can store the formula and show the `ƒ` badge
//!   ([`State::Formula`]). Typing an unclosed `{` enters pick mode ([`UnitInput::on_pick_mode`]); the app then marks other
//!   fields [`State::PickTarget`] and answers a pick with [`UnitInput::insert`].
//!
//! The edit buffer, focus, drag and unit choice are widget state, so a screen only keeps the SI value:
//!
//! ```ignore
//! unit_input("Root chord", self.root_chord, Quantity::Length, &self.units)
//!     .on_scrub(Message::RootChord)
//!     .on_commit(Message::RootChord)
//! ```

use iced::advanced::layout::{self, Layout};
use iced::advanced::renderer::{self, Quad};
use iced::advanced::text::{Alignment, Ellipsis, LineHeight, Paragraph, Shaping, Text, Wrapping};
use iced::advanced::widget::operation::{Focusable, TextInput as TextInputState};
use iced::advanced::widget::{tree, Id, Meta, Operation, Tree};
use iced::advanced::{shell, Renderer as _, Shell, Widget};
use iced::widget::text_input;
use iced::{alignment, keyboard, mouse, Background, Border, Color, Element, Event, Font, Length, Pixels, Point, Rectangle, Renderer, Size, Theme};

use super::dashed;
use crate::theme::{tokens, MONO, SANS};
use crate::tokens::{text as type_scale, CONTROL_H, SPACE_2};
use crate::units::{self, Quantity, Unit, UnitSystem};

type TextRenderer = Renderer;

/// Width of the scrub grip at the field's left edge.
const GRIP_W: f32 = 16.0;
/// Height of a badge.
const BADGE_H: f32 = 16.0;
/// Default width of the label column.
pub const LABEL_W: f32 = super::field::LABEL_W;

/// How the field looks and behaves beyond the default. Focus and drag are tracked by the widget itself.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum State {
    /// A plain value.
    #[default]
    Normal,
    /// The value is the result of a formula: `ƒ` badge. Editing shows the text given to [`UnitInput::formula`].
    Formula,
    /// A derived value: dashed border, no fill, `ink-muted` value, `AUTO` badge. Typing overrides it.
    Auto,
    /// An overridden value (mass): the value is `warn`, `OVR` badge.
    Override,
    /// `err` border and this message under the field; the last valid value is kept.
    Error(String),
    /// This field is the one being edited in pick mode: `accent` border plus an inset ring.
    Picking,
    /// Another field is in pick mode and this one can be clicked to insert a reference: dashed `accent` outline.
    PickTarget,
}

/// The small tag before the unit chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Badge<'a> {
    /// `AUTO`: derived from other values.
    Auto,
    /// `OVR`: the computed value is overridden.
    Override,
    /// `ƒ`: the value is a formula.
    Formula,
    /// `→ N`: this value is referenced by N other fields (the field also gets an `accent` bar on its left edge).
    Refs(usize),
    /// `PER CONFIG`: stored per motor configuration.
    PerConfig,
    /// Any other short tag.
    Text(&'a str),
}

impl Badge<'_> {
    /// The text of the badge.
    pub fn label(&self) -> String {
        match self {
            Badge::Auto => "AUTO".into(),
            Badge::Override => "OVR".into(),
            Badge::Formula => "ƒ".into(),
            Badge::Refs(n) => format!("→ {n}"),
            Badge::PerConfig => "PER CONFIG".into(),
            Badge::Text(s) => (*s).into(),
        }
    }
}

/// Which colour the value text takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ValueTone {
    Ink,
    Muted,
    Warn,
    Accent,
}

#[derive(Debug, Clone)]
enum Inner {
    Input(String),
    Submit,
}

/// A numeric field with a physical quantity. Create it with [`unit_input`] or [`UnitInput::new`] and convert it to an
/// element with `.into()`.
pub struct UnitInput<'a, M> {
    label: &'a str,
    label_width: f32,
    value_si: f64,
    quantity: Quantity,
    system: UnitSystem,
    state: State,
    badge: Option<Badge<'a>>,
    hint: Option<&'a str>,
    formula: Option<&'a str>,
    insert: Option<(u64, &'a str)>,
    id: Option<Id>,
    on_commit: Option<Box<dyn Fn(f64) -> M + 'a>>,
    on_scrub: Option<Box<dyn Fn(f64) -> M + 'a>>,
    on_formula: Option<Box<dyn Fn(String) -> M + 'a>>,
    on_unit: Option<Box<dyn Fn(&'static Unit) -> M + 'a>>,
    on_pick_mode: Option<Box<dyn Fn(bool) -> M + 'a>>,
    on_pick: Option<M>,
    resolve: Option<Box<dyn Fn(&str) -> Option<f64> + 'a>>,
    input: Element<'a, Inner>,
}

/// A [`UnitInput`] showing `value_si` (SI) as `quantity` in the unit `units` selects for it.
pub fn unit_input<'a, M: Clone + 'a>(label: &'a str, value_si: f64, quantity: Quantity, units: &UnitSystem) -> UnitInput<'a, M> {
    UnitInput::new(label, value_si, quantity, units)
}

/// What the widget remembers between frames.
struct Memory {
    focused: bool,
    editing: bool,
    buffer: String,
    initial: String,
    unit: Option<&'static Unit>,
    drag: Option<Drag>,
    mods: keyboard::Modifiers,
    error: Option<String>,
    picking: bool,
    insert_seen: Option<u64>,
    unit_w: f32,
    badge_w: f32,
    hint_h: f32,
}

#[derive(Clone, Copy)]
struct Drag {
    last_x: f32,
    /// The value in display units, unquantised.
    shown: f64,
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            focused: false,
            editing: false,
            buffer: String::new(),
            initial: String::new(),
            unit: None,
            drag: None,
            mods: keyboard::Modifiers::default(),
            error: None,
            picking: false,
            insert_seen: None,
            unit_w: 0.0,
            badge_w: 0.0,
            hint_h: 0.0,
        }
    }
}

/// The regions of a field, in absolute coordinates.
struct Geo {
    label: Rectangle,
    field: Rectangle,
    grip: Rectangle,
    text: Rectangle,
    badge: Option<Rectangle>,
    unit: Rectangle,
    hint: Rectangle,
}

impl<'a, M: Clone + 'a> UnitInput<'a, M> {
    /// A field for `value_si` of `quantity`, displayed in the unit `system` selects.
    pub fn new(label: &'a str, value_si: f64, quantity: Quantity, system: &UnitSystem) -> Self {
        let unit = quantity.unit_for(*system);
        Self {
            label,
            label_width: LABEL_W,
            value_si,
            quantity,
            system: *system,
            state: State::Normal,
            badge: None,
            hint: None,
            formula: None,
            insert: None,
            id: None,
            on_commit: None,
            on_scrub: None,
            on_formula: None,
            on_unit: None,
            on_pick_mode: None,
            on_pick: None,
            resolve: None,
            input: make_input(&units::format_in(value_si, unit), ValueTone::Ink, None),
        }
    }

    /// Published with the new SI value when an edit is committed (Enter, blur) and when a scrub ends.
    pub fn on_commit(mut self, f: impl Fn(f64) -> M + 'a) -> Self {
        self.on_commit = Some(Box::new(f));
        self
    }

    /// Published with the new SI value on every step of a scrub, so the model and viewport follow live.
    pub fn on_scrub(mut self, f: impl Fn(f64) -> M + 'a) -> Self {
        self.on_scrub = Some(Box::new(f));
        self
    }

    /// Published with the raw text (`0.5 * {body_tube#3.od}`) instead of [`on_commit`](Self::on_commit) when a committed
    /// expression contains references. The expression has already been evaluated with the [`resolver`](Self::resolver).
    pub fn on_formula(mut self, f: impl Fn(String) -> M + 'a) -> Self {
        self.on_formula = Some(Box::new(f));
        self
    }

    /// Published when the unit chip is clicked, with the new display unit.
    pub fn on_unit(mut self, f: impl Fn(&'static Unit) -> M + 'a) -> Self {
        self.on_unit = Some(Box::new(f));
        self
    }

    /// Published with `true` when typing an unclosed `{` enters pick mode and `false` when it ends.
    pub fn on_pick_mode(mut self, f: impl Fn(bool) -> M + 'a) -> Self {
        self.on_pick_mode = Some(Box::new(f));
        self
    }

    /// Published when this field is clicked while it is a [`State::PickTarget`].
    pub fn on_pick(mut self, message: M) -> Self {
        self.on_pick = Some(message);
        self
    }

    /// Resolves `{id}` references to SI values, for evaluating formulas.
    pub fn resolver(mut self, f: impl Fn(&str) -> Option<f64> + 'a) -> Self {
        self.resolve = Some(Box::new(f));
        self
    }

    /// Sets the look: [`State::Auto`], [`State::Override`], [`State::Formula`], [`State::Error`], pick states.
    pub fn state(mut self, state: State) -> Self {
        self.state = state;
        self
    }

    /// Overrides the badge (the default follows the state: `AUTO`, `OVR`, `ƒ`).
    pub fn badge(mut self, badge: Badge<'a>) -> Self {
        self.badge = Some(badge);
        self
    }

    /// A muted hint line under the field ("Matched to Body tube. Type to override."). Errors replace it.
    pub fn hint(mut self, hint: &'a str) -> Self {
        self.hint = Some(hint);
        self
    }

    /// The text to edit when a [`State::Formula`] field is focused (`2 * {body_tube.od}`).
    pub fn formula(mut self, text: &'a str) -> Self {
        self.formula = Some(text);
        self
    }

    /// Inserts `text` (a `{reference}`) into the edit buffer once per distinct `token`: the app's answer to a pick.
    pub fn insert(mut self, token: u64, text: &'a str) -> Self {
        self.insert = Some((token, text));
        self
    }

    /// The width of the label column (default 96; 0 hides the label for fields inside tables).
    pub fn label_width(mut self, width: f32) -> Self {
        self.label_width = width;
        self
    }

    /// A widget id, so the app can focus the field with `iced::widget::operation::focus`.
    pub fn id(mut self, id: impl Into<Id>) -> Self {
        self.id = Some(id.into());
        self
    }

    fn unit(&self, m: &Memory) -> &'static Unit {
        m.unit.unwrap_or_else(|| self.quantity.unit_for(self.system))
    }

    fn display(&self, m: &Memory) -> String {
        units::format_in(self.value_si, self.unit(m))
    }

    fn effective_badge(&self) -> Option<Badge<'a>> {
        self.badge.or(match self.state {
            State::Auto => Some(Badge::Auto),
            State::Override => Some(Badge::Override),
            State::Formula => Some(Badge::Formula),
            _ => None,
        })
    }

    fn tone(&self, m: &Memory) -> ValueTone {
        if m.drag.is_some() {
            ValueTone::Accent
        } else {
            match self.state {
                State::Auto => ValueTone::Muted,
                State::Override => ValueTone::Warn,
                _ => ValueTone::Ink,
            }
        }
    }

    fn hint_text(&self, m: &Memory) -> Option<(String, bool)> {
        match (&self.state, &m.error) {
            (State::Error(msg), _) => Some((msg.clone(), true)),
            (_, Some(msg)) => Some((msg.clone(), true)),
            _ => self.hint.map(|h| (h.to_owned(), false)),
        }
    }

    /// Rebuilds the inner text input for what should be showing now.
    fn sync(&mut self, m: &mut Memory) {
        if let Some((token, text)) = self.insert {
            if m.insert_seen != Some(token) {
                m.insert_seen = Some(token);
                if m.editing {
                    m.buffer.push_str(text);
                    m.picking = unclosed_brace(&m.buffer);
                }
            }
        }
        let shown = if m.editing { m.buffer.clone() } else { self.display(m) };
        self.input = make_input(&shown, self.tone(m), self.id.clone());
    }

    fn geometry(&self, bounds: Rectangle, m: &Memory) -> Geo {
        let gap = if self.label_width > 0.0 { SPACE_2 } else { 0.0 };
        let field = Rectangle { x: bounds.x + self.label_width + gap, y: bounds.y, width: (bounds.width - self.label_width - gap).max(0.0), height: CONTROL_H };
        let unit = Rectangle { x: field.x + field.width - m.unit_w - 2.0, y: field.y + 2.0, width: m.unit_w, height: CONTROL_H - 4.0 };
        let badge =
            (m.badge_w > 0.0).then(|| Rectangle { x: unit.x - 4.0 - m.badge_w, y: field.y + (CONTROL_H - BADGE_H) / 2.0, width: m.badge_w, height: BADGE_H });
        let right = badge.map_or(unit.x, |b| b.x) - 2.0;
        let grip = Rectangle { x: field.x, y: field.y, width: GRIP_W, height: CONTROL_H };
        let text = Rectangle { x: field.x + GRIP_W, y: field.y + 1.0, width: (right - field.x - GRIP_W).max(0.0), height: CONTROL_H - 2.0 };
        Geo {
            label: Rectangle { x: bounds.x, y: bounds.y, width: self.label_width, height: CONTROL_H },
            field,
            grip,
            text,
            badge,
            unit,
            hint: Rectangle { x: field.x, y: bounds.y + CONTROL_H + 4.0, width: field.width, height: m.hint_h },
        }
    }

    /// Commits the edit buffer: parses it and publishes the result, or keeps the last valid value and records why not.
    fn commit(&self, m: &mut Memory, shell: &mut Shell<'_, M>) {
        let text = std::mem::take(&mut m.buffer);
        let changed = text != m.initial;
        m.editing = false;
        m.picking = false;
        if !changed {
            m.error = None;
            return;
        }
        let text = text.trim();
        if text.is_empty() {
            m.error = None;
            return;
        }
        let none = |_: &str| None;
        let resolve = |id: &str| self.resolve.as_ref().and_then(|f| f(id));
        let resolver: &dyn Fn(&str) -> Option<f64> = if self.resolve.is_some() { &resolve } else { &none };
        match units::parse_quantity(text, self.quantity, self.unit(m), resolver) {
            Ok(si) => {
                m.error = None;
                match (&self.on_formula, text.contains('{')) {
                    (Some(on_formula), true) => shell.publish(on_formula(text.to_owned())),
                    _ => {
                        if let Some(on_commit) = &self.on_commit {
                            shell.publish(on_commit(si));
                        }
                    }
                }
            }
            Err(message) => m.error = Some(message),
        }
    }

    fn revert(m: &mut Memory) {
        m.editing = false;
        m.picking = false;
        m.buffer.clear();
        m.error = None;
    }
}

fn make_input<'a>(shown: &str, tone: ValueTone, id: Option<Id>) -> Element<'a, Inner> {
    let mut input =
        text_input("", shown.to_owned()).on_input(Inner::Input).on_submit(Inner::Submit).font(MONO).size(type_scale::NUM_BODY.size).padding([3.0, 4.0]).style(
            move |theme: &Theme, _| {
                let t = tokens(theme);
                text_input::Style {
                    background: Background::Color(Color::TRANSPARENT),
                    border: Border::default(),
                    placeholder: t.ink_faint,
                    value: match tone {
                        ValueTone::Ink => t.ink,
                        ValueTone::Muted => t.ink_muted,
                        ValueTone::Warn => t.warn,
                        ValueTone::Accent => t.accent,
                    },
                    selection: t.accent.scale_alpha(0.35),
                }
            },
        );
    if let Some(id) = id {
        input = input.id(id);
    }
    iced::Widget::boxed(input)
}

/// True when `text` has a `{` that is not yet closed: the field is in pick mode.
fn unclosed_brace(text: &str) -> bool {
    match (text.rfind('{'), text.rfind('}')) {
        (Some(open), Some(close)) => open > close,
        (Some(_), None) => true,
        _ => false,
    }
}

/// One scrub step: moves the display value `shown` by `dx` pixels of `step` display units each, ×0.1 with Shift and ×10
/// with Ctrl/⌘, and returns the new unquantised value with the value to publish (quantised to the effective step).
fn scrub(shown: f64, dx: f32, step: f64, mods: keyboard::Modifiers) -> (f64, f64) {
    let scale = if mods.shift() {
        0.1
    } else if mods.command() || mods.control() {
        10.0
    } else {
        1.0
    };
    let moved = shown + f64::from(dx) * step * scale;
    let quantum = step * scale.min(1.0);
    (moved, (moved / quantum).round() * quantum)
}

struct Probe {
    focused: bool,
}

impl Operation for Probe {
    fn traverse(&mut self, operate: &mut dyn FnMut(&mut dyn Operation)) {
        operate(self);
    }

    fn focusable(&mut self, _: Option<&Id>, _: Rectangle, state: &mut dyn Focusable) {
        self.focused = state.is_focused();
    }
}

#[derive(Clone, Copy)]
enum Act {
    Focus,
    Unfocus,
    SelectAll,
    MoveToEnd,
}

struct Do(Act);

impl Operation for Do {
    fn traverse(&mut self, operate: &mut dyn FnMut(&mut dyn Operation)) {
        operate(self);
    }

    fn focusable(&mut self, _: Option<&Id>, _: Rectangle, state: &mut dyn Focusable) {
        match self.0 {
            Act::Focus => state.focus(),
            Act::Unfocus => state.unfocus(),
            _ => {}
        }
    }

    fn text_input(&mut self, _: Option<&Id>, _: Rectangle, state: &mut dyn TextInputState) {
        match self.0 {
            Act::SelectAll => state.select_all(),
            Act::MoveToEnd => state.move_cursor_to_end(),
            _ => {}
        }
    }
}

fn measure(text: &str, size: f32, font: Font, bounds: Size, wrapping: Wrapping) -> Size {
    <TextRenderer as iced::advanced::text::Renderer>::Paragraph::with_text(Text {
        content: text,
        bounds,
        size: Pixels(size),
        line_height: LineHeight::default(),
        font,
        align_x: Alignment::Default,
        align_y: alignment::Vertical::Top,
        shaping: Shaping::Auto,
        wrapping,
        ellipsis: Ellipsis::None,
        hint_factor: None,
    })
    .min_bounds()
}

#[allow(clippy::too_many_arguments)]
fn put(
    r: &mut Renderer,
    content: &str,
    at: Rectangle,
    size: f32,
    font: Font,
    color: Color,
    align_x: Alignment,
    wrapping: Wrapping,
    ellipsis: Ellipsis,
    clip: Rectangle,
) {
    use iced::advanced::text::Renderer as _;
    r.fill_text(
        Text {
            content: content.to_owned(),
            bounds: at.size(),
            size: Pixels(size),
            line_height: LineHeight::default(),
            font,
            align_x,
            align_y: if wrapping == Wrapping::None { alignment::Vertical::Center } else { alignment::Vertical::Top },
            shaping: Shaping::Auto,
            wrapping,
            ellipsis,
            hint_factor: r.hint_factor(),
        },
        Point::new(
            match align_x {
                Alignment::Center => at.center_x(),
                Alignment::Right => at.x + at.width,
                _ => at.x,
            },
            if wrapping == Wrapping::None { at.center_y() } else { at.y },
        ),
        color,
        clip,
    );
}

fn outline(r: &mut Renderer, bounds: Rectangle, color: Color, fill: Option<Color>) {
    r.fill_quad(
        Quad { bounds, border: Border { color, width: 1.0, radius: 0.0.into() }, snap: true, ..Quad::default() },
        Background::Color(fill.unwrap_or(Color::TRANSPARENT)),
    );
}

/// Carries what a child [`Shell`] asked for (redraws, layout, clipboard, capture) over to its parent.
fn forward<M, I>(parent: &mut Shell<'_, M>, child: &mut Shell<'_, I>) {
    parent.request_redraw_at(child.redraw_request());
    parent.invalidate(child.invalidation());
    if child.is_event_captured() {
        parent.capture_event();
    }
    parent.request_input_method(child.input_method());
    parent.clipboard_mut().merge(child.clipboard_mut());
}

impl<'a, M: Clone + 'a> Meta for UnitInput<'a, M> {}

impl<'a, M: Clone + 'a> Widget<M, Theme, Renderer> for UnitInput<'a, M> {
    fn tag(&self) -> tree::Tag {
        tree::Tag::of::<Memory>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(Memory::default())
    }

    fn size(&self) -> Size<Length> {
        Size::new(Length::Fill, Length::Shrink)
    }

    fn diff(&mut self, tree: &mut Tree) {
        tree.diff_children(std::slice::from_mut(&mut self.input));
    }

    fn layout(&mut self, tree: &mut Tree, renderer: &Renderer, limits: &layout::Limits) {
        let m = tree.state.downcast_mut::<Memory>();
        self.sync(m);

        let unit = self.unit(m);
        m.unit_w = measure(unit.symbol, 11.0, MONO, Size::INFINITE, Wrapping::None).width + 10.0;
        m.badge_w = match self.effective_badge() {
            Some(Badge::Formula) => BADGE_H,
            Some(b) => measure(&b.label(), 10.0, MONO, Size::INFINITE, Wrapping::None).width + 8.0,
            None => 0.0,
        };

        let width = match limits.bounds().width {
            w if w.is_finite() => w,
            _ => 304.0,
        };
        let probe = self.geometry(Rectangle::new(Point::ORIGIN, Size::new(width, CONTROL_H)), m);
        m.hint_h = match self.hint_text(m) {
            Some((text, _)) => measure(&text, 12.0, SANS, Size::new(probe.field.width, f32::INFINITY), Wrapping::Word).height,
            None => 0.0,
        };
        let height = CONTROL_H + if m.hint_h > 0.0 { 4.0 + m.hint_h } else { 0.0 };
        let geo = self.geometry(Rectangle::new(Point::ORIGIN, Size::new(width, height)), m);

        let child_limits = layout::Limits::new(Size::ZERO, Size::new(geo.text.width, geo.text.height));
        self.input.layout(&mut tree.children[0], renderer, &child_limits);
        let child = &mut tree.children[0];
        child.size.width = geo.text.width;
        child.translation = iced::Vector::new(geo.text.x, geo.text.y + (geo.text.height - child.size.height) / 2.0);
        tree.size = limits.resolve(Length::Fill, Length::Shrink, Size::new(width, height));
    }

    fn operate(&mut self, tree: &mut Tree, layout: Layout, viewport: &Rectangle, renderer: &Renderer, operation: &mut dyn Operation) {
        let Some((child_layout, child_tree)) = layout.iter_mut(&mut tree.children).next() else { return };
        operation.traverse(&mut |operation| self.input.operate(child_tree, child_layout, viewport, renderer, operation));
    }

    fn update(
        &mut self,
        tree: &mut Tree,
        event: &Event,
        layout: Layout,
        cursor: mouse::Cursor,
        renderer: &Renderer,
        shell: &mut Shell<'_, M>,
        viewport: &Rectangle,
    ) {
        let bounds = layout.bounds();
        let Some((child_layout, child_tree)) = layout.iter_mut(&mut tree.children).next() else { return };
        let m = tree.state.downcast_mut::<Memory>();
        let geo = self.geometry(bounds, m);

        if let Event::Keyboard(key_event) = event {
            match key_event {
                keyboard::Event::ModifiersChanged(mods) => m.mods = *mods,
                keyboard::Event::KeyPressed { modifiers, .. } | keyboard::Event::KeyReleased { modifiers, .. } => m.mods = *modifiers,
            }
            if m.focused {
                if let keyboard::Event::KeyPressed { key: keyboard::Key::Named(keyboard::key::Named::Escape), .. } = key_event {
                    Self::revert(m);
                    self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
                    m.focused = false;
                    self.sync(m);
                    shell.invalidate_layout();
                    shell.request_redraw();
                    shell.capture_event();
                    return;
                }
            }
        }

        // A scrub in progress owns the mouse until the button is released.
        if let Some(drag) = m.drag {
            match event {
                Event::Mouse(mouse::Event::CursorMoved { position }) => {
                    let unit = self.unit(m);
                    let (shown, value) = scrub(drag.shown, position.x - drag.last_x, unit.step(), m.mods);
                    m.drag = Some(Drag { last_x: position.x, shown });
                    let si = unit.to_si(value);
                    if (si - self.value_si).abs() > f64::EPSILON {
                        if let Some(on_scrub) = &self.on_scrub {
                            shell.publish(on_scrub(si));
                        }
                    }
                    shell.request_redraw();
                    shell.capture_event();
                }
                Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                    m.drag = None;
                    if let Some(on_commit) = &self.on_commit {
                        shell.publish(on_commit(self.unit(m).to_si((drag.shown / self.unit(m).step() / 1.0).round() * self.unit(m).step())));
                    }
                    shell.request_redraw();
                    shell.capture_event();
                }
                _ => {}
            }
            return;
        }

        if let (Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)), Some(p)) = (event, cursor.position()) {
            if geo.unit.contains(p) {
                let next = self.quantity.next_unit(self.unit(m));
                m.unit = Some(next);
                Self::revert(m);
                if m.focused {
                    self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
                    m.focused = false;
                }
                if let Some(on_unit) = &self.on_unit {
                    shell.publish(on_unit(next));
                }
                self.sync(m);
                shell.invalidate_layout();
                shell.request_redraw();
                shell.capture_event();
                return;
            }
            let scrub_start = geo.grip.contains(p) || (m.mods.alt() && geo.field.contains(p));
            if scrub_start && self.on_scrub.is_some() | self.on_commit.is_some() {
                if m.focused {
                    self.commit(m, shell);
                    self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
                    m.focused = false;
                }
                let unit = self.unit(m);
                m.drag = Some(Drag { last_x: p.x, shown: unit.from_si(self.value_si) });
                self.sync(m);
                shell.invalidate_layout();
                shell.request_redraw();
                shell.capture_event();
                return;
            }
            if self.state == State::PickTarget && geo.field.contains(p) {
                if let Some(message) = &self.on_pick {
                    shell.publish(message.clone());
                }
                shell.capture_event();
                return;
            }
        }

        // Everything else goes to the text input.
        let mut bus = shell::Bus::new();
        let mut local = shell.local(&mut bus);
        self.input.update(child_tree, event, child_layout, cursor, renderer, &mut local, viewport);
        forward(shell, &mut local);
        drop(local);
        let mut submitted = false;
        for (message, _receipt) in bus.drain() {
            match message {
                Inner::Input(text) => {
                    m.buffer = text;
                    m.editing = true;
                    m.error = None;
                    let picking = unclosed_brace(&m.buffer);
                    if picking != m.picking {
                        m.picking = picking;
                        if let Some(on_pick_mode) = &self.on_pick_mode {
                            shell.publish(on_pick_mode(picking));
                        }
                    }
                }
                Inner::Submit => submitted = true,
            }
        }

        let mut probe = Probe { focused: false };
        self.input.operate(child_tree, child_layout, viewport, renderer, &mut probe);
        let (was, now) = (m.focused, probe.focused);
        m.focused = now;

        if !was && now {
            // Focus gained: start editing the shown value (or the formula behind it), all selected.
            let start = match (&self.state, self.formula) {
                (State::Formula, Some(formula)) => formula.to_owned(),
                _ => self.display(m),
            };
            let differs = start != self.display(m);
            m.initial = start.clone();
            m.buffer = start;
            m.editing = true;
            m.error = None;
            self.sync(m);
            if differs {
                shell.invalidate_layout();
                self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::MoveToEnd));
            } else {
                self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::SelectAll));
            }
            shell.request_redraw();
        } else if was && (submitted || !now) {
            if !now && m.picking {
                // A click on another field to pick a reference must not end the edit.
                self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Focus));
                m.focused = true;
            } else {
                self.commit(m, shell);
                if now {
                    self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
                    m.focused = false;
                }
                self.sync(m);
                shell.invalidate_layout();
                shell.request_redraw();
            }
        } else if was != now {
            shell.request_redraw();
        }
    }

    fn mouse_interaction(&self, tree: &Tree, layout: Layout, cursor: mouse::Cursor, viewport: &Rectangle, renderer: &Renderer) -> mouse::Interaction {
        let m = tree.state.downcast_ref::<Memory>();
        let geo = self.geometry(layout.bounds(), m);
        if m.drag.is_some() {
            return mouse::Interaction::ResizingHorizontally;
        }
        let Some(p) = cursor.position() else { return mouse::Interaction::None };
        if geo.unit.contains(p) {
            mouse::Interaction::Pointer
        } else if geo.grip.contains(p) {
            mouse::Interaction::ResizingHorizontally
        } else if self.state == State::PickTarget && geo.field.contains(p) {
            mouse::Interaction::Crosshair
        } else if let Some((child_layout, child_tree)) = layout.iter(&tree.children).next() {
            self.input.mouse_interaction(child_tree, child_layout, cursor, viewport, renderer)
        } else {
            mouse::Interaction::None
        }
    }

    fn draw(&self, tree: &Tree, renderer: &mut Renderer, theme: &Theme, style: &renderer::Style, layout: Layout, cursor: mouse::Cursor, viewport: &Rectangle) {
        let m = tree.state.downcast_ref::<Memory>();
        let t = tokens(theme);
        let geo = self.geometry(layout.bounds(), m);
        let field = geo.field;
        let hovered = cursor.position().is_some_and(|p| field.contains(p));
        let dragging = m.drag.is_some();
        let picking = self.state == State::Picking || m.picking;
        let failed = matches!(self.state, State::Error(_)) || m.error.is_some();

        // Label column.
        if self.label_width > 0.0 {
            put(renderer, self.label, geo.label, 13.0, SANS, t.ink_muted, Alignment::Default, Wrapping::None, Ellipsis::End, geo.label);
        }

        // Frame.
        let live = m.focused || dragging || picking;
        let border = if failed {
            t.err
        } else if live || self.state == State::PickTarget {
            t.accent
        } else if hovered {
            t.ink_faint
        } else {
            t.line_200
        };
        let dashed_frame = matches!(self.state, State::Auto | State::PickTarget) && !live && !failed;
        let fill = (self.state != State::Auto || live).then_some(t.bg_300);
        if dashed_frame {
            if let Some(fill) = fill {
                renderer.fill_quad(Quad { bounds: field, snap: true, ..Quad::default() }, Background::Color(fill));
            }
            dashed::draw(renderer, field, border);
        } else {
            outline(renderer, field, border, fill);
        }
        if picking {
            outline(renderer, field.shrink(2.0), t.accent.scale_alpha(0.45), Some(t.accent_soft));
        }
        if matches!(self.effective_badge(), Some(Badge::Refs(_))) {
            renderer.fill_quad(Quad { bounds: Rectangle { width: 2.0, ..field }, snap: true, ..Quad::default() }, Background::Color(t.accent));
        }

        // Scrub grip: three hairlines.
        let grip_color = if dragging { t.accent } else { t.ink_faint };
        for dx in [5.0, 8.0, 11.0] {
            renderer.fill_quad(
                Quad { bounds: Rectangle { x: geo.grip.x + dx, y: geo.grip.y + 9.0, width: 1.0, height: 8.0 }, snap: true, ..Quad::default() },
                Background::Color(grip_color),
            );
        }

        // Value.
        if let Some((child_layout, child_tree)) = layout.iter(&tree.children).next() {
            self.input.draw(child_tree, renderer, theme, style, child_layout, cursor, viewport);
        }

        // Badge.
        if let (Some(badge), Some(rect)) = (self.effective_badge(), geo.badge) {
            let color = if badge == Badge::Formula { t.accent } else { t.ink_muted };
            let edge = if badge == Badge::Formula { t.accent } else { t.line_200 };
            outline(renderer, rect, edge, None);
            put(
                renderer,
                &badge.label(),
                rect,
                if badge == Badge::Formula { 11.0 } else { 10.0 },
                MONO,
                color,
                Alignment::Center,
                Wrapping::None,
                Ellipsis::None,
                rect,
            );
        }

        // Unit chip.
        let unit_hovered = cursor.position().is_some_and(|p| geo.unit.contains(p));
        if unit_hovered {
            renderer.fill_quad(Quad { bounds: geo.unit, snap: true, ..Quad::default() }, Background::Color(t.bg_400));
        }
        put(
            renderer,
            self.unit(m).symbol,
            geo.unit,
            11.0,
            MONO,
            if unit_hovered { t.ink } else { t.ink_muted },
            Alignment::Center,
            Wrapping::None,
            Ellipsis::None,
            geo.unit,
        );

        // Hint or error line.
        if let Some((text, is_error)) = self.hint_text(m) {
            put(
                renderer,
                &text,
                geo.hint,
                12.0,
                SANS,
                if is_error { t.err } else { t.ink_muted },
                Alignment::Default,
                Wrapping::Word,
                Ellipsis::None,
                geo.hint,
            );
        }
    }
}

impl<'a, M: Clone + 'a> From<UnitInput<'a, M>> for Element<'a, M> {
    fn from(input: UnitInput<'a, M>) -> Self {
        iced::Widget::boxed(input)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scrub_moves_one_display_step_per_pixel() {
        // Section 4 of design/README.md: mm shows two decimals, so one pixel is 0.01 mm.
        let step = units::Quantity::Length.unit_for(units::UnitSystem::MmGS).step();
        let (_, value) = scrub(100.0, 5.0, step, keyboard::Modifiers::default());
        assert!((value - 100.05).abs() < 1e-9);
        let (_, value) = scrub(100.0, -3.0, step, keyboard::Modifiers::default());
        assert!((value - 99.97).abs() < 1e-9);
    }

    #[test]
    fn shift_is_a_tenth_and_ctrl_is_ten_times() {
        let (_, fine) = scrub(0.0, 10.0, 0.01, keyboard::Modifiers::SHIFT);
        assert!((fine - 0.01).abs() < 1e-9, "{fine}");
        let (_, coarse) = scrub(0.0, 10.0, 0.01, keyboard::Modifiers::CTRL);
        assert!((coarse - 1.0).abs() < 1e-9, "{coarse}");
    }

    #[test]
    fn fine_scrub_accumulates_below_the_display_step() {
        // Ten Shift-pixels are one step; the unquantised value must carry the fraction between events.
        let mut shown = 0.0;
        for _ in 0..10 {
            shown = scrub(shown, 1.0, 0.01, keyboard::Modifiers::SHIFT).0;
        }
        assert!((shown - 0.01).abs() < 1e-9);
    }

    #[test]
    fn an_unclosed_brace_is_pick_mode() {
        assert!(unclosed_brace("0.5 * {"));
        assert!(unclosed_brace("{a} + {b"));
        assert!(!unclosed_brace("{a} + 2"));
        assert!(!unclosed_brace("2*38"));
    }

    #[test]
    fn badges_read_as_the_kit_labels_them() {
        assert_eq!(Badge::Refs(2).label(), "→ 2");
        assert_eq!(Badge::PerConfig.label(), "PER CONFIG");
        assert_eq!(Badge::Override.label(), "OVR");
    }
}
