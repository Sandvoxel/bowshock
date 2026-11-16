use super::logic::make_input;
use super::*;

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
            on_badge: None,
            scrub_asks: None,
            resolve: None,
            chips: None,
            formula_in: None,
            mixed: false,
            input: make_input(&units::format_in(value_si, unit), ValueTone::Ink, None, false),
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

    /// Published with the raw text (`0.5 * {body_tube#3.od}`) and the unit it was typed in instead of
    /// [`on_commit`](Self::on_commit) when a committed expression contains references. The expression has already been
    /// evaluated with the [`resolver`](Self::resolver).
    pub fn on_formula(mut self, f: impl Fn(String, &'static Unit) -> M + 'a) -> Self {
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

    /// How a reference token (`body_tube#3.od`) shows as a chip while a formula is edited; `None` leaves it as text.
    pub fn chips(mut self, f: impl Fn(&str) -> Option<Chip> + 'a) -> Self {
        self.chips = Some(Box::new(f));
        self
    }

    /// The formula text for a field in [`State::Formula`], given the unit the field is shown in (numbers in it are in that
    /// unit). Replaces [`formula`](Self::formula) when the unit can change under the field.
    pub fn formula_in(mut self, f: impl Fn(&'static Unit) -> String + 'a) -> Self {
        self.formula_in = Some(Box::new(f));
        self
    }

    /// Published when the badge is clicked (the `→ N` of a source field lists its dependants).
    pub fn on_badge(mut self, message: M) -> Self {
        self.on_badge = Some(message);
        self
    }

    /// Published instead of scrubbing when the grip is pressed: for a formula field, which must ask before it is replaced.
    pub fn on_scrub_attempt(mut self, message: M) -> Self {
        self.scrub_asks = Some(message);
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

    /// Several values at once (a multi-selection whose parts disagree): shows the faint `—` placeholder instead of a number,
    /// does not scrub, and typing a value commits it as usual (the app applies it to every part).
    pub fn mixed(mut self, mixed: bool) -> Self {
        self.mixed = mixed;
        self.input = make_input("", ValueTone::Ink, self.id.clone(), mixed);
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
}
