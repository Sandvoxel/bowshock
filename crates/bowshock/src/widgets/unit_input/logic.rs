use super::cells::{ascii_operator, cells_from, cells_text, display_char, open_brace, CHIP_FONT, CHIP_GAP, CHIP_H, CHIP_PAD, CHIP_SQUARE};
use super::*;

impl<'a, M: Clone + 'a> UnitInput<'a, M> {
    pub(super) fn unit(&self, m: &Memory) -> &'static Unit {
        m.unit.unwrap_or_else(|| self.quantity.unit_for(self.system))
    }

    pub(super) fn display(&self, m: &Memory) -> String {
        // A mixed value shows nothing but the `—` placeholder, so focusing the field starts from an empty buffer.
        if self.mixed {
            return String::new();
        }
        units::format_in(self.value_si, self.unit(m))
    }

    /// The badge before the unit chip (`AUTO`, `OVR`, `→ N`, …). The `ƒ` of a formula is not one: see [`leading_formula`].
    pub(super) fn effective_badge(&self) -> Option<Badge<'a>> {
        match self.badge {
            Some(Badge::Formula) => None,
            Some(b) => Some(b),
            None => match self.state {
                State::Auto => Some(Badge::Auto),
                State::Override => Some(Badge::Override),
                _ => None,
            },
        }
    }

    /// Whether the `ƒ` badge sits before the value: the field is a formula (kit: UnitInput, States).
    pub(super) fn leading_formula(&self) -> bool {
        self.state == State::Formula || self.badge == Some(Badge::Formula)
    }

    pub(super) fn tone(&self, m: &Memory) -> ValueTone {
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

    pub(super) fn hint_text(&self, m: &Memory) -> Option<(String, bool)> {
        match (&self.state, &m.error) {
            (State::Error(msg), _) => Some((msg.clone(), true)),
            (_, Some(msg)) => Some((msg.clone(), true)),
            _ => self.hint.map(|h| (h.to_owned(), false)),
        }
    }

    /// Rebuilds the inner text input for what should be showing now.
    pub(super) fn sync(&mut self, m: &mut Memory) {
        if let Some((token, text)) = self.insert {
            if m.insert_seen != Some(token) {
                m.insert_seen = Some(token);
                if m.chip {
                    // The pick lands on the open `{` (or the slot of a chip being re-picked) and ends pick mode.
                    if let (Some(at), Some(chip)) = (open_brace(&m.cells), text.trim().strip_prefix('{').and_then(|t| t.strip_suffix('}'))) {
                        let end = if m.cells[at] == Cell::Char('{') { m.caret.max(at + 1).min(m.cells.len()) } else { at + 1 };
                        m.cells.splice(at..end, [Cell::Chip(chip.trim().to_owned())]);
                        m.caret = at + 1;
                    }
                    m.picking = open_brace(&m.cells).is_some();
                } else if m.editing {
                    m.buffer.push_str(text);
                    let cells = cells_from(&m.buffer);
                    m.caret = cells.len();
                    m.cells = cells;
                    m.chip = true;
                    m.picking = open_brace(&m.cells).is_some();
                }
            }
        }
        let shown = if m.editing { m.buffer.clone() } else { self.display(m) };
        self.input = make_input(&shown, self.tone(m), self.id.clone(), self.mixed);
    }

    pub(super) fn geometry(&self, bounds: Rectangle, m: &Memory) -> Geo {
        let gap = if self.label_width > 0.0 { SPACE_2 } else { 0.0 };
        let field = Rectangle { x: bounds.x + self.label_width + gap, y: bounds.y, width: (bounds.width - self.label_width - gap).max(0.0), height: CONTROL_H };
        let unit = Rectangle { x: field.x + field.width - m.unit_w - 2.0, y: field.y + 2.0, width: m.unit_w, height: CONTROL_H - 4.0 };
        // The `ƒ` of a formula sits before the value (kit: UnitInput, States); every other badge sits before the unit.
        let leading = self.leading_formula();
        let by = field.y + (CONTROL_H - BADGE_H) / 2.0;
        let formula = leading.then_some(Rectangle { x: field.x + GRIP_W, y: by, width: BADGE_H, height: BADGE_H });
        let badge = (m.badge_w > 0.0).then_some(Rectangle { x: unit.x - 4.0 - m.badge_w, y: by, width: m.badge_w, height: BADGE_H });
        let right = badge.map_or(unit.x, |b| b.x) - 2.0;
        let grip = Rectangle { x: field.x, y: field.y, width: GRIP_W, height: CONTROL_H };
        let left = field.x + GRIP_W + if leading { BADGE_H + 2.0 } else { 0.0 };
        let text = Rectangle { x: left, y: field.y + 1.0, width: (right - left).max(0.0), height: CONTROL_H - 2.0 };
        Geo {
            label: Rectangle { x: bounds.x, y: bounds.y, width: self.label_width, height: CONTROL_H },
            field,
            grip,
            text,
            badge,
            formula,
            unit,
            hint: Rectangle { x: field.x, y: bounds.y + CONTROL_H + 4.0, width: field.width, height: m.hint_h },
        }
    }

    /// Commits the edit buffer: parses it and publishes the result, or keeps the last valid value and records why not.
    pub(super) fn commit(&self, m: &mut Memory, shell: &mut Shell<'_, M>) {
        if m.chip {
            m.buffer = cells_text(&m.cells);
        }
        let text = std::mem::take(&mut m.buffer);
        let changed = text != m.initial;
        m.editing = false;
        m.picking = false;
        m.chip = false;
        m.cells.clear();
        self.tell_pick_mode(m, shell);
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
                    (Some(on_formula), true) => shell.publish(on_formula(text.to_owned(), self.unit(m))),
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

    pub(super) fn revert(m: &mut Memory) {
        m.editing = false;
        m.picking = false;
        m.chip = false;
        m.cells.clear();
        m.buffer.clear();
        m.error = None;
    }

    /// Tells the app when pick mode starts or ends.
    pub(super) fn tell_pick_mode(&self, m: &mut Memory, shell: &mut Shell<'_, M>) {
        if m.picking != m.told_picking {
            m.told_picking = m.picking;
            if let Some(on_pick_mode) = &self.on_pick_mode {
                shell.publish(on_pick_mode(m.picking));
            }
        }
    }

    /// Starts editing `text` as cells (a formula with references), with the caret at its end.
    pub(super) fn start_cells(m: &mut Memory, text: &str) {
        m.cells = cells_from(text);
        m.caret = m.cells.len();
        m.chip = true;
        m.editing = true;
        m.focused = true;
        m.picking = open_brace(&m.cells).is_some();
    }

    /// How a cell shows: the width of a character, the label and width of a chip.
    pub(super) fn cell_label(&self, cell: &Cell) -> Option<(String, Option<Chip>)> {
        match cell {
            Cell::Char(_) => None,
            Cell::Chip(token) => match self.chips.as_ref().and_then(|f| f(token)) {
                Some(chip) => Some((chip.label.clone(), Some(chip))),
                None => Some((token.clone(), None)),
            },
            Cell::Slot(_) => Some(("pick…".to_owned(), None)),
        }
    }

    /// The x extent of each cell (chips include their gaps) with `text` the area they are laid out in, scrolled so the caret is
    /// visible, and the x of the caret.
    pub(super) fn cell_extents(&self, m: &Memory, text: Rectangle) -> (Vec<(f32, f32)>, f32) {
        let char_w = measure("0", type_scale::NUM_BODY.size, MONO, Size::INFINITE, Wrapping::None).width;
        let mut x = text.x + 4.0;
        let mut out = Vec::with_capacity(m.cells.len());
        for cell in &m.cells {
            let w = match self.cell_label(cell) {
                None => char_w,
                Some((label, _)) => {
                    CHIP_GAP * 2.0 + CHIP_PAD * 2.0 + CHIP_SQUARE + 4.0 + measure(&label, CHIP_FONT, MONO, Size::INFINITE, Wrapping::None).width
                }
            };
            out.push((x, x + w));
            x += w;
        }
        let caret = |out: &[(f32, f32)]| match m.caret {
            0 => text.x + 4.0,
            i => out.get(i - 1).map_or(x, |e| e.1),
        };
        let shift = (caret(&out) + 6.0 - (text.x + text.width)).max(0.0);
        for e in &mut out {
            e.0 -= shift;
            e.1 -= shift;
        }
        let at = caret(&out);
        (out, at)
    }

    /// Handles an event while the field edits as cells. Returns whether the event was used up.
    pub(super) fn chip_event(&self, m: &mut Memory, event: &Event, geo: &Geo, cursor: mouse::Cursor, shell: &mut Shell<'_, M>) -> bool {
        use keyboard::key::Named;
        match event {
            Event::Keyboard(keyboard::Event::KeyPressed { key, text, modifiers, .. }) if m.focused => {
                match key.as_ref() {
                    keyboard::Key::Named(Named::Enter) => {
                        self.commit(m, shell);
                        m.focused = false;
                        shell.invalidate_layout();
                        shell.request_redraw();
                        return true;
                    }
                    keyboard::Key::Named(Named::Backspace) if m.caret > 0 => {
                        m.cells.remove(m.caret - 1);
                        m.caret -= 1;
                    }
                    keyboard::Key::Named(Named::Delete) if m.caret < m.cells.len() => {
                        m.cells.remove(m.caret);
                    }
                    keyboard::Key::Named(Named::ArrowLeft) => m.caret = m.caret.saturating_sub(1),
                    keyboard::Key::Named(Named::ArrowRight) => m.caret = (m.caret + 1).min(m.cells.len()),
                    keyboard::Key::Named(Named::Home) => m.caret = 0,
                    keyboard::Key::Named(Named::End) => m.caret = m.cells.len(),
                    _ if modifiers.command() || modifiers.control() => return false,
                    _ => {
                        let Some(text) = text else { return false };
                        for c in text.chars().filter(|c| !c.is_control()) {
                            m.cells.insert(m.caret, Cell::Char(ascii_operator(c)));
                            m.caret += 1;
                            if c == '}' {
                                self.close_brace(m);
                            }
                        }
                    }
                }
                m.picking = open_brace(&m.cells).is_some();
                m.error = None;
                self.tell_pick_mode(m, shell);
                shell.invalidate_layout();
                shell.request_redraw();
                true
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let Some(p) = cursor.position() else { return false };
                if !geo.field.contains(p) {
                    if m.picking || !m.focused {
                        return false;
                    }
                    self.commit(m, shell);
                    m.focused = false;
                    shell.invalidate_layout();
                    shell.request_redraw();
                    return false;
                }
                if !geo.text.contains(p) {
                    return false;
                }
                let (extents, _) = self.cell_extents(m, geo.text);
                let hit = extents.iter().position(|(a, b)| p.x >= *a && p.x < *b);
                match hit {
                    Some(i) if !matches!(m.cells[i], Cell::Char(_)) => {
                        // A click on a chip picks it again.
                        if let Cell::Chip(token) = m.cells[i].clone() {
                            m.cells[i] = Cell::Slot(Some(token));
                        }
                        m.caret = i + 1;
                    }
                    Some(i) => m.caret = if p.x > (extents[i].0 + extents[i].1) / 2.0 { i + 1 } else { i },
                    None => m.caret = if extents.last().is_some_and(|e| p.x < e.0) { 0 } else { m.cells.len() },
                }
                m.picking = open_brace(&m.cells).is_some();
                self.tell_pick_mode(m, shell);
                shell.invalidate_layout();
                shell.request_redraw();
                true
            }
            _ => false,
        }
    }

    /// A `}` was typed before the caret: if it closes a `{token` that names a reference, the whole becomes a chip.
    pub(super) fn close_brace(&self, m: &mut Memory) {
        let close = m.caret - 1;
        let Some(open) = m.cells[..close].iter().rposition(|c| *c == Cell::Char('{')) else { return };
        let token: String = m.cells[open + 1..close].iter().filter_map(|c| if let Cell::Char(c) = c { Some(*c) } else { None }).collect();
        let token = token.trim().to_owned();
        if m.cells[open + 1..close].iter().all(|c| matches!(c, Cell::Char(_))) && self.chips.as_ref().is_some_and(|f| f(&token).is_some()) {
            m.cells.splice(open..=close, [Cell::Chip(token)]);
            m.caret = open + 1;
        }
    }

    /// Draws the cells of a formula being edited: characters as mono text, references as chips, and the caret.
    pub(super) fn draw_cells(&self, m: &Memory, r: &mut Renderer, t: &crate::tokens::Tokens, geo: &Geo) {
        let (extents, caret_x) = self.cell_extents(m, geo.text);
        let size = type_scale::NUM_BODY.size;
        let char_w = measure("0", size, MONO, Size::INFINITE, Wrapping::None).width;
        let clip = geo.text;
        r.with_layer(clip, |r| {
            let flush = |r: &mut Renderer, run: &mut Option<(f32, String)>| {
                if let Some((x, s)) = run.take() {
                    let at = Rectangle { x, y: clip.y, width: s.chars().count() as f32 * char_w + 2.0, height: clip.height };
                    put(r, &s, at, size, MONO, t.ink, Alignment::Default, Wrapping::None, Ellipsis::None, clip);
                }
            };
            let mut run: Option<(f32, String)> = None;
            for (cell, (x0, x1)) in m.cells.iter().zip(&extents) {
                let Cell::Char(c) = cell else {
                    flush(r, &mut run);
                    let Some((label, chip)) = self.cell_label(cell) else { continue };
                    let frame = Rectangle { x: x0 + CHIP_GAP, y: clip.center_y() - CHIP_H / 2.0, width: x1 - x0 - CHIP_GAP * 2.0, height: CHIP_H };
                    let broken = chip.as_ref().is_some_and(|c| c.broken) || (chip.is_none() && matches!(cell, Cell::Chip(_)));
                    if matches!(cell, Cell::Slot(_)) {
                        r.fill_quad(Quad { bounds: frame, snap: true, ..Quad::default() }, Background::Color(t.accent_soft));
                        dashed::draw(r, frame, t.accent);
                    } else {
                        outline(r, frame, if broken { t.err } else { t.ink_muted }, Some(t.bg_200));
                    }
                    let square = Rectangle { x: frame.x + CHIP_PAD, y: frame.center_y() - CHIP_SQUARE / 2.0, width: CHIP_SQUARE, height: CHIP_SQUARE };
                    let square_color = match (&chip, broken, cell) {
                        (_, _, Cell::Slot(_)) => t.accent,
                        (_, true, _) => t.err,
                        (Some(chip), _, _) => chip.color,
                        _ => t.ink_muted,
                    };
                    r.fill_quad(Quad { bounds: square, snap: true, ..Quad::default() }, Background::Color(square_color));
                    let label_at =
                        Rectangle { x: square.x + CHIP_SQUARE + 4.0, y: frame.y, width: frame.x + frame.width - square.x - CHIP_SQUARE - 4.0, height: CHIP_H };
                    let ink = if broken {
                        t.err
                    } else if matches!(cell, Cell::Slot(_)) {
                        t.accent
                    } else {
                        t.ink
                    };
                    put(r, &label, label_at, CHIP_FONT, MONO, ink, Alignment::Default, Wrapping::None, Ellipsis::None, frame);
                    if broken {
                        let used = measure(&label, CHIP_FONT, MONO, Size::INFINITE, Wrapping::None).width;
                        r.fill_quad(
                            Quad { bounds: Rectangle { x: label_at.x, y: frame.center_y(), width: used, height: 1.0 }, snap: true, ..Quad::default() },
                            Background::Color(t.err),
                        );
                    }
                    continue;
                };
                let shown = display_char(*c);
                match &mut run {
                    Some((_, s)) => s.push(shown),
                    None => run = Some((*x0, shown.to_string())),
                }
            }
            flush(r, &mut run);
            if m.focused {
                r.fill_quad(
                    Quad { bounds: Rectangle { x: caret_x, y: clip.center_y() - 7.0, width: 1.0, height: 14.0 }, snap: true, ..Quad::default() },
                    Background::Color(t.accent),
                );
            }
        });
    }
}

pub(super) fn make_input<'a>(shown: &str, tone: ValueTone, id: Option<Id>, mixed: bool) -> Element<'a, Inner> {
    let placeholder = if mixed { "—" } else { "" };
    let mut input = text_input(placeholder, shown.to_owned())
        .on_input(Inner::Input)
        .on_submit(Inner::Submit)
        .font(MONO)
        .size(type_scale::NUM_BODY.size)
        .padding([3.0, 4.0])
        .style(move |theme: &Theme, _| {
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
        });
    if let Some(id) = id {
        input = input.id(id);
    }
    iced::Widget::boxed(input)
}

/// One scrub step: moves the display value `shown` by `dx` pixels of `step` display units each, ×0.1 with Shift and ×10
/// with Ctrl/⌘, and returns the new unquantised value with the value to publish (quantised to the effective step).
pub(super) fn scrub(shown: f64, dx: f32, step: f64, mods: keyboard::Modifiers) -> (f64, f64) {
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

pub(super) struct Probe {
    pub(super) focused: bool,
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
pub(super) enum Act {
    Focus,
    Unfocus,
    SelectAll,
    MoveToEnd,
}

pub(super) struct Do(pub(super) Act);

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

pub(super) fn measure(text: &str, size: f32, font: Font, bounds: Size, wrapping: Wrapping) -> Size {
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
pub(super) fn put(
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

pub(super) fn outline(r: &mut Renderer, bounds: Rectangle, color: Color, fill: Option<Color>) {
    r.fill_quad(
        Quad { bounds, border: Border { color, width: 1.0, radius: 0.0.into() }, snap: true, ..Quad::default() },
        Background::Color(fill.unwrap_or(Color::TRANSPARENT)),
    );
}

/// Carries what a child [`Shell`] asked for (redraws, layout, clipboard, capture) over to its parent.
pub(super) fn forward<M, I>(parent: &mut Shell<'_, M>, child: &mut Shell<'_, I>) {
    parent.request_redraw_at(child.redraw_request());
    parent.invalidate(child.invalidation());
    if child.is_event_captured() {
        parent.capture_event();
    }
    parent.request_input_method(child.input_method());
    parent.clipboard_mut().merge(child.clipboard_mut());
}
