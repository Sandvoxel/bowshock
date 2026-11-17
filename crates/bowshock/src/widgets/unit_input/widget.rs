use super::cells::open_brace;
use super::logic::{forward, measure, outline, put, scrub, Act, Do, Probe};
use super::*;

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
                    if m.chip && m.picking {
                        // Esc leaves pick mode: the open `{` goes (or the chip being re-picked comes back).
                        if let Some(at) = open_brace(&m.cells) {
                            match m.cells[at].clone() {
                                Cell::Slot(Some(token)) => m.cells[at] = Cell::Chip(token),
                                _ => {
                                    m.cells.remove(at);
                                    m.caret = m.caret.min(m.cells.len());
                                }
                            }
                        }
                        m.picking = open_brace(&m.cells).is_some();
                        self.tell_pick_mode(m, shell);
                        shell.invalidate_layout();
                        shell.request_redraw();
                        shell.capture_event();
                        return;
                    }
                    Self::revert(m);
                    self.tell_pick_mode(m, shell);
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

        // Right-click on the unit chip: this unit for the whole project (every field and readout of the quantity).
        if let (Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)), Some(p)) = (event, cursor.position()) {
            if geo.unit.contains(p) {
                units::set_unit_override(self.quantity, self.unit(m));
                m.unit = None;
                Self::revert(m);
                self.sync(m);
                shell.invalidate_layout();
                shell.request_redraw();
                shell.capture_event();
                return;
            }
        }

        if let (Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)), Some(p)) = (event, cursor.position()) {
            if geo.unit.contains(p) {
                let next = self.quantity.next_unit(self.unit(m));
                m.unit = Some(next);
                Self::revert(m);
                self.tell_pick_mode(m, shell);
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
            if self.state == State::PickTarget && geo.field.contains(p) {
                if let Some(message) = &self.on_pick {
                    shell.publish(message.clone());
                }
                shell.capture_event();
                return;
            }
            if geo.badge.is_some_and(|b| b.contains(p)) {
                if let Some(message) = &self.on_badge {
                    shell.publish(message.clone());
                    shell.capture_event();
                    return;
                }
            }
            let scrub_start = geo.grip.contains(p) || (m.mods.alt() && geo.field.contains(p));
            if scrub_start && self.scrub_asks.is_some() {
                if let Some(message) = &self.scrub_asks {
                    shell.publish(message.clone());
                }
                shell.capture_event();
                return;
            }
            if scrub_start && !self.mixed && self.on_scrub.is_some() | self.on_commit.is_some() {
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
        }

        // A formula with references is edited as cells, not by the text input.
        if m.chip {
            let used = self.chip_event(m, event, &geo, cursor, shell);
            self.sync(m);
            if used {
                shell.capture_event();
            }
            return;
        }

        // Everything else goes to the text input.
        let mut bus = shell::Bus::new();
        let mut local = shell.local(&mut bus);
        self.input.update(child_tree, event, child_layout, cursor, renderer, &mut local, viewport);
        forward(shell, &mut local);
        drop(local);
        let mut submitted = false;
        let mut enter_cells: Option<String> = None;
        for (message, _receipt) in bus.drain() {
            match message {
                Inner::Input(text) => {
                    m.error = None;
                    if text.contains('{') {
                        // Typing `{` starts a reference: the field becomes a row of characters and chips.
                        enter_cells = Some(text);
                    } else {
                        m.buffer = text;
                        m.editing = true;
                    }
                }
                Inner::Submit => submitted = true,
            }
        }

        if let Some(text) = enter_cells {
            Self::start_cells(m, &text);
            self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
            self.tell_pick_mode(m, shell);
            self.sync(m);
            shell.invalidate_layout();
            shell.request_redraw();
            shell.capture_event();
            return;
        }
        let mut probe = Probe { focused: false };
        self.input.operate(child_tree, child_layout, viewport, renderer, &mut probe);
        let (was, now) = (m.focused, probe.focused);
        m.focused = now;

        if !was && now {
            // Focus gained: start editing the shown value (or the formula behind it), all selected.
            let start = match (&self.formula_in, self.formula) {
                (Some(formula), _) if self.state == State::Formula || matches!(self.state, State::Error(_)) => formula(self.unit(m)),
                (_, Some(formula)) if self.state == State::Formula => formula.to_owned(),
                _ => self.display(m),
            };
            if start.contains('{') {
                m.initial = start.clone();
                Self::start_cells(m, &start);
                m.error = None;
                self.input.operate(child_tree, child_layout, viewport, renderer, &mut Do(Act::Unfocus));
                self.tell_pick_mode(m, shell);
                self.sync(m);
                shell.invalidate_layout();
                shell.request_redraw();
                return;
            }
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
        } else if geo.badge.is_some_and(|b| b.contains(p)) && self.on_badge.is_some() {
            mouse::Interaction::Pointer
        } else if m.chip && geo.text.contains(p) {
            mouse::Interaction::Text
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
            outline(renderer, field.shrink(1.0), t.accent, None);
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
        if m.chip {
            self.draw_cells(m, renderer, t, &geo);
        } else if let Some((child_layout, child_tree)) = layout.iter(&tree.children).next() {
            self.input.draw(child_tree, renderer, theme, style, child_layout, cursor, viewport);
        }

        // Badges: the `ƒ` before the value, the rest before the unit.
        if let Some(rect) = geo.formula {
            outline(renderer, rect, t.accent, None);
            put(renderer, "ƒ", rect, 11.0, MONO, t.accent, Alignment::Center, Wrapping::None, Ellipsis::None, rect);
        }
        if let (Some(badge), Some(rect)) = (self.effective_badge(), geo.badge) {
            outline(renderer, rect, t.line_200, None);
            put(renderer, &badge.label(), rect, 10.0, MONO, t.ink_muted, Alignment::Center, Wrapping::None, Ellipsis::None, rect);
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
