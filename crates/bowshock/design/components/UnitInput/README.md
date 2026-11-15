# UnitInput

A numeric field that knows its physical quantity. Every dimension in the inspector is one of these. It also takes **expressions that reference other values**, so one dimension can drive another.

**Anatomy:** label (96 px column, `ink-muted`) · scrub grip · value (`num-body`, tabular mono) or expression · optional badge (`AUTO`, `OVR`, `→ N`) · unit chip.

**Typing**
- A number, a unit-suffixed value (`3in`, `76.2mm`, `1' 2"`), or arithmetic (`50+10`, `2*38`, `(120-50)/2`). Commit with Enter or by clicking away, and Esc reverts.
- **Scrub**: drag the grip (or Alt-drag the field) sideways. 1 px = one display step, `Shift` ×0.1, `Ctrl` ×10. The model updates live (`is-drag`).
- **Unit chip**: click cycles the display unit (mm → cm → in). Right-click sets it project-wide.

**References: `{`**
1. Type `{` anywhere in an expression. The field enters **pick mode** (`accent` inset ring, accent caret) and every other numeric field on screen gets a dashed `accent` outline as a target. The viewport and Station Lanes become pickable too.
2. **Click a field** to insert it. **Click a part** (in the viewport or the lanes) to open a small menu of its values: outer/inner Ø, length, station, wall, mass…
3. The value lands as a **reference chip**: `[■ Body tube · Ø]`, where the square is the part's system colour. Keep typing maths around it: `0.5 × [Body tube · Ø] + 2`. Typing `}` or Esc leaves pick mode.
4. On commit the field shows the **result** with an `ƒ` badge. Hovering shows the formula, and clicking the field shows it again for editing.

**How references behave**
- They are **live**: change the body tube and every field that references it updates, along with CG/CP.
- They bind to the part's ID, not its name, so renaming a part never breaks them. Chips just show the new name.
- **Source side**: a field that others reference shows `→ N`. Click it to list the dependants and jump to them.
- **Scrubbing a formula field** asks first: "Replace formula with a value?" You never lose a formula by accident.
- **Errors** keep the last valid value and explain themselves under the field:
  - *Circular*: "Root chord already uses Tip chord."
  - *Broken*: the referenced part was deleted. The chip turns red and struck through, with options to re-pick or convert to a plain value.
- Works with **symmetry**: a reference to one fin of a linked group means the group's value.
- Plain-text form for files and copy/paste: `0.5 * {body_tube#3.od}`.

**States**
| State | Look |
|---|---|
| default | `bg-300`, `line-200` border |
| focus | `accent` 1 px border |
| drag | `accent` border, value and grip turn `accent` |
| pick | `accent` border + inset ring, caret after `{`, other fields dashed-`accent` |
| formula | `ƒ` badge + result value |
| auto | dashed border, no fill, `ink-muted` value + `AUTO` badge |
| override | value in `warn` + `OVR` badge |
| error | `err` border + hint line; last valid value kept |

**iced:** a custom `Widget` wrapping `text_input`. Parse expressions into a small AST (`Num | Ref(ComponentId, Prop) | BinOp`). Store `Value::Plain(f64) | Value::Expr(Ast)` per field and recompute in topological order after each edit, rejecting cycles. Pick mode is an app-level state, `Picking { target: FieldId }`, that every field and the viewport check on click.
