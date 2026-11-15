# InsideViews

How you see inside the rocket in 3D. These are four view modes on one control in the viewport toolbar (`1`–`4`). They change only how the model is drawn, never the model itself.

| Mode | What it does | When to use it |
|---|---|---|
| **Solid** | Materials and paint. Nothing hidden | Overall shape, fins, finish |
| **Cutaway** | A section plane along the axis removes half the airframe. Cut walls, rings and bulkheads are **hatched** (`cut-face`), the exposed inside is `cut-interior`, and internals are solid in their **system colour** | Placing and checking internals: the main working view for anything inside |
| **X-ray** | The shell becomes a faint outline (`part-body` at 10 % fill) and every part, front and back, shows in system colour | A quick look at everything at once; checking clearances |
| **Exploded** | Airframe sections separate along the axis, fins pull outward, and internals drop below their station on dashed leaders. Animates out and back in about 250 ms | Seeing stacking order; build docs and screenshots |

**System colours are shared with Station Lanes.** `part-body` = Structure, `sys-aero`, `sys-propulsion`, `sys-recovery`, `sys-avionics`. A part looks the same colour in the lanes and inside the rocket, so the two views read as one.

**Behaviour**
- **Auto-reveal (always on, any mode):** selecting an internal part (from the lanes, or `Alt`-click through the shell) ghosts the airframe around it, so you can edit a centering ring without switching modes.
- **Cutaway section handle:** drag the round handle on the axis to rotate the cut plane. Half cut (180°) is the default, and holding `Shift` snaps it to 90° steps, which gives a quarter cut like a technical drawing.
- Selection, CG/CP markers, gizmos and dimension labels draw the same way in every mode.
- Exploded view is view-only: dragging is disabled, but click still selects. Leave it with `Esc` or `1`.

**iced:** one `Uniform { mode, clip_plane: Vec4, xray_alpha, explode_t }` in the wgpu pipeline. Cutaway is a clip-plane discard plus a cap pass that fills cut faces with a screen-space hatch. X-ray uses alpha blending with the shell drawn last. Exploded offsets each component's model matrix by `explode_t × its offset`.
