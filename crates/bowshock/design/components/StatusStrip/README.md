# StatusStrip

The bottom bar, always visible: the rocket's vital numbers update live as you build.

- Height `statusbar-h` (36 px), `bg-100`, `line-100` top border. Readouts are key (`ink-faint`) + value (mono `ink`), in ruled cells like an instrument panel; keys are uppercase mono.
- Order is fixed: **Mass · Length · Ø ref · CG · CP · Stability · Motor**, with the warnings chip pinned right.
- CG and CP carry their glyphs and colours. Stability gets a compact gauge (bands + tick).
- Each readout is clickable: CG/CP toggle their viewport markers, Stability opens Analyze, Motor opens the motor picker, and the warnings chip opens the issues list.
- While dragging, values that are changing briefly use `ink` at weight 600 so the eye can track them.
- With a selection, a second group appears before the warnings: "Selected: Fin set · 42.6 g".
