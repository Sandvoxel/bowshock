# WarningChip

A compact, outlined status label: always icon + words + (where relevant) the number that's wrong and the limit it broke.

- 22 px tall, square, 1 px border in the state colour, text in the same colour, transparent fill so it sits on any surface.
- States: `ok`, `warn`, `err`, and neutral `info` (conditions, context).
- Messages follow "*what* · *by how much* (*limit*)": "Unstable · 0.31 cal (min 1.0)".
- Clicking a warning selects the offending part and scrolls the inspector to the field.
