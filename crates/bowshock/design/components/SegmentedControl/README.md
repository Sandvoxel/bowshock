# SegmentedControl

A row of mutually exclusive options. Use it when there are 2–6 choices that all fit at once and switching between them is frequent.

- A square strip: `bg-100` inside a 1 px `line-200` frame, with segments split by `line-100` rules. The active segment is `bg-300` with a **2 px `accent` underline**. No pill and no floating thumb.
- Used for: 3D / 2D, inside view (Solid / Cutaway / X-ray / Exploded), the Parts bin symmetry count, angle snap, fin cross-section, the top-bar Build / Simulate / Analyze switcher, and palette category tabs.
- Numeric options (fin count) use the mono face.
- More than 6 options → use a select.
- Keyboard: arrow keys move the selection when focused.

**iced:** a `row!` of `button`s with a shared style fn taking `is_active`. Wrap it as `segmented(options, selected, on_select)`.
