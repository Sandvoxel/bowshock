# Button

Push buttons in four weights: primary for the one commit action per view, default for secondary commands, ghost for toolbar and in-panel actions, and icon-only for dense toolbars.

- Height is `control-h` (26 px) with square corners (`radius-0`). Labels are 13 px Plex Sans, weight 500. Shortcuts can sit inside as a framed mono key (`F5`).
- **Primary** (`accent` fill, `accent-ink` text): one per screen region at most, such as *Simulate* in the top bar or *Apply* in a dialog.
- **Default** (`bg-300` + `line-200` border) for secondary commands.
- **Ghost**: no border until hover. Use it for undo/redo, overflow menus and inspector row actions.
- **Icon-only** buttons are square and always need a tooltip with the shortcut ("Undo  Ctrl+Z").
- Focus is a 1 px `accent` outline offset by 2 px, shown for keyboard focus only.
- Disabled buttons drop the fill, switch to a dashed border and use `ink-faint`. Never hide a disabled command; show the reason in its tooltip.

**iced:** `button(...).style(bs::button::primary)`. Implement each weight as a `fn(&Theme, button::Status) -> button::Style` that reads the token struct.
