# Bowshock parts-bin rail icons

The seven category icons from the Parts bin rail (left edge of the Build screen), top to bottom:

| File | Category |
|---|---|
| `bs-rail-airframe` | Airframe |
| `bs-rail-fins-guides` | Fins & guides |
| `bs-rail-internal` | Internal |
| `bs-rail-recovery` | Recovery |
| `bs-rail-mass` | Mass |
| `bs-rail-motors` | Motors |
| `bs-rail-presets` | Presets |

## Files
- `svg/`: masters. 24 × 24, 1.5 px stroke, square caps, `stroke="currentColor"`, no fills. Use these in the app.
- `png/dark/{24,48,96}/`: rasters in `ink` (#e6ebf1) for the dark theme, transparent background.
- `png/light/{24,48,96}/`: rasters in `ink` (#151a21) for the light theme.

## Colour by state (from the kit)
| State | Colour token |
|---|---|
| idle | `ink-faint` |
| hover | `ink` on `bg-400` |
| active | `ink` on `bg-200`, with a 2 px `accent` bar on the left edge of the button |

## iced
Embed the SVGs and tint them with the style colour rather than shipping PNGs:

```rust
use iced::widget::svg;

static FINS: &[u8] = include_bytes!("../assets/icons/bs-rail-fins-guides.svg");

fn rail_icon(bytes: &'static [u8], active: bool, t: &Tokens) -> iced::Element<'static, Msg> {
    svg(svg::Handle::from_memory(bytes))
        .width(24).height(24)
        .style(move |_, status| svg::Style {
            color: Some(match (active, status) {
                (true, _) => t.ink,
                (false, svg::Status::Hovered) => t.ink,
                _ => t.ink_faint,
            }),
        })
        .into()
}
```

`svg::Style.color` repaints every stroke, so one SVG covers both themes and all states.
