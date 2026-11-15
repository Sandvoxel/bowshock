# Logo

The Bowshock mark is a **detached bow shock standing ahead of a Von Kármán nose at Mach 2.4**, the thing the product is named after, drawn the way it actually looks.

**Construction** (22-unit grid):
- **Nose**: a true Von Kármán (Haack series, C = 0) profile, `y = R·√((θ − sin 2θ / 2) / π)` with `θ = acos(1 − 2x/L)`, 14 u long and 8 u in diameter, always `ink`. The tip is very steep (the profile goes as x^¾), so it reads as slightly blunt, which is what lets a shock stand off ahead of it.
- **Shock**: a hyperbola, normal to the flow on the axis and straightening into arms that tend toward the Mach angle μ = asin(1/2.4) = 24.6°. 2 u stroke with square caps, always `accent`. The standoff is 3.6 u ahead of the nose tip.

**Lockup**: the mark followed by **BOWSHOCK** in IBM Plex Sans 600, uppercase, +0.02em tracking. Gap = 0.3 × mark height.

**Sizes**: the master works from 20 px up (use a 2.4 u stroke at 20 px). **16 px and below** uses the optical cut: heavier 3 u stroke, shorter arms, a fatter nose.

**App icon**: the mark on `bg-100` with a 1 px `line-200` frame (default), or reversed in `accent-ink` on a solid `accent` square. Square-cornered; let the OS round it.

**Cover / hero**: the mark drawn large on the viewport ground (grid, dashed axis, registration marks) with a longer nose and two faint outer standoff lines, labelled `VON KÁRMÁN NOSE · DETACHED SHOCK · M 2.4`. There are no colour blocks.

**Don't**: recolour shock and nose independently, add gradients or glow, rotate or mirror it (the nose always points left into the oncoming flow), replace the hyperbola with a circular arc, or put the mark inside a circle or rounded tile.
