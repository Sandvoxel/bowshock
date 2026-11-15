# StagingScreen

The Build screen with a two-stage rocket, showing where configurations and staging sit. Nothing about the layout changes from **BuilderScreen**: no new panels or windows.

- **Status strip, first cell:** the **Config** cell with its menu open over the foot of the Parts bin. Each configuration lists its motors in firing order and its margin (see **ConfigPicker**).
- **Status strip, second cell:** the **Phase** switch, which appears only on staged rockets. Mass, length, CG, CP and stability to its right follow both the configuration and the phase.
- **Parts bin:** unchanged. There is no staging part; a second motor mount in line is what makes a second stage.
- **Viewport:** the separation is selected: a thin `accent` seam across the body with a filled diamond on the axis and a small station tag above. It sits at the joint behind the upper motor, and each stage is bracketed and named underneath.
- **Station Lanes:** the same separation line through every lane, stage names on the ruler, and one motor per stage in Propulsion.
- **Inspector:** the separation's sections: where (read-only), separation, stage 2 ignition, sequence (see **Staging**).
