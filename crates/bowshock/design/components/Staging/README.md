# Staging

How a rocket becomes multistage. You never mark where a rocket separates. Bowshock works it out from the **motor mounts**: put a second motor mount in line with the first and the rocket has two stages.

**How the split is found**
1. **One stage per in-line motor mount.** Mounts that share the same stretch of the rocket (a cluster) are one stage. Mounts one behind the other are separate stages, numbered in firing order, so the aft-most is Stage 1.
2. **By default the separation is the first joint between airframe parts at or aft of the upper motor's aft end.** The upper motor needs a clear path behind it once the lower stage is gone, so the rocket parts at the first joint behind its nozzle unless you choose another joint (see below).
3. Everything aft of that joint belongs to the lower stage, and everything forward to the upper.

**Choosing the joint.** The rule above is only the default. Real rockets separate at other joints: an interstage coupler that stays with the booster or with the sustainer, a joint further aft. You can choose any eligible joint: one at or aft of the upper stage's mount reach (the nearer of the upper motor's aft end and the upper mount tube's aft end, so the upper stage contains its own mount tube) and at or forward of the fore end of the lower stage's foremost mount tube (so the lower stage contains its own motor mount). A motor that hangs out past the chosen joint is not warned about. The choice is stored with the rocket (it is not a configuration setting); if the part is deleted, or the choice stops being eligible after a mount moves, the automatic joint is used again and the choice is kept for if it fits again.

On the rocket the separation is drawn quietly: a thin `accent` seam across the body at the joint with a small diamond marker on the axis (diamonds mean separation; squares are resize handles). The diamond is hollow at rest and filled when selected, and a small tag above gives the station. In Station Lanes it is a dashed line through every lane. Each stage is bracketed and named. While it is automatic the line follows the model: move the motor mount, change the motor's overhang, or move the joint, and the line moves.

**Moving it.** Drag the diamond (viewport) or the line (Station Lanes, also in the ruler) to another joint: it snaps joint to joint and skips joints that are not eligible. While dragging, the eligible joints show as hollow diamonds (a tick and diamond in the lanes) and the tag reads `SEPARATION · <station>`. Release commits one undo step (`Move separation`); Esc puts it back; a plain click only selects. The Inspector's select does the same without dragging.

**When it can't be found**
- **No joint between the two motors** (both mounts in one tube): a `warn` chip says so, the place where the split should be is shown as a dashed `warn` line, and one button fixes it: **Split tube at 870.0** cuts the tube at the upper motor's aft end.
- **A joint forward of the upper motor's aft end only** (the motor is recessed past the joint): the same warning, since the motor would fire inside the lower stage.
- A rocket with one motor mount has no separation and none of this appears.

**The separation's Inspector** opens when the diamond or the lane line is clicked.

| Section | Field | Stored with |
|---|---|---|
| **01 Where** | **Separates at** (select over the eligible joints, or Automatic), Found from (which motor's aft end, while automatic) | Rocket |
| **02 Separation** | **Separates on** (Booster burnout, Time from launch, Altitude) and Delay | Configuration |
| **03 Stage N ignition** | **Ignites on** (Separation, Booster burnout, Time from launch, Altitude) and Delay | Configuration |
| **04 Sequence** | Read-only event list with times, worked out from the current configuration's motors | Computed |

- **Separates at** is a select listing `Automatic (first joint behind the motor)` and then each eligible joint as `<station> · <fore part> | <aft part>`; a muted line under it says `Chosen` or `Automatic`. It belongs to the rocket, not to a configuration (no `PER CONFIG` badge). Found from shows only while the joint is automatic.
- Sections 02 and 03 carry the `PER CONFIG` badge: the right delay depends on which motors are in, so timing belongs to the configuration (see **ConfigPicker**).
- **Sequence** lists launch, burnout, separation, ignition and upper-stage burnout, one row each, with a square in the colour of what it concerns (`sys-propulsion` for motors, `ink` for separation, `accent` for the event being edited). Changing a delay or swapping a motor updates every later time.
- A coast longer than 3 s before ignition adds a `warn` chip under the sequence: *Long coast: check speed and tilt at ignition*.
- Parachute deployment uses the same pattern (an event, a trigger, a delay), so timing is set one way across the app.
- A tube coupler sitting at the joint stays with the stage of the tube it is attached to.

**Phase**
- On a staged rocket the status strip gains a **Phase** switch beside the Config cell: **Full stack**, then one entry per upper stage.
- It chooses what is flying. The viewport and Station Lanes fade the stages that have dropped away, and mass, CG, CP and margin are recomputed for the rest. Stability has to hold in every phase.
- Phase is a view setting. It changes no part and is not saved in a configuration.

**More than two stages** work the same way: each further in-line motor mount adds a stage, a separation, an ignition section and a Phase entry.

**Clusters** (several motors firing together in one stage) are not staging. They are radial symmetry on a motor mount (see **Symmetry**).

**iced:** the stages are never stored, only the user's choice of joint is (`Rocket::separation_joints`, keyed by the upper stage's foremost motor mount so it survives other stages and parts coming and going; it stores the first airframe part of the lower stage). `fn stages(&Rocket) -> Vec<Stage>` sorts in-line motor mounts by station and, for each neighbouring pair, takes the chosen joint if it is still eligible, else the first structural joint with `station >= upper_motor.aft`. `None` produces the warning and the suggested split station. `Rocket::separation_joints(stage)` lists the eligible joints; `Rocket::set_separation_joint(stage, Option<PartId>)` sets or clears the choice (the app does it as one undo step, `Move separation`). Everything that depends on the separation (stage spans and masses, the flight's vehicle per phase, Station Lanes, the CFD plans) reads `stages()` and follows. `Config.events: Vec<Event { kind, trigger, delay }>` is keyed by stage index; the sequence is a small fixed-point pass over triggers.
