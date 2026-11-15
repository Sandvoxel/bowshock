# FAQ and troubleshooting

## The program hangs or crashes when it exits

On NVIDIA's Linux driver (seen on 615.71.09) a process that used the GPU can hang at exit (100 %
CPU in `libnvidia-gpucomp`) or abort with `free(): chunks in smallbin corrupted`, while the
driver's shader compiler thread is still running at `exit()`. It is intermittent (1 in 3 runs of one
benchmark) and a driver bug, not a bowshock or Luma bug (`bowshock-cfd/ISSUES.md`, row 1). It can
hit a program that calls `Analysis::run` on the GPU. The results are already printed by then. If it
bites you, update the driver, or run on `Backend::Cpu`.

## `Polar::backend` is `Cpu` but I have a GPU

Under `Backend::Auto` the reason is in `Polar::fallback`:

- No Vulkan driver, or a driver that fails to open a device: install your GPU vendor's Vulkan driver.
- A **software Vulkan device** (Mesa llvmpipe, lavapipe) is deliberately treated as no GPU, so the
  analysis runs on the CPU solver instead.
- The GPU run failed or the GPU solver panicked: the message says which; the CPU re-ran the
  sweep.

`Backend::Gpu` turns every one of these into an `Err`. Builds without the `gpu` feature only
have the CPU.

## How long does a run take?

A robustness matrix of inviscid `Coarse` sweeps (3 bodies, 10 Mach numbers, GPU) took about 20 s
in total (`bowshock-validate/README.md`). A viscous (RANS) point at `Medium` with fins takes 4 to
19 s on an RTX 3090 and 60 to 213 s on the CPU; a 23-point sweep from Mach 0.3 to 2.5 took 178 s on
the GPU (the table in [Performance](performance.md)). Each Mach number is a separate solve, so the cost is proportional
to the number of points. To iterate on a design: `Coarse`, a few Mach numbers, `max_iterations`
as a cap; confirm at `Medium`. `Analysis::run` has no progress callback; split a long sweep
into several `run` calls (one per Mach number) to show progress and to keep partial results.
Subsonic viscous points whose drag oscillates stop at the mean of the cycle after at least 14000
iterations (`StopReason::MeanStationary`, [Results](results.md#stopreason)).

## What does `Quality::Stalled` mean?

The solver reached its iteration cap without the residual dropping enough and without the
drag (or base pressure) becoming stationary. The number in `drag.total` is the last iterate, not a
converged value. Check `detail.stop_reason` (`MaxIterations`), `detail.residual_drop_orders` and
`Warning::UnsteadyWake` (an unsteady base wake does not converge to a steady state). Remedies:
raise `max_iterations` if you lowered it, try the neighbouring `Resolution`, or treat the point as
unreliable. A cap set with `max_iterations` for a smoke test produces `Stalled` points by design.
See [Results](results.md#quality).

## A Mach number near 1 gives a wiggly curve

Transonic drag changes quickly with Mach number, and with a sharp shoulder corner it also depends
on the grid ([`Warning::SharpCornerTransonic`](results.md#warnings)). Use a finer Mach spacing near
the drag rise (for example every 0.02 to 0.05), and compare two resolutions with
`estimate_uncertainty(true)`. Residuals at transonic Mach numbers stall 3 to 4 orders down; those
points end `Stationary` when the drag is steady.

## `InvalidInput` for my body

The message starts with the field that failed (`nose.length`, `aft[1].aft_diameter`,
`fins.count`, `motor.exit_diameter`). See [The body](body.md#validation-rules),
[Fins](fins.md#validation) and [Motor-on base flow](motor.md#motorspec) for the rules.

## Can I get lift, angle of attack or 3-D effects?

No: the analysis is axisymmetric and zero-lift. 3-D CFD with resolved fins and angle of attack is
planned.
