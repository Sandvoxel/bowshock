# Performance

## GPU or CPU

The GPU solver is written in plain Luma array ops and runs the same scheme as the f64 CPU solver
(f32 or double-float arithmetic on the GPU). Measured per iteration on an RTX 3090 against the
f64 CPU solver on 32 rayon threads (`bowshock-cfd/README.md`, "Speed"; microseconds per iteration,
15 deg cone at Mach 2, 2000 iterations from the free stream, best of 3):

| grid | CPU f64 | GPU f32 | GPU double-float |
| --- | --- | --- | --- |
| 100x50 | 602 | 33.2 | 190.6 |
| 200x100 | 1304 | 44.7 | 260.2 |

Viscous implicit step on the G3 body (`bowshock-cfd/README.md`, "Viscous terms and the SST
model"; SST, Mach 0.5, flat base and wake, microseconds per iteration, replay):

| level | grid | CPU f64 | GPU f32 | GPU double-float |
| --- | --- | --- | --- | --- |
| 0 | 156x122 | 6724 | 319 | 447 |
| 1 | 312x122 | 12454 | 487 | 645 |

That is 21 to 26 times faster per iteration than the CPU in f32 (the CPU column moves by a factor
of 2 between runs on a shared machine). Over the Fox TM X-2439 sweep (16 inviscid runs, level 1) the
whole sweep took 1555.7 s on the CPU and 60.13 s on the GPU, 25.9 times faster, and 12.2 to 29.2
times per run (`bowshock-cfd/README.md`, "Timing"). Double-float costs 1.2 to 1.4 times the f32 time
in the viscous step.

The first call of a sweep also compiles and traces the GPU kernels: about 1 s for the default
inviscid staging on the cone case, and longer for viscous cases (`bowshock-cfd/README.md`, "Speed",
"First call").

## Resolution

Each `Resolution` level roughly doubles the grid along the body (156x122 at level 0, 312x122 at
level 1 on the G3 body), so the per-iteration cost grows by 1.5 times (GPU f32) to 1.9 times (CPU)
per level on that body, and finer grids also need more iterations to converge. Use `Coarse`
for exploring, `Medium` for results and `Fine` for confirmation; see
`Analysis::estimate_uncertainty` for a grid-sensitivity number.

The sweep is sequential over Mach numbers (cold starts by default, see
[warm start](sweeps.md#warm-start)): cost is proportional to the number of points.

## Timed realistic sweep

`cargo run --release -p bowshock --example sweep_timing`: the body of the examples (tangent-ogive nose
0.30 x 0.10 m, 0.60 m tube, 0.10 m boattail to 0.08 m) with four rounded fins, viscous at ISA 0 m,
`Resolution::Medium`, `Backend::Auto` (here: GPU, RTX 3090, double-float), every Mach number run on
its own (cold start, including the kernel build of each run):

| M | seconds | quality | CD total |
| --- | --- | --- | --- |
| 0.3 | 14.4 | Stationary | 0.20570 |
| 0.4 | 17.7 | Stationary | 0.20129 |
| 0.5 | 18.9 | Stationary | 0.19953 |
| 0.6 | 15.5 | Stationary | 0.20175 |
| 0.7 | 6.5 | Stationary | 0.20877 |
| 0.8 | 4.0 | Stationary | 0.22362 |
| 0.9 | 4.9 | Stationary | 0.26712 |
| 1.0 | 7.7 | Stationary | 0.38851 |
| 1.1 | 4.3 | Stationary | 0.43594 |
| 1.2 | 4.3 | Stationary | 0.43921 |
| 1.3 | 4.3 | Stationary | 0.43607 |
| 1.4 | 6.4 | Stationary | 0.43138 |
| 1.5 | 4.5 | Stationary | 0.42572 |
| 1.6 | 4.4 | Stationary | 0.42006 |
| 1.7 | 4.4 | Stationary | 0.41451 |
| 1.8 | 4.4 | Stationary | 0.40898 |
| 1.9 | 4.4 | Stationary | 0.40324 |
| 2.0 | 3.9 | Stationary | 0.39705 |
| 2.1 | 6.1 | Stationary | 0.39067 |
| 2.2 | 4.1 | Stationary | 0.38351 |
| 2.3 | 4.1 | Stationary | 0.37714 |
| 2.4 | 14.4 | Stationary | 0.37121 |
| 2.5 | 14.4 | Stationary | 0.36535 |

Total 177.9 s for 23 points (7.7 s per point on average). Points that stop by `CdStationary` take
about 4000 iterations (4-8 s); points whose drag settles into a limit cycle (subsonic, and here
M 2.4-2.5) stop by `MeanStationary` after at least 14000 iterations (14-19 s). Before the
mean-stationary rule the subsonic points ran to the 160000-iteration cap: 142-154 s each.

The same set-up on the CPU (`-- --cpu`, `Backend::Cpu`, 32 threads):

| M | CPU seconds | GPU seconds | CPU / GPU | CD CPU | CD GPU |
| --- | --- | --- | --- | --- | --- |
| 0.5 | 213.4 | 18.9 | 11.3 | 0.19953 | 0.19953 |
| 1.0 | 116.3 | 7.7 | 15.1 | 0.38851 | 0.38851 |
| 2.0 | 60.4 | 3.9 | 15.5 | 0.39706 | 0.39705 |

The GPU runner applies the CPU stop rules to the same residual and drag history, and the totals agree
within 1e-5.
