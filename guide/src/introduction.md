# Introduction

bowshock computes the zero-lift drag of an axisymmetric rocket body as a function of Mach number,
from Mach 0.1 to 3. You describe the body in metres, optionally add fins, flight conditions and a
motor, and get one drag coefficient per Mach number, split into

- **forebody pressure drag**: nose and aft sections, from an axisymmetric Euler or RANS solution,
- **skin friction**: from the RANS solution (viscous analyses only),
- **base drag**: from the solver's base wake (viscous analyses), or from an empirical correlation,
- **fin drag**: friction plus leading/trailing-edge and wave pressure drag, from an empirical model.

The flow solver is a finite-volume code on structured grids. It runs on the GPU through
[Luma](https://github.com/flipnoteman/Luma) (wgpu, Vulkan) and falls back to the CPU when no GPU is
available. All of it is reached through a small Rust API: [`BodySpec`](body.md) describes the body,
[`Analysis`](sweeps.md) is a builder that you configure and run, and [`Polar`](results.md) is the
result. There is no command-line tool and no file format; the guide is written for Rust
programmers who call the crate from their own code. The
[API reference](api/bowshock_core/index.html) lists every item.

## Limits

Read these before trusting a number:

- **Axisymmetric bodies only.** The body is a nose followed by tubes and conical transitions,
  ending in a flat base.
- **Zero lift.** The angle of attack is always 0; `PolarPoint::alpha` is 0.0.
- **Fully turbulent, smooth walls.** Viscous analyses solve the Reynolds-averaged Navier-Stokes
  equations with the k-omega SST model; there is no transition model and no wall roughness.
- **Mach 0.1 to 3.** Other values are an `InvalidInput` error.
- **Fins are an empirical model.** Fin drag follows the OpenRocket technical documentation
  (Niskanen, 2013). Fin-body interference drag is neglected, and the fins do not enter the flow
  solution. On the Arcas wind-tunnel model the fin drag increment is under-predicted by 16-65 %
  (Mach 0.6-3.0); the fins-on total is 16-25 % low at Mach 0.9-1.8 (see
  [Validation](validation.md#arcas-sounding-rocket-d1)).
- **The motor jet is a cold-air model, and motor-on base pressure is not validated.** The jet is
  air with gamma = 1.4 leaving the nozzle exit at a uniform supersonic state; there is no real-gas
  exhaust and no nozzle lip angle. Against strongly under-expanded jets (NASA TP-2005) the base
  pressure is 0.2-1.0 p_inf too low and does not rise with the jet (see
  [Validation](validation.md#motor-on-base-pressure-nasa-tp-2005-d2)). Treat power-on base drag
  as qualitative.
- **Inviscid base drag is a correlation.** The Euler equations give no physical base pressure,
  so Euler analyses add a Mach-only free-flight correlation.

3-D CFD with resolved fins and angle of attack is planned.

## Where to go

| If you want to | Read |
| --- | --- |
| add the crate and run a first analysis | [Getting started](getting-started.md) |
| describe your rocket | [The body](body.md), [Fins](fins.md) |
| set altitude or wind-tunnel conditions | [Flight conditions](flight.md) |
| choose resolution, backend, Euler or RANS | [Mach sweeps](sweeps.md) |
| compute drag with the motor running | [Motor-on base flow](motor.md) |
| interpret a `Polar` | [Results](results.md) |
| see how far to trust the numbers | [Validation](validation.md) |
| know how long a sweep takes | [Performance](performance.md) |
