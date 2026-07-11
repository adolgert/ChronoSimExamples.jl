# ChronoSimExamples

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://adolgert.github.io/ChronoSimExamples.jl/stable/)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://adolgert.github.io/ChronoSimExamples.jl/dev/)
[![Build Status](https://github.com/adolgert/ChronoSimExamples.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/adolgert/ChronoSimExamples.jl/actions/workflows/CI.yml?query=branch%3Amain)

This repository has examples of simulations written with
[ChronoSim.jl](https://github.com/adolgert/ChronoSim.jl), which samples its
continuous-time events with
[CompetingClocks.jl](https://github.com/adolgert/CompetingClocks.jl).

 * [Bank of multiple elevators](src/elevator/elevator.jl)
 * [Reliability of a set of trucks](src/reliability/reliability.jl)
 * [Spread across a landscape](src/landspread/landspread.jl)
 * [Machine repair shop with a differentiable likelihood](src/repairshop/repairshop.jl)
 * [Village-scale epidemic](src/sirvillage/sirvillage.jl)

Several models come in pairs: a hand-written module and a "derived twin" whose
event triggers are generated from the event preconditions by ChronoSim's
derivation machinery. The test suite asserts the two members of each pair
produce identical trajectories from the same seed.

## What the examples demonstrate

The examples track the `worldtimer-adoption` line of ChronoSim and
CompetingClocks 0.4, and exercise its new capabilities:

* **The θ (parameter) seam.** An event's `enable` can take the parameter
  vector explicitly — `enable(event, physical, θ, when)` — so a recorded
  trajectory can be re-evaluated at any θ, including ForwardDiff dual numbers,
  through `trace_likelihood(sim, init, trace; params=θ)`. The
  [landspread](src/landspread/landspread.jl) model shows the recommended
  four-argument style; the [reliability](src/reliability/reliability.jl) model
  deliberately keeps the old three-argument signature to demonstrate that the
  backward-compatible fallback works.
* **Records and the effect check.** The
  [repair shop](src/repairshop/repairshop.jl) example runs once under the
  `RecordMinimal` policy to capture a `MinimalRecord`, verifies the record with
  `effect_check` (replay must equal the forward log-likelihood exactly, `==`
  not `≈`), and then differentiates the trace log-likelihood in θ with
  ForwardDiff — the full record → check → differentiate workflow, tested
  against a hand-derived analytic score in
  [test/test_repairshop.jl](test/test_repairshop.jl).
* **The gradient-estimator family.** The companion
  [repair-shop gradients](src/repairshop/repairshop_gradients.jl) example runs
  ClockGradients' whole family on the same law against one closed-form oracle:
  the score/IPA pairing on a pure model (the pairing flags that the pathwise
  estimate of a terminal count is identically zero), the branching estimator
  driven through the live ChronoSim simulation, and smoothed perturbation
  analysis (SPA) with the hand-written **pure model twin** the estimator
  audits at every step. On this model SPA's criticality gate proves every
  event-order swap harmless — independent machines commute — so it spawns no
  clones and the whole derivative flows through its horizon boundary term.
  Tested in
  [test/test_repairshop_gradients.jl](test/test_repairshop_gradients.jl).
* **Master-seed, per-clock randomness.** Every random stream derives from one
  seed, keyed by clock identity, so an event's draws do not depend on how other
  events interleave. The elevator model documents the one sharp edge: a clock
  key component needs a content-based `Base.hash` (see the `ElevatorDirection`
  enum in [src/elevator/elevator.jl](src/elevator/elevator.jl)).
* **Trigger derivation, over-approximation, and read verification.** The
  hand-vs-derived twin tests run under ChronoSim's coverage oracle, and the
  over-approximation tests measure the proposal cost of derived generators.

For per-event coupling and memory declarations (`reevaluation_coupling`,
`memory_policy`), simulation cloning, and forced firing, see the ChronoSim
documentation; for keyed streams and sampler-level coupling guarantees, see the
CompetingClocks documentation.

## Migrating an existing model

If you have a model written against ChronoSim with CompetingClocks 0.3, see
[MIGRATION.md](MIGRATION.md). Most models need no code change; the document
orders the cases by likelihood and includes before/after snippets from this
repository's own migration.
