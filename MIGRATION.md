# Migrating models to ChronoSim (worldtimer-adoption) and CompetingClocks 0.4

This package's five example models were migrated to the `worldtimer-adoption`
branches of [ChronoSim.jl](https://github.com/adolgert/ChronoSim.jl) and
[CompetingClocks.jl](https://github.com/adolgert/CompetingClocks.jl)
(CompetingClocks version 0.4.0, a breaking release). This document lists what a
model author has to change, ordered from most likely to least likely. Every
before/after snippet is drawn from an actual fix made in this repository.

## Summary of what changed upstream

CompetingClocks 0.4.0 moved ownership of randomness into the sampler: the
sampler verbs (`enable!`, `next`, `fire!`, ...) no longer take a random number
generator argument. Instead, each clock key owns a dedicated random stream,
seeded deterministically from one master seed via `hash((seed, clock_key))`.
`clone` now means "full-state coupled copy" (the old empty-copy behavior is
`similar_sampler`), and the `CommonRandom` common-random-numbers subsystem was
removed because keyed streams subsume it. ChronoSim gained the θ (parameter)
seam — a four-argument `enable(event, physical, θ, when)` — plus per-event
re-evaluation and memory declarations, a `MinimalRecord` trajectory record with
an exact-replay `effect_check`, `clone`/`force_fire!` for branching estimators,
and opt-in read verification. ChronoSim's own randomness is likewise derived
from a single master seed.

## 1. Most models: change nothing but a compat bound

If your model talks only to ChronoSim's documented surface — `SimulationFSM`,
`precondition`/`enable`/`fire!`, `@conditionsfor`, `ChronoSim.run`,
`trace_likelihood` — it compiles and runs unchanged. The verb-level breaking
change in CompetingClocks 0.4 is absorbed entirely below ChronoSim's
`SamplingContext`. All five models in this package (elevator, reliability,
sirvillage, landspread, and their derived twins) needed **zero** signature
changes. The only mandatory edit was the compat bound:

```toml
# Project.toml, before
CompetingClocks = "0.3"

# after
CompetingClocks = "0.4"
```

The old three-argument `enable(event, physical, when)` keeps working: the
engine calls the four-argument form, and its default method forwards to yours.
`src/reliability/reliability.jl` deliberately keeps the three-argument
signature as standing evidence of that fallback.

## 2. Seeded trajectories differ; re-pin golden outputs

A `SimulationFSM` built with `rng=Xoshiro(seed)` or `seed=` now derives one
master seed and seeds every per-clock stream from it, so **the same seed
produces a different (equally valid) trajectory than it did before**. Any test
that pins concrete event times or event orders from a seeded run needs its
literals re-captured once, with a comment saying why. Statistical assertions
and structural identities are stream-layout independent and should pass
unchanged — in this package, no re-pin was needed at all, because the tests
assert identities (hand-written and derived twins produce the same trajectory,
derived generators propose a superset, admissions are equal) rather than
literal times.

The compensation for re-pinning is a much stronger reproducibility property:
an event's draws now belong to that event's clock key, so two same-seed runs
give each event identical randomness even when other events interleave
differently.

## 3. Clock keys must hash content-stably (the enum trap)

This was the one real breakage in this package, and it broke three test files
at once. CompetingClocks 0.4 seeds each clock's stream from
`hash((seed, clock_key))`. A clock key component with no content-based
`Base.hash` method falls back to `objectid` hashing, which is neither stable
across Julia processes with different compile options (a plain
`julia --project` run and a `Pkg.test` run disagreed) nor equal across two
modules that define the "same" `@enum`. The elevator's `DispatchElevator`
event carries a direction enum in its clock key, so its firing times were not
reproducible, and the hand-written and derived twin modules — whose
trajectories must be identical — silently diverged.

The fix is one line per enum, in the module that owns the enum (this is a
method on your own type, not type piracy):

```julia
# before: clock streams seeded from objectid-based hashes — unstable
@enum ElevatorDirection Up Down Stationary

# after: content-based hash, stable across processes and modules
@enum ElevatorDirection Up Down Stationary
Base.hash(x::ElevatorDirection, h::UInt) = hash(Symbol(x), h)
```

Rule of thumb: every component of every clock key should be a `Symbol`, an
integer, a string, or a type that defines a content-based `Base.hash`. Audit
your event structs' fields, because `clock_key(event)` is built from them.

## 4. `CommonRandom` is gone; keyed streams are the successor

This package never used `CommonRandom`, so nothing here needed rewriting, but
for models that did: the coupling it provided is now structural. Two samplers
built from the same seed give the same clock key the same draws, by
construction, no recording or replay machinery required:

```julia
# Coupled pair for a finite-difference: same seed, different θ.
sim_lo = SimulationFSM(build_state(), events; seed=42, params=[θ])
sim_hi = SimulationFSM(build_state(), events; seed=42, params=[θ + h])
# Clocks shared by the two runs consume identical per-(key, occurrence) draws.
```

To make a cloned simulation diverge on purpose, rekey its streams
(`CompetingClocks.rekey_streams!`); divergence is now an explicit act rather
than a side effect of passing a different generator. See the CompetingClocks
documentation on keyed streams for the coupling guarantees, and ChronoSim's
`clone(sim)`/`force_fire!` for branching workflows built on top of them.

## 5. Hand-rolled `@obswrite` state: a fixed bug worth knowing about

This is a pre-existing bug that the migration audit surfaced, relevant only to
models that implement `ObservedPhysical` by hand with `@obsread`/`@obswrite`
(models using `@observedphysical` containers are unaffected). The assignment
form of `@obswrite` recorded the written address but **discarded the
right-hand side**, so the write silently never happened. The landspread
example ran zero events because of this. The macro is **fixed on ChronoSim's
`worldtimer-adoption` branch** (the assignment now executes, with a
regression test asserting the value actually changes), so the assignment form
is safe against that branch and later releases. The separated form used by
this package's landspread example remains equally valid:

```julia
# before: records the address, silently drops the assignment
@fire function fire!(evt::Spread, land, when, rng)
    @obswrite land.mark[evt.destination] = 1
end

# after: assign, then record
@fire function fire!(evt::Spread, land, when, rng)
    land.mark[evt.destination] = 1
    @obswrite land.mark[evt.destination]
end
```

If your model has no functional test that asserts events actually fire, add
one; that absence is what let this example stay silently dead
(`test/test_landspread.jl` now pins it).

## 6. New capabilities worth adopting (optional)

None of these are required, but each is demonstrated working in this package:

* **The θ seam.** Read parameters from an explicit vector in a four-argument
  `enable`, and evaluate a recorded trace at any θ — including ForwardDiff
  dual numbers — via `trace_likelihood(sim, init, trace; params=θ)`. See
  `src/landspread/landspread.jl` for the model-side change:

  ```julia
  # before: parameters hard-coded in the model
  function enable(evt::Spread, land, when)
      scale = 0.1 * land.distance[evt.source, evt.destination]^1.2
      return (Weibull(2, scale), when)
  end

  # after: parameters read from θ at the seam
  function enable(evt::Spread, land, θ, when)
      scale = θ[1] * land.distance[evt.source, evt.destination]^θ[2]
      return (Weibull(2, scale), when)
  end
  ```

* **Records and the effect check.** Run with `policy=RecordMinimal(...)` to
  capture a `MinimalRecord`, then `effect_check` replays it and demands exact
  floating-point equality with the forward log-likelihood. See
  `src/repairshop/repairshop.jl` for the full record → check → differentiate
  workflow, including horizon-censored evaluation (`censor=true`).

* **Per-event declarations.** `reevaluation_coupling(::Type{MyEvent}) = :carry`
  declares the IPA-safe coupling for a still-enabled clock whose distribution
  is re-evaluated (`:redraw` is the default and matches prior behavior for
  models whose `reenable` never changes the schedule);
  `memory_policy(::Type{MyEvent}) = :resume` banks a clock's age across
  disable/enable cycles. See ChronoSim's documentation for both.

* **Cloning and forced firing.** `clone(sim)` produces a coupled, independent
  copy of a running simulation; `force_fire!(sim, key, t)` imposes a chosen
  transition at a chosen time through the same update path as a natural
  firing. These are the primitives for weak-derivative branching estimators.

* **Read verification.** `ChronoSim.with_read_verification(f)` (or
  `check_derivation_coverage`) audits that derived triggers cover every place
  a precondition reads; this package's differential tests run under it.
