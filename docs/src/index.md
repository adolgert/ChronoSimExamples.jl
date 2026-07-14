```@meta
CurrentModule = ChronoSimExamples
```

# ChronoSimExamples

Documentation for [ChronoSimExamples](https://github.com/adolgert/ChronoSimExamples.jl).

The repository README describes what each example demonstrates, including the
θ (parameter) seam, trajectory records with the exact-replay effect check, and
the repair-shop differentiation workflow. Users migrating a model from
CompetingClocks 0.3 to 0.4 should read
[MIGRATION.md](https://github.com/adolgert/ChronoSimExamples.jl/blob/main/MIGRATION.md)
at the repository root, which orders the required changes by likelihood with
before/after snippets.

## The models

Each example has its own section: a first page presenting the model — where it
comes from, what it represents, and the state and events it defines — and one or
more further pages on how to work with it, drawn from the model's tests.

- **[Elevator](models/elevator/model.md)** — a continuous-time port of a TLA+
  multi-car elevator spec, and the main fixture for ChronoSim's debugging tools.
- **[Reliability](models/reliability/model.md)** — a fleet of machines that age
  and fail; the suite's evidence that the pre-θ-seam three-argument `enable`
  still works.
- **[Repair Shop](models/repairshop/model.md)** — machines breaking and being
  repaired, used to demonstrate record/replay, exact effect checks, and
  differentiable trajectory likelihoods with the ClockGradients estimator family.
- **[Landspread](models/landspread/model.md)** — a contagion spreading across a
  landscape; the canonical demonstration of the θ (parameter) seam.
- **[SIR Village](models/sirvillage/model.md)** — an individual-based SIRS
  epidemic with movement and an evolving pathogen.

Several models come in pairs — a hand-written module and a "derived twin" whose
event triggers ChronoSim generates from the preconditions — and the test suite
asserts the two produce identical trajectories from the same seed.

```@index
```

```@autodocs
Modules = [ChronoSimExamples]
```
