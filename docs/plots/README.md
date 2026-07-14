# Figure generation

This sub-project generates the plots and diagrams embedded in the documentation.
It is kept separate from the package and its test suite on purpose: it pulls in
heavy plotting dependencies (CairoMakie, GraphMakie) that would slow CI, and the
figures it produces are committed to the repository so the docs build without
running any of this code.

## Layout

- `Project.toml` — this sub-project's dependencies. `ChronoSimExamples` is taken
  from the repository root (`path = "../.."`); the unregistered ChronoSim-family
  packages track their public repos, matching `docs/Project.toml`.
- `common.jl` — shared CairoMakie theme, palette, and the `savefig` helper.
- `elevator.jl`, `reliability.jl`, `repairshop.jl`, `landspread.jl`,
  `sirvillage.jl` — one script per model. Each includes `common.jl`, runs the
  model (or draws a diagram), and writes PNGs into
  `docs/src/assets/figures/<model>/`.
- `make_plots.jl` — runs every per-model script.

## Regenerating

From the repository root:

```bash
julia --project=docs/plots -e 'using Pkg; Pkg.instantiate()'   # first time only
julia --project=docs/plots docs/plots/make_plots.jl            # all figures
```

To regenerate a single model's figures:

```bash
julia --project=docs/plots docs/plots/elevator.jl
```

Runs are seeded, so regenerating produces the same figures. After regenerating,
commit the changed PNGs under `docs/src/assets/figures/`.
