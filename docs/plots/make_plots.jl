# Regenerate every figure used in the documentation.
#
# Usage, from the repository root:
#
#     julia --project=docs/plots docs/plots/make_plots.jl
#
# Each per-model script is self-contained (it includes common.jl and writes its
# own PNGs into docs/src/assets/figures/<model>/). Figures are committed to the
# repository, so this only needs to be re-run when a model or a figure changes.
# It is deliberately kept out of the unit tests, which must stay fast in CI.

const HERE = @__DIR__

for script in ["elevator.jl", "reliability.jl", "repairshop.jl",
               "landspread.jl", "sirvillage.jl"]
    path = joinpath(HERE, script)
    if isfile(path)
        println("\n=== ", script, " ===")
        include(path)
    else
        @warn "missing figure script" script
    end
end

println("\nAll figures regenerated.")
