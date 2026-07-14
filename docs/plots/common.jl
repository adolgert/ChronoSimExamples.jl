# Shared setup for the figure-generation scripts.
#
# Every per-model script starts with `include("common.jl")`. This file loads
# CairoMakie with a consistent theme, defines where rendered figures go, and
# provides a `savefig` helper that writes into the model's asset directory under
# docs/src so Documenter serves the images.

using CairoMakie
using Colors
using Random

CairoMakie.activate!(; type = "png", px_per_unit = 2)

# A calm, colorblind-friendly palette reused across every figure.
const PALETTE = [
    colorant"#2f6db3",  # blue
    colorant"#d1642b",  # orange
    colorant"#3f9152",  # green
    colorant"#b5482e",  # rust
    colorant"#7a5ea8",  # purple
    colorant"#8a8f98",  # grey
]

const FIG_THEME = Theme(
    fontsize = 15,
    figure_padding = 12,
    palette = (color = PALETTE,),
    Axis = (
        xgridcolor = (:black, 0.06),
        ygridcolor = (:black, 0.06),
        xgridwidth = 0.8,
        ygridwidth = 0.8,
        topspinevisible = false,
        rightspinevisible = false,
        spinewidth = 0.8,
        titlesize = 17,
        titlealign = :left,
        titlefont = :bold,
    ),
    Legend = (framevisible = false, patchsize = (18, 12)),
)

set_theme!(FIG_THEME)

# docs/src/assets/figures relative to this file (docs/plots/common.jl).
const FIGROOT = normpath(joinpath(@__DIR__, "..", "src", "assets", "figures"))

"""
    savefig(fig, model, name; kw...)

Write `fig` to docs/src/assets/figures/<model>/<name>.png, creating the
directory if needed, and print the path. `name` is given without extension.
"""
function savefig(fig, model::AbstractString, name::AbstractString; kw...)
    dir = joinpath(FIGROOT, model)
    mkpath(dir)
    path = joinpath(dir, name * ".png")
    save(path, fig; kw...)
    println("wrote ", relpath(path, normpath(joinpath(@__DIR__, ".."))))
    return path
end
