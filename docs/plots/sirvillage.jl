include(joinpath(@__DIR__, "common.jl"))
using ChronoSimExamples
using ChronoSim
using Random
using Logging
using Graphs
using GraphMakie
using NetworkLayout

const SV = ChronoSimExamples.SIRVillage

# ---------------------------------------------------------------------------
# Observer that captures S/I/R counts and the strain count after each firing.
# Called as (physical, when, event, changed_places).
# ---------------------------------------------------------------------------
mutable struct SIRObserver
    when::Vector{Float64}
    S::Vector{Int}
    I::Vector{Int}
    R::Vector{Int}
    strains::Vector{Int}
    SIRObserver() = new(Float64[], Int[], Int[], Int[], Int[])
end

function (obs::SIRObserver)(physical, when, event, changed_places)
    s = i = r = 0
    for a in physical.actors
        if a.state == SV.Susceptible
            s += 1
        elseif a.state == SV.Infectious
            i += 1
        else
            r += 1
        end
    end
    push!(obs.when, when)
    push!(obs.S, s)
    push!(obs.I, i)
    push!(obs.R, r)
    push!(obs.strains, length(physical.strains))
    return nothing
end

# ---------------------------------------------------------------------------
# Build and run a 30-person, 15-day village, matching test/test_sirvillage.jl.
# ---------------------------------------------------------------------------
function run_village(; person_cnt = 30, location_cnt = 10, days = 15.0,
                       seed = 2938423)
    day_length = 1.0
    rng = Xoshiro(seed)
    physical = SV.Village(person_cnt, location_cnt, day_length, rng)
    included = [
        SV.InitEvent, SV.Travel, SV.Infect, SV.Recover, SV.Reset, SV.Mutate,
    ]
    obs = SIRObserver()
    sim = SimulationFSM(physical, included; rng = rng, observer = obs,
                        policy = ChronoSim.NoPolicy())
    stop_condition = (p, step_idx, event, when) -> when > days
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, SV.InitEvent(), stop_condition)
    end
    return obs, physical, sim.when
end

# ===========================================================================
# Figure 1: SIRS compartment diagram with the driving events on the arrows.
# ===========================================================================
function figure_compartments()
    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1])
    hidedecorations!(ax)
    hidespines!(ax)
    ax.aspect = DataAspect()
    xlims!(ax, 0, 20)
    ylims!(ax, 0, 11)

    boxw, boxh = 3.8, 1.6

    function node!(cx, cy, label, color)
        poly!(ax, Point2f[(cx - boxw/2, cy - boxh/2), (cx + boxw/2, cy - boxh/2),
                          (cx + boxw/2, cy + boxh/2), (cx - boxw/2, cy + boxh/2)];
              color = (color, 0.16), strokecolor = color, strokewidth = 1.8)
        text!(ax, cx, cy; text = label, align = (:center, :center),
              fontsize = 14, color = :black, font = :bold)
        return (cx, cy)
    end

    function edge!(p, q; color = PALETTE[6], label = nothing, laboff = (0.0, 0.0),
                   padx = boxw/2 + 0.2, pady = boxh/2 + 0.2)
        dx, dy = q[1] - p[1], q[2] - p[2]
        L = hypot(dx, dy)
        ux, uy = dx / L, dy / L
        sp = (abs(ux) > abs(uy)) ? padx / max(abs(ux), 1e-3) : pady / max(abs(uy), 1e-3)
        x0, y0 = p[1] + ux*sp, p[2] + uy*sp
        x1, y1 = q[1] - ux*sp, q[2] - uy*sp
        arrows!(ax, [x0], [y0], [x1 - x0], [y1 - y0];
                color = color, linewidth = 2.0, arrowsize = 13)
        if label !== nothing
            mx, my = (x0 + x1)/2 + laboff[1], (y0 + y1)/2 + laboff[2]
            text!(ax, mx, my; text = label, align = (:center, :center),
                  fontsize = 12.5, color = color, font = :bold)
        end
    end

    scol = PALETTE[1]  # susceptible
    icol = PALETTE[2]  # infectious
    rcol = PALETTE[3]  # recovered
    mcol = PALETTE[5]  # mutation

    text!(ax, 10, 10.3; text = "SIRS cycle", align = (:center, :center),
          fontsize = 15, color = :black, font = :bold)

    S = node!(3.2, 7.5, "Susceptible", scol)
    I = node!(10.0, 7.5, "Infectious", icol)
    R = node!(16.8, 7.5, "Recovered", rcol)

    edge!(S, I; color = icol, label = "Infect", laboff = (0, 0.55))
    edge!(I, R; color = rcol, label = "Recover", laboff = (0, 0.55))

    # Reset arc: Recovered back to Susceptible along the bottom.
    Rb = (16.8, 6.7)
    Sb = (3.2, 6.7)
    arrows!(ax, [Rb[1]], [Rb[2] - 2.4], [Sb[1] - Rb[1]], [0.0];
            color = scol, linewidth = 2.0, arrowsize = 13)
    lines!(ax, [Rb[1], Rb[1]], [Rb[2], Rb[2] - 2.4]; color = scol, linewidth = 2.0)
    lines!(ax, [Sb[1], Sb[1]], [Sb[2] - 2.4, Sb[2]]; color = scol, linewidth = 2.0)
    text!(ax, 10, 4.3 - 0.5; text = "Reset (waning immunity)",
          align = (:center, :center), fontsize = 12.5, color = scol, font = :bold)

    # Mutate self-loop within Infectious: a new strain is born.
    strain = node!(10.0, 2.4, "new Strain\n(child of parent)", mcol)
    edge!(I, strain; color = mcol, label = "Mutate", laboff = (1.7, 0.0),
          pady = boxh/2 + 0.2)

    savefig(fig, "sirvillage", "compartments")
    return fig
end

# ===========================================================================
# Figure 2: epidemic curve (S, I, R over time) plus cumulative strain count.
# ===========================================================================
function figure_epidemic(obs)
    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1];
        title = "Epidemic over a 30-person, 15-day village run",
        xlabel = "time (days)", ylabel = "individuals")

    t = obs.when
    stairs!(ax, t, obs.S; step = :post, color = PALETTE[1], linewidth = 2.2,
            label = "Susceptible")
    stairs!(ax, t, obs.I; step = :post, color = PALETTE[2], linewidth = 2.2,
            label = "Infectious")
    stairs!(ax, t, obs.R; step = :post, color = PALETTE[3], linewidth = 2.2,
            label = "Recovered")
    axislegend(ax; position = :rc, orientation = :vertical)
    xlims!(ax, 0, maximum(t))

    # Second panel: cumulative distinct strains over time.
    ax2 = Axis(fig[2, 1];
        xlabel = "time (days)", ylabel = "strains",
        height = 110)
    stairs!(ax2, t, obs.strains; step = :post, color = PALETTE[5], linewidth = 2.2)
    xlims!(ax2, 0, maximum(t))
    ylims!(ax2, 0, maximum(obs.strains) + 1)
    rowgap!(fig.layout, 8)

    savefig(fig, "sirvillage", "epidemic")
    return fig
end

# ===========================================================================
# Figure 3: strain phylogeny (parent -> child), node color = infectivity,
# node size = virulence.
# ===========================================================================
function figure_phylogeny(physical)
    ids = sort(collect(keys(physical.strains)))
    n = length(ids)
    idpos = Dict(id => k for (k, id) in enumerate(ids))

    g = SimpleDiGraph(n)
    for id in ids
        p = physical.strains[id].parent
        if p != 0 && haskey(idpos, p)
            add_edge!(g, idpos[p], idpos[id])
        end
    end

    infect = [physical.strains[id].infectivity for id in ids]
    virul = [physical.strains[id].virulence for id in ids]
    # Node size mapped from virulence, node color from infectivity.
    vmin, vmax = extrema(virul)
    sizes = [12 + 22 * (vmax > vmin ? (v - vmin) / (vmax - vmin) : 0.5)
             for v in virul]

    fig = Figure(size = (700, 520))
    ax = Axis(fig[1, 1];
        title = "Strain phylogeny ($n strains): color = infectivity, size = virulence")
    hidedecorations!(ax)
    hidespines!(ax)

    layout = Buchheim()
    p = graphplot!(ax, g;
        layout = layout,
        node_size = sizes,
        node_color = infect,
        node_attr = (; colormap = :viridis),
        arrow_size = 12,
        edge_color = (:black, 0.3),
        edge_width = 1.2,
        nlabels = string.(ids),
        nlabels_fontsize = 11,
        nlabels_align = (:center, :bottom),
    )
    Colorbar(fig[1, 2], colorrange = extrema(infect), colormap = :viridis,
             label = "infectivity", height = Relative(0.7))

    savefig(fig, "sirvillage", "phylogeny")
    return fig
end

function main()
    obs, physical, ended = run_village()
    println("captured ", length(obs.when), " firings; run ended at ", ended, " days")
    println("strains at end: ", length(physical.strains),
            " (next_strain_id=", physical.next_strain_id, ")")
    figure_compartments()
    figure_epidemic(obs)
    figure_phylogeny(physical)
end

main()
