include(joinpath(@__DIR__, "common.jl"))
using ChronoSimExamples
using ChronoSim
using Random
using Logging

const LS = ChronoSimExamples.LandSpread

# ---------------------------------------------------------------------------
# Reconstruct the point coordinates the run used. `run_landspread` builds its
# landscape with `Landscape(point_cnt, rng)` where `rng = Xoshiro(9437294723)`,
# and the constructor's very first action is `locs = rand(rng, 2, N)`. We
# replicate that exact call to recover the coordinates, which are not stored.
function reconstruct_locs(point_cnt)
    rng = Xoshiro(9437294723)
    return rand(rng, 2, point_cnt)
end

# Pull the spread events (source, destination, time) out of a trajectory,
# skipping the leading InitializeEvent.
function spread_edges(traj)
    edges = Tuple{Int,Int,Float64}[]
    for te in traj
        te.event[1] === :Spread || continue
        push!(edges, (te.event[2], te.event[3], te.when))
    end
    return edges
end

# ===========================================================================
# Figure 1: spatial spread map. Points at their coordinates, colored by
# infection time, with an arrow from source to destination for every spread.
# ===========================================================================
function figure_spread_map(traj, point_cnt)
    locs = reconstruct_locs(point_cnt)
    edges = spread_edges(traj)

    # Infection time per point: seed (point 1) at 0.0, each destination at its
    # spread time. Points never occupied (there should be none at saturation)
    # get NaN.
    inf_time = fill(NaN, point_cnt)
    inf_time[1] = 0.0
    for (_, dst, when) in edges
        inf_time[dst] = when
    end
    tmax = maximum(filter(!isnan, inf_time))

    fig = Figure(size = (620, 560))
    ax = Axis(fig[1, 1];
        title = "Spread cascade across $point_cnt points",
        aspect = DataAspect())
    hidedecorations!(ax)
    ax.leftspinevisible = ax.bottomspinevisible = true
    ax.topspinevisible = ax.rightspinevisible = true
    for s in (:leftspinecolor, :rightspinecolor, :topspinecolor, :bottomspinecolor)
        setproperty!(ax, s, (:black, 0.15))
    end

    # Arrows source -> destination showing the infection tree.
    xs = Float64[]; ys = Float64[]; us = Float64[]; vs = Float64[]
    for (src, dst, _) in edges
        push!(xs, locs[1, src]); push!(ys, locs[2, src])
        push!(us, locs[1, dst] - locs[1, src]); push!(vs, locs[2, dst] - locs[2, src])
    end
    arrows!(ax, xs, ys, us, vs;
        color = (:black, 0.30), linewidth = 1.2, arrowsize = 8)

    # Points colored by infection time.
    cmap = :viridis
    sc = scatter!(ax, locs[1, :], locs[2, :];
        color = inf_time, colormap = cmap, colorrange = (0.0, tmax),
        markersize = 15, strokecolor = (:black, 0.4), strokewidth = 0.6)

    # Mark the seed distinctly.
    scatter!(ax, [locs[1, 1]], [locs[2, 1]];
        marker = :star5, markersize = 26, color = PALETTE[2],
        strokecolor = :black, strokewidth = 1.0, label = "seed")

    Colorbar(fig[1, 2], sc; label = "infection time")
    axislegend(ax; position = :rt, framevisible = true)

    savefig(fig, "landspread", "spread_map")
    return fig
end

# ===========================================================================
# Figure 2: saturation curve. Cumulative occupied points versus time.
# ===========================================================================
function figure_saturation(traj, point_cnt)
    edges = spread_edges(traj)
    # Start with the seed occupied at t=0, then one more point per spread event.
    times = vcat(0.0, [when for (_, _, when) in edges])
    occupied = collect(1:length(times))     # 1, 2, 3, ... N

    fig = Figure(size = (820, 440))
    ax = Axis(fig[1, 1];
        title = "Saturation of a $point_cnt-point landscape",
        xlabel = "time", ylabel = "points occupied")

    stairs!(ax, times, occupied; step = :post, color = PALETTE[1], linewidth = 2.5)
    scatter!(ax, times, occupied; color = PALETTE[1], markersize = 7)
    hlines!(ax, [point_cnt]; color = (PALETTE[6], 0.7), linestyle = :dash,
        linewidth = 1.2)
    text!(ax, times[end], point_cnt; text = "all $point_cnt occupied",
        align = (:right, :top), offset = (-2, -4), fontsize = 12,
        color = PALETTE[6])

    ylims!(ax, 0, point_cnt + 1)
    xlims!(ax, 0, maximum(times) * 1.03)

    savefig(fig, "landspread", "saturation")
    return fig
end

# ===========================================================================
# Figure 3: the θ seam. Score the SAME recorded trace at a grid of θ values
# and plot how the trace log-likelihood varies smoothly with the parameter.
# ===========================================================================
function figure_theta_seam(point_cnt)
    base = LS.SPREAD_THETA          # [scale coeff, distance exponent]

    # Sweep θ[1] (scale coefficient) with θ[2] fixed.
    scale_grid = range(0.05, 0.20; length = 25)
    ll_scale = [LS.landspread_likelihood(point_cnt; θ = [s, base[2]]) for s in scale_grid]

    # Sweep θ[2] (distance exponent) with θ[1] fixed.
    exp_grid = range(0.6, 1.8; length = 25)
    ll_exp = [LS.landspread_likelihood(point_cnt; θ = [base[1], e]) for e in exp_grid]

    fig = Figure(size = (820, 440))

    ax1 = Axis(fig[1, 1];
        title = "Log-likelihood vs scale θ[1]",
        xlabel = "θ[1] (scale coefficient)", ylabel = "trace log-likelihood")
    lines!(ax1, scale_grid, ll_scale; color = PALETTE[1], linewidth = 2.5)
    vlines!(ax1, [base[1]]; color = (PALETTE[2], 0.8), linestyle = :dash, linewidth = 1.2)
    scatter!(ax1, [base[1]], [LS.landspread_likelihood(point_cnt; θ = base)];
        color = PALETTE[2], markersize = 10)

    ax2 = Axis(fig[1, 2];
        title = "Log-likelihood vs exponent θ[2]",
        xlabel = "θ[2] (distance exponent)")
    lines!(ax2, exp_grid, ll_exp; color = PALETTE[3], linewidth = 2.5)
    vlines!(ax2, [base[2]]; color = (PALETTE[2], 0.8), linestyle = :dash, linewidth = 1.2)

    savefig(fig, "landspread", "theta_seam")
    return fig
end

function main()
    point_cnt = 40
    traj = with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        LS.run_landspread(point_cnt)
    end
    println("trajectory: ", length(traj), " entries (",
        count(te -> te.event[1] === :Spread, traj), " spreads)")

    figure_spread_map(traj, point_cnt)
    figure_saturation(traj, point_cnt)

    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        figure_theta_seam(10)
    end
end

main()
