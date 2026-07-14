include(joinpath(@__DIR__, "common.jl"))
using ChronoSimExamples
using ChronoSim
using Distributions
using Logging

const R = ChronoSimExamples.ReliabilitySim

# ---------------------------------------------------------------------------
# Figure 1: state-machine diagram of a single machine.
# ---------------------------------------------------------------------------

# Draw a filled box centred at `c` with white label text.
function state_box!(ax, c, label, color; w = 1.7, h = 0.8)
    rect = Rect2f(c[1] - w / 2, c[2] - h / 2, w, h)
    poly!(ax, rect; color = color, strokecolor = (:black, 0.35), strokewidth = 1.2)
    text!(ax, Point2f(c); text = label, align = (:center, :center),
        color = :white, font = :bold, fontsize = 16)
    return nothing
end

# Draw an arrow from p1 to p2 with a triangular head and an optional label.
function draw_arrow!(ax, p1, p2; color = :black, lw = 2.0, hw = 0.11, hl = 0.20,
        label = nothing, laboff = (0.0, 0.0), labalign = (:center, :center))
    p1 = Point2f(p1)
    p2 = Point2f(p2)
    d = p2 - p1
    u = d ./ hypot(d[1], d[2])
    perp = Point2f(-u[2], u[1])
    base = p2 - u * hl
    lines!(ax, Point2f[p1, base]; color = color, linewidth = lw)
    poly!(ax, Point2f[p2, base + perp * hw, base - perp * hw]; color = color)
    if label !== nothing
        mid = (p1 + p2) / 2 + Point2f(laboff)
        text!(ax, mid; text = label, align = labalign, fontsize = 13,
            color = :black)
    end
    return nothing
end

function machine_diagram()
    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1])
    hidedecorations!(ax)
    hidespines!(ax)
    ax.aspect = DataAspect()

    ready_c = (0.0, 1.0)
    working_c = (4.0, 1.0)
    broken_c = (4.0, -1.4)

    # StartDay: ready -> working (upper track).
    draw_arrow!(ax, (ready_c[1] + 0.85, ready_c[2] + 0.18),
        (working_c[1] - 0.85, working_c[2] + 0.18);
        color = PALETTE[1], label = "StartDay", laboff = (0.0, 0.22))
    # EndDay: working -> ready (lower track).
    draw_arrow!(ax, (working_c[1] - 0.85, working_c[2] - 0.18),
        (ready_c[1] + 0.85, ready_c[2] - 0.18);
        color = PALETTE[3], label = "EndDay", laboff = (0.0, -0.24))
    # Break: working -> broken (vertical).
    draw_arrow!(ax, (working_c[1], working_c[2] - 0.4),
        (broken_c[1], broken_c[2] + 0.4);
        color = PALETTE[2], label = "Break", laboff = (0.55, 0.0),
        labalign = (:left, :center))
    # Repair: broken -> ready (diagonal).
    draw_arrow!(ax, (broken_c[1] - 0.85, broken_c[2]),
        (ready_c[1] + 0.55, ready_c[2] - 0.4);
        color = PALETTE[4], label = "Repair", laboff = (0.1, 0.32),
        labalign = (:center, :bottom))

    state_box!(ax, ready_c, "ready", PALETTE[3])
    state_box!(ax, working_c, "working", PALETTE[1])
    state_box!(ax, broken_c, "broken", PALETTE[2])

    xlims!(ax, -1.4, 5.9)
    ylims!(ax, -2.3, 1.9)
    return fig
end

# ---------------------------------------------------------------------------
# Figure 2: state occupancy time series from a real multi-day run.
# ---------------------------------------------------------------------------

# Observer matching ChronoSim's (physical, when, event, changed) signature. It
# records the fleet's state histogram after every event fires.
mutable struct StateRecorder
    t::Vector{Float64}
    n_ready::Vector{Int}
    n_working::Vector{Int}
    n_broken::Vector{Int}
    StateRecorder() = new(Float64[], Int[], Int[], Int[])
end

function (rec::StateRecorder)(physical, when, event, changed)
    nr = nw = nb = 0
    for i in eachindex(physical.actors)
        s = physical.actors[i].state
        if s == R.ready
            nr += 1
        elseif s == R.working
            nw += 1
        else
            nb += 1
        end
    end
    push!(rec.t, when)
    push!(rec.n_ready, nr)
    push!(rec.n_working, nw)
    push!(rec.n_broken, nb)
    return nothing
end

# Rebuild run_reliability's simulation with our own observer.
function record_run(days)
    agent_cnt = 15
    physical = R.IndividualState(agent_cnt, 10)
    included_transitions = [R.StartDay, R.EndDay, R.Break, R.Repair]
    rec = StateRecorder()
    sim = SimulationFSM(
        physical, included_transitions;
        rng = Xoshiro(2947223), observer = rec,
        sampler = ChronoSim.NextReactionMethod(), key_type = Tuple,
    )
    initializer = (init_physical, when, rng) -> R.initialize!(init_physical, rng)
    stop_condition = (physical, step_idx, event, when) -> when > days
    ChronoSim.run(sim, initializer, stop_condition)
    return rec
end

function occupancy_series(rec, days)
    # Prepend the warm-start state (all ready) at t = 0.
    t = vcat(0.0, rec.t)
    working = vcat(0, rec.n_working)
    broken = vcat(0, rec.n_broken)
    ready = vcat(15, rec.n_ready)

    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1]; xlabel = "time (days)", ylabel = "machines",
        title = "Fleet state over a 10-day run")
    # Day boundaries.
    vlines!(ax, collect(1.0:floor(days)); color = (:black, 0.12), linewidth = 1)

    stairs!(ax, t, working; step = :post, color = PALETTE[1],
        linewidth = 2.2, label = "working")
    stairs!(ax, t, ready; step = :post, color = PALETTE[3],
        linewidth = 2.2, label = "ready")
    stairs!(ax, t, broken; step = :post, color = PALETTE[2],
        linewidth = 2.2, label = "broken")

    xlims!(ax, 0, days)
    ylims!(ax, -0.5, 15.5)
    axislegend(ax; position = :rc)
    return fig
end

# ---------------------------------------------------------------------------
# Figure 3: the model's three clock distributions on one axis.
# ---------------------------------------------------------------------------

function clock_distributions()
    done_dist = LogUniform(0.8, 0.99)   # end-of-day clock
    fail_dist = LogNormal(1.5, 0.4)     # failure clock (vs. work age)
    repair_dist = Weibull(1.0, 2.0)     # repair clock

    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1]; xlabel = "duration (days)", ylabel = "probability density",
        title = "Heterogeneous clocks driving the model")

    xs = range(0.01, 9.0, length = 600)
    lines!(ax, xs, pdf.(repair_dist, xs); color = PALETTE[4], linewidth = 2.4,
        label = "Repair · Weibull(1.0, 2.0)")
    lines!(ax, xs, pdf.(fail_dist, xs); color = PALETTE[2], linewidth = 2.4,
        label = "Break · LogNormal(1.5, 0.4)")
    lines!(ax, xs, pdf.(done_dist, xs); color = PALETTE[1], linewidth = 2.4,
        label = "EndDay · LogUniform(0.8, 0.99)")

    xlims!(ax, 0, 9)
    axislegend(ax; position = :rt)
    return fig
end

# ---------------------------------------------------------------------------

with_logger(ConsoleLogger(stderr, Logging.Warn)) do
    savefig(machine_diagram(), "reliability", "state_machine")

    rec = record_run(10)
    savefig(occupancy_series(rec, 10), "reliability", "occupancy")

    savefig(clock_distributions(), "reliability", "clocks")
end
