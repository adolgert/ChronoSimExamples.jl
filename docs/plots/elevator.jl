include(joinpath(@__DIR__, "common.jl"))
using ChronoSimExamples
using ChronoSim
using Logging
using Random
using StatsBase

const E = ChronoSimExamples.ElevatorExample

# ---------------------------------------------------------------------------
# Observer that captures the time series we want to plot.
# Called as (physical, when, event, changed_places) after each firing.
# ---------------------------------------------------------------------------
mutable struct CaptureObserver
    when::Vector{Float64}
    floor::Vector{Int}            # elevator 1 floor
    doors_open::Vector{Bool}      # elevator 1 doors
    person_loc::Vector{Int}       # person 1 location (0 while riding)
    person_elev::Vector{Int}      # person 1 elevator (0 if not riding)
    events::Vector{Symbol}        # event type name that just fired
    CaptureObserver() = new(Float64[], Int[], Bool[], Int[], Int[], Symbol[])
end

function (obs::CaptureObserver)(physical, when, event, changed_places)
    push!(obs.when, when)
    push!(obs.floor, physical.elevator[1].floor)
    push!(obs.doors_open, physical.elevator[1].doors_open)
    push!(obs.person_loc, physical.person[1].location)
    push!(obs.person_elev, physical.person[1].elevator)
    push!(obs.events, ChronoSim.clock_key(event)[1])
    return nothing
end

# ---------------------------------------------------------------------------
# Run the same 1-person / 1-elevator / 3-floor scenario as E.run_elevator,
# but with our own observer so we can capture a trajectory.
# ---------------------------------------------------------------------------
function run_capture()
    person_cnt, elevator_cnt, floor_cnt = 1, 1, 3
    minutes = 120.0
    physical = E.ElevatorSystem(person_cnt, elevator_cnt, floor_cnt)
    included_transitions = [
        E.PickNewDestination, E.CallElevator, E.OpenElevatorDoors,
        E.EnterElevator, E.ExitElevator, E.CloseElevatorDoors,
        E.MoveElevator, E.StopElevator, E.DispatchElevator,
    ]
    obs = CaptureObserver()
    sim = ChronoSim.SimulationFSM(
        physical, included_transitions;
        sampler=ChronoSim.NextReactionMethod(), key_type=Tuple,
        rng=Xoshiro(93472934), observer=obs, policy=ChronoSim.NoPolicy(),
    )
    stop_condition = (physical, step_idx, event, when) -> when > minutes
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, E.init_physical, stop_condition)
    end
    return obs, sim.when
end

# ===========================================================================
# Figure 1: structural diagram of the elevator car's service cycle plus the
# rider's lifecycle.
# ===========================================================================
function figure_lifecycle()
    fig = Figure(size = (820, 470))
    ax = Axis(fig[1, 1])
    hidedecorations!(ax)
    hidespines!(ax)
    ax.aspect = DataAspect()
    xlims!(ax, 0, 20)
    ylims!(ax, 0, 11)

    boxw, boxh = 3.6, 1.3

    function node!(cx, cy, label, color)
        poly!(ax, Point2f[(cx - boxw/2, cy - boxh/2), (cx + boxw/2, cy - boxh/2),
                          (cx + boxw/2, cy + boxh/2), (cx - boxw/2, cy + boxh/2)];
              color = (color, 0.16), strokecolor = color, strokewidth = 1.6)
        text!(ax, cx, cy; text = label, align = (:center, :center),
              fontsize = 13, color = :black)
        return (cx, cy)
    end

    # Edge that stops at the box borders (roughly), drawn as an arrow.
    function edge!(p, q; color = PALETTE[6], label = nothing, laboff = (0.0, 0.0),
                   pad = boxh/2 + 0.15)
        dx, dy = q[1] - p[1], q[2] - p[2]
        L = hypot(dx, dy)
        ux, uy = dx / L, dy / L
        # shrink both ends so arrows touch box edges, not centers
        padx = boxw/2 + 0.15
        sp = (abs(ux) > abs(uy)) ? padx / max(abs(ux), 1e-3) : pad / max(abs(uy), 1e-3)
        sq = sp
        x0, y0 = p[1] + ux*sp, p[2] + uy*sp
        x1, y1 = q[1] - ux*sq, q[2] - uy*sq
        arrows!(ax, [x0], [y0], [x1 - x0], [y1 - y0];
                color = color, linewidth = 1.8, arrowsize = 11)
        if label !== nothing
            mx, my = (x0 + x1)/2 + laboff[1], (y0 + y1)/2 + laboff[2]
            text!(ax, mx, my; text = label, align = (:center, :center),
                  fontsize = 10.5, color = color)
        end
    end

    ecol = PALETTE[1]  # elevator cycle color
    pcol = PALETTE[2]  # person lifecycle color

    # --- Elevator car service cycle (top row, a loop) ---
    text!(ax, 10, 10.4; text = "Elevator car service cycle", align = (:center, :center),
          fontsize = 14, color = ecol, font = :bold)
    stat  = node!(2.4, 8.4, "Stationary",  ecol)
    disp  = node!(7.0, 8.4, "Dispatch\n(set direction)", ecol)
    move  = node!(11.6, 8.4, "Move\n(one floor)", ecol)
    open  = node!(16.2, 8.4, "Open doors", ecol)
    close = node!(11.6, 6.2, "Close doors", ecol)
    stop  = node!(2.4, 6.2, "Stop\n(at boundary)", ecol)

    edge!(stat, disp; color = ecol, label = "call", laboff = (0, 0.4))
    edge!(disp, move; color = ecol)
    edge!(move, open; color = ecol, label = "reached call", laboff = (0, 0.4))
    edge!(open, close; color = ecol)
    edge!(close, move; color = ecol, label = "keep going", laboff = (0, 0.35))
    edge!(move, stop; color = ecol, label = "boundary", laboff = (-0.2, 0.0))
    edge!(stop, stat; color = ecol)

    # --- Person lifecycle (bottom row) ---
    text!(ax, 10, 4.5; text = "Rider lifecycle", align = (:center, :center),
          fontsize = 14, color = pcol, font = :bold)
    idle  = node!(2.4, 2.9, "Idle\non floor",  pcol)
    pick  = node!(6.6, 2.9, "PickNew\nDestination", pcol)
    call  = node!(10.8, 2.9, "CallElevator\n(waiting)", pcol)
    board = node!(15.0, 2.9, "EnterElevator\n(riding)", pcol)
    exitn = node!(15.0, 0.9, "ExitElevator", pcol)

    edge!(idle, pick; color = pcol)
    edge!(pick, call; color = pcol)
    edge!(call, board; color = pcol, label = "doors open", laboff = (0, 0.35))
    edge!(board, exitn; color = pcol, label = "arrive", laboff = (1.15, 0.0))
    edge!(exitn, idle; color = pcol, label = "back to idle", laboff = (0, -0.45))

    savefig(fig, "elevator", "lifecycle")
    return fig
end

# ===========================================================================
# Figure 2: elevator floor trajectory over a 120-minute run.
# ===========================================================================
function figure_trajectory(obs)
    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1];
        title = "Elevator car over a 120-minute run",
        xlabel = "time (minutes)", ylabel = "floor")
    ax.yticks = 1:3
    ylims!(ax, 0.5, 3.6)

    t = obs.when

    # Person location as a filled band-ish overlay: while riding location==0,
    # so plot the person's effective floor (their car's floor while riding).
    person_floor = [obs.person_loc[i] == 0 ? obs.floor[i] : obs.person_loc[i]
                    for i in eachindex(t)]

    stairs!(ax, t, person_floor; step = :post, color = (PALETTE[2], 0.55),
            linewidth = 3.5, label = "rider floor")
    stairs!(ax, t, obs.floor; step = :post, color = PALETTE[1],
            linewidth = 2.0, label = "car floor")

    # Mark moments the doors opened (an OpenElevatorDoors firing).
    open_idx = findall(==(:OpenElevatorDoors), obs.events)
    scatter!(ax, t[open_idx], obs.floor[open_idx]; color = PALETTE[4],
             marker = :diamond, markersize = 11, label = "doors open")

    # Mark rider boarding / exiting.
    board_idx = findall(==(:EnterElevator), obs.events)
    exit_idx = findall(==(:ExitElevator), obs.events)
    scatter!(ax, t[board_idx], obs.floor[board_idx]; color = PALETTE[3],
             marker = :utriangle, markersize = 11, label = "board")
    scatter!(ax, t[exit_idx], obs.floor[exit_idx]; color = PALETTE[5],
             marker = :dtriangle, markersize = 11, label = "exit")

    axislegend(ax; position = :rb, orientation = :horizontal, nbanks = 2)
    xlims!(ax, 0, min(120, maximum(t)))

    savefig(fig, "elevator", "trajectory")
    return fig
end

# ===========================================================================
# Figure 3: bar chart of how often each event fired.
# ===========================================================================
function figure_event_counts(obs)
    order = [:PickNewDestination, :CallElevator, :DispatchElevator,
             :OpenElevatorDoors, :EnterElevator, :ExitElevator,
             :CloseElevatorDoors, :MoveElevator, :StopElevator]
    cm = countmap(obs.events)
    counts = [get(cm, s, 0) for s in order]
    labels = String.(order)

    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1];
        title = "Event firings over the 120-minute run",
        xlabel = "count", yticks = (1:length(order), labels))
    ax.yreversed = true
    barplot!(ax, 1:length(order), counts; direction = :x,
             color = PALETTE[1], strokewidth = 0)
    text!(ax, counts, 1:length(order); text = string.(counts),
          align = (:left, :center), offset = (4, 0), fontsize = 12,
          color = :black)
    xlims!(ax, 0, maximum(counts) * 1.15)

    savefig(fig, "elevator", "event_counts")
    return fig
end

function main()
    obs, ended = run_capture()
    println("captured ", length(obs.when), " firings; run ended at ", ended, " minutes")
    figure_lifecycle()
    figure_trajectory(obs)
    figure_event_counts(obs)
end

main()
