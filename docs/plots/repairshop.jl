# Figures for the Repair Shop model documentation.
#
#   1. repairshop_states  — the one-machine two-state diagram (model.md)
#   2. repairshop_differentiability — trace log-likelihood vs breakdown rate
#      with the ForwardDiff tangent (likelihood.md)
#   3. repairshop_estimators — the estimator family against the closed-form
#      oracle, per parameter component (gradients.md)

include(joinpath(@__DIR__, "common.jl"))
using ChronoSimExamples
using Logging

const RS = ChronoSimExamples.RepairShop
const RSG = ChronoSimExamples.RepairShopGradients

quiet(f) = with_logger(f, ConsoleLogger(stderr, Logging.Warn))

# --- Figure 1: two-state diagram ------------------------------------------------

function state_diagram()
    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1])
    hidedecorations!(ax)
    hidespines!(ax)
    xlims!(ax, 0, 10)
    ylims!(ax, 0, 6)

    running_c = PALETTE[3]   # green
    down_c    = PALETTE[2]   # orange

    # Boxes
    bw, bh = 2.6, 1.4
    rx, ry = 1.0, 2.3        # running box lower-left
    dx, dy = 6.4, 2.3        # down box lower-left
    poly!(ax, Rect(rx, ry, bw, bh); color = (running_c, 0.15),
          strokecolor = running_c, strokewidth = 2.0)
    poly!(ax, Rect(dx, dy, bw, bh); color = (down_c, 0.15),
          strokecolor = down_c, strokewidth = 2.0)
    text!(ax, rx + bw/2, ry + bh/2; text = "running", align = (:center, :center),
          fontsize = 20, font = :bold, color = running_c)
    text!(ax, dx + bw/2, dy + bh/2; text = "down", align = (:center, :center),
          fontsize = 20, font = :bold, color = down_c)

    # Breakdown arrow: running -> down (upper)
    xa, xb = rx + bw, dx
    y_top, y_bot = ry + bh - 0.15, ry + 0.15
    arrows!(ax, [xa + 0.05], [y_top + 0.55], [xb - xa - 0.1], [0.0];
            linewidth = 2.5, arrowsize = 16, color = PALETTE[1])
    text!(ax, (xa + xb)/2, y_top + 0.9; text = "Breakdown (rate θ₁)",
          align = (:center, :bottom), fontsize = 16, color = PALETTE[1])

    # Repair arrow: down -> running (lower)
    arrows!(ax, [xb - 0.05], [y_bot - 0.55], [xa - xb + 0.1], [0.0];
            linewidth = 2.5, arrowsize = 16, color = PALETTE[5])
    text!(ax, (xa + xb)/2, y_bot - 0.9; text = "Repair (rate θ₂)",
          align = (:center, :top), fontsize = 16, color = PALETTE[5])

    text!(ax, 5.0, 5.4; text = "One machine — an independent two-state chain",
          align = (:center, :center), fontsize = 15, color = (:black, 0.6))
    return fig
end

# --- Figure 2: differentiability of the trace log-likelihood --------------------

function differentiability_figure()
    θ0 = [0.3, 1.0]
    record = quiet(() -> RS.record_repair_shop(θ0; machine_cnt = 3,
                                               horizon = 20.0, seed = 90210).record)

    b0 = θ0[1]
    grid = range(0.12, 0.60; length = 61)
    ll = quiet(() -> [RS.repair_shop_loglik([b, θ0[2]], record; machine_cnt = 3)
                      for b in grid])
    ll0 = quiet(() -> RS.repair_shop_loglik(θ0, record; machine_cnt = 3))
    slope = quiet(() -> RS.repair_shop_score(θ0, record; machine_cnt = 3)[1])

    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1];
              title = "ForwardDiff gives the exact slope of the trace log-likelihood",
              xlabel = "breakdown rate θ₁   (repair rate θ₂ = $(θ0[2]) fixed)",
              ylabel = "trace log-likelihood")

    lines!(ax, grid, ll; color = PALETTE[1], linewidth = 2.5,
           label = "log-likelihood of a fixed trace")

    # Tangent line at θ1 = 0.3 with slope = score[1].
    tan_x = range(b0 - 0.12, b0 + 0.12; length = 2)
    tan_y = ll0 .+ slope .* (tan_x .- b0)
    lines!(ax, tan_x, tan_y; color = PALETTE[2], linewidth = 2.0, linestyle = :dash,
           label = "tangent, slope = repair_shop_score(θ)[1] = $(round(slope; digits=2))")

    scatter!(ax, [b0], [ll0]; color = PALETTE[2], markersize = 13)
    text!(ax, b0, ll0; text = "  θ₁ = 0.3", align = (:left, :top), fontsize = 14,
          color = PALETTE[2])

    axislegend(ax; position = :rb)
    return fig
end

# --- Figure 3: estimator family vs oracle ---------------------------------------

function estimator_figure()
    θ = [0.3, 1.0]
    machine_cnt = 3
    horizon = 8.0
    nreps = 800

    oracle = RSG.expected_down_gradient(θ, machine_cnt, horizon)
    pair = quiet(() -> RSG.repair_shop_pairing(θ; machine_cnt, horizon, nreps))
    br   = quiet(() -> RSG.repair_shop_branching(θ; machine_cnt, horizon, nreps))
    spa  = quiet(() -> RSG.repair_shop_spa(θ; machine_cnt, horizon, nreps))

    # Estimators (excluding the oracle, which is drawn as a reference line/marker).
    labels = ["score (pairing)", "branching", "SPA"]
    ests   = [pair.score, br.estimate, spa.estimate]
    errs   = [pair.score_stderr, br.stderr, spa.stderr]
    cols   = [PALETTE[1], PALETTE[3], PALETTE[5]]

    fig = Figure(size = (820, 460))
    ax = Axis(fig[1, 1];
              title = "The estimator family agrees with the closed-form oracle",
              ylabel = "∂ E[#down(T)] / ∂θ",
              xticks = ([1, 2], ["∂/∂θ₁  (breakdown)", "∂/∂θ₂  (repair)"]))

    ngrp = length(labels)
    width = 0.18
    offs = range(-(ngrp - 1)/2, (ngrp - 1)/2; length = ngrp) .* width

    for (k, comp) in enumerate([1, 2])
        for (j, _) in enumerate(labels)
            x = comp + offs[j]
            barplot!(ax, [x], [ests[j][comp]]; width = width, color = cols[j],
                     label = comp == 1 ? labels[j] : nothing)
            errorbars!(ax, [x], [ests[j][comp]], [errs[j][comp]];
                       whiskerwidth = 8, color = :black, linewidth = 1.2)
        end
        # Oracle reference marker spanning the group.
        scatter!(ax, [comp], [oracle[comp]]; marker = :diamond, markersize = 15,
                 color = PALETTE[2], strokecolor = :black, strokewidth = 0.8,
                 label = comp == 1 ? "oracle (closed form)" : nothing)
    end

    axislegend(ax; position = :rt, orientation = :horizontal, nbanks = 2)
    text!(ax, 0.5, 0.02; space = :relative,
          text = "nreps = $nreps, horizon = $horizon, θ = [0.3, 1.0]; ±1 stderr",
          align = (:left, :bottom), fontsize = 12, color = (:black, 0.55))
    return fig
end

# --- render ---------------------------------------------------------------------

savefig(state_diagram(), "repairshop", "repairshop_states")
savefig(differentiability_figure(), "repairshop", "repairshop_differentiability")
savefig(estimator_figure(), "repairshop", "repairshop_estimators")
