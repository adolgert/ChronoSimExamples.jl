# RepairShopGradients: the estimator family on one model.
#
# `repairshop.jl` demonstrates the SCORE path: record a trajectory, replay its
# likelihood at a dual θ. This companion demonstrates the rest of the
# ClockGradients estimator family on the same law, all against one closed-form
# oracle:
#
#   * the score/IPA PAIRING (`paired_simulate_and_estimate`) on the pure model,
#     which flags that the pathwise estimator is identically zero for a
#     terminal count while the score recovers the truth;
#   * the BRANCHING estimator (`branching_gradient`) driven through the live
#     ChronoSim simulation via the ClockGradients–ChronoSim extension;
#   * SMOOTHED PERTURBATION ANALYSIS (`spa_gradient`) through the same live
#     simulation, which needs one extra ingredient — the PURE MODEL TWIN
#     defined below — and shows off its criticality gate: this shop's machines
#     are independent, so EVERY event-order swap provably commutes, the gate
#     spawns no clones at all, and the whole derivative flows through SPA's
#     horizon boundary term (machines crossing into the observation window).
#
# The functional is the number of down machines at the horizon. Each machine is
# an independent two-state Markov chain (up →(λ) down →(μ) up), so
#
#     E[#down(T)] = n · (λ/(λ+μ)) · (1 − exp(−(λ+μ)T))
#
# in closed form, and ForwardDiff through it gives the exact gradient oracle.
#
# THE PURE MODEL TWIN. SPA replays records and speculatively fires event pairs,
# so it needs the law as a pure five-function ClockGradients model whose clock
# keys match ChronoSim's `clock_key` convention — (:Breakdown, m) and
# (:Repair, m) here. The estimator audits the twin's enabled set against the
# live simulation's at every step and stops with a named error on the first
# disagreement, so the hand-written duplication cannot silently drift.

module RepairShopGradients

using ChronoSim
using CompetingClocks: NextReactionMethod
using ClockGradients
using ClockGradients: TerminalObservable
using Distributions
using ForwardDiff
using Random: Xoshiro
import ClockGradients: initial_state, clockkeytype, enabled, clock_distribution, fire

using ..RepairShop: RepairShop

export ShopLaw, expected_down, expected_down_gradient,
    repair_shop_pairing, repair_shop_branching, repair_shop_spa,
    repair_shop_gradients_demo

# --- the pure model twin -------------------------------------------------------

struct ShopLawState
    down::Vector{Bool}
end
# Value equality is what lets SPA's criticality gate recognize commuting swaps;
# the default == on a struct with an array field is identity, which would
# silently disable the gate.
Base.:(==)(a::ShopLawState, b::ShopLawState) = a.down == b.down

"""
    ShopLaw(machine_cnt)

The repair shop as a pure ClockGradients model: `machine_cnt` machines, each
breaking down at rate `θ[1]` and being repaired at rate `θ[2]`, with clock
keys matching the ChronoSim model's `clock_key` convention.
"""
struct ShopLaw
    machine_cnt::Int
end
initial_state(m::ShopLaw) = ShopLawState(fill(false, m.machine_cnt))
clockkeytype(::ShopLaw) = Tuple
function enabled(m::ShopLaw, s::ShopLawState)
    ks = Tuple[]
    for i in 1:m.machine_cnt
        push!(ks, s.down[i] ? (:Repair, i) : (:Breakdown, i))
    end
    ks
end
clock_distribution(::ShopLaw, θ, key::Tuple) =
    key[1] === :Breakdown ? Exponential(one(eltype(θ)) / θ[1]) :
                            Exponential(one(eltype(θ)) / θ[2])
function fire(::ShopLaw, s::ShopLawState, key::Tuple)
    down = copy(s.down)
    down[key[2]] = key[1] === :Breakdown
    ShopLawState(down)
end

# The functional on the twin's states and on the live physical state.
ndown_twin(s::ShopLawState) = count(s.down)
ndown_physical(shop) =
    count(shop.machines[i].condition == RepairShop.down
          for i in eachindex(shop.machines))

# --- the closed-form oracle -----------------------------------------------------

"""
    expected_down(θ, machine_cnt, horizon)

Exact `E[#down(horizon)]` starting all-up: each machine is an independent
two-state chain, so the down probability is `(λ/(λ+μ))(1 − exp(−(λ+μ)T))`.
"""
function expected_down(θ, machine_cnt::Integer, horizon::Real)
    λ, μ = θ[1], θ[2]
    machine_cnt * (λ / (λ + μ)) * (1 - exp(-(λ + μ) * horizon))
end

expected_down_gradient(θ, machine_cnt::Integer, horizon::Real) =
    ForwardDiff.gradient(p -> expected_down(p, machine_cnt, horizon), collect(float.(θ)))

# --- the estimators, one call each ------------------------------------------------

# A fresh live simulation at params = θ; the estimators reseed every replication.
_sim_factory(θ, machine_cnt) = () -> SimulationFSM(
    RepairShop.Shop(machine_cnt), [RepairShop.Breakdown, RepairShop.Repair];
    seed=UInt64(1), sampler=NextReactionMethod(), key_type=Tuple, params=θ)

"""
    repair_shop_pairing(θ; machine_cnt=3, horizon=8.0, nreps=4000, seed=2028)

The score/IPA pairing on the pure model: for the terminal down-count the
pathwise estimate is identically zero (a frozen discrete read), the score
recovers the oracle, and the pairing flags the bias.
"""
repair_shop_pairing(θ; machine_cnt::Int=3, horizon::Float64=8.0,
                    nreps::Int=4000, seed::Int=2028) =
    paired_simulate_and_estimate(Xoshiro(seed), ShopLaw(machine_cnt),
                                 collect(float.(θ)), NextReactionMethod(),
                                 TerminalObservable(ndown_twin);
                                 nreps=nreps, horizon=horizon)

"""
    repair_shop_branching(θ; machine_cnt=3, horizon=8.0, nreps=800, seed=2029)

The weak-derivative branching estimator through the LIVE ChronoSim simulation
(the ClockGradients–ChronoSim extension supplies the branchable-world verbs).
"""
repair_shop_branching(θ; machine_cnt::Int=3, horizon::Float64=8.0,
                      nreps::Int=800, seed::Int=2029) =
    branching_gradient(_sim_factory(collect(float.(θ)), machine_cnt),
                       RepairShop.init!, collect(float.(θ)), ndown_physical;
                       nreps=nreps, horizon=horizon, seed=seed,
                       branch_rng_seed=seed + 1)

"""
    repair_shop_spa(θ; machine_cnt=3, horizon=8.0, nreps=4000, seed=2030)

Smoothed perturbation analysis through the live ChronoSim simulation with
`ShopLaw` as the pure model twin. On this model the criticality gate proves
every event-order swap harmless (independent machines commute), so the
estimator spawns NO clones and the whole derivative is its horizon boundary
term — check `skip_fraction == 1.0` and `clones_per_rep == 0.0` in the result.

An honest caveat this example makes visible: with no order effects to recover,
SPA has no advantage here — the plain score estimate is several times tighter
per replication. SPA earns its keep on functionals whose derivative is carried
by event ORDER (queueing counts, contended first passage); this model
demonstrates its gate and horizon machinery, not its best-case variance.
"""
repair_shop_spa(θ; machine_cnt::Int=3, horizon::Float64=8.0,
                nreps::Int=4000, seed::Int=2030) =
    spa_gradient(_sim_factory(collect(float.(θ)), machine_cnt),
                 RepairShop.init!, ShopLaw(machine_cnt), collect(float.(θ)),
                 TerminalObservable(ndown_twin);
                 nreps=nreps, horizon=horizon, seed=seed)

"""
    repair_shop_gradients_demo(; θ=[0.3, 1.0], machine_cnt=3, horizon=8.0)

Run the family and print every estimate against the closed-form oracle.
"""
function repair_shop_gradients_demo(; θ=[0.3, 1.0], machine_cnt::Int=3,
                                    horizon::Float64=8.0)
    oracle = expected_down_gradient(θ, machine_cnt, horizon)
    println("oracle dE[#down(T)]/dθ = $(oracle)")
    pair = repair_shop_pairing(θ; machine_cnt=machine_cnt, horizon=horizon)
    println("score = $(pair.score) ± $(pair.score_stderr); IPA = $(pair.ipa) " *
            "(pinned zero); bias flagged: $(pair.bias_detected)")
    br = repair_shop_branching(θ; machine_cnt=machine_cnt, horizon=horizon)
    println("branching = $(br.estimate) ± $(br.stderr) " *
            "($(br.clones_per_rep) clones/rep)")
    spa = repair_shop_spa(θ; machine_cnt=machine_cnt, horizon=horizon)
    println("SPA = $(spa.estimate) ± $(spa.stderr) " *
            "(skip fraction $(spa.skip_fraction), $(spa.clones_per_rep) clones/rep)")
    return (oracle=oracle, pairing=pair, branching=br, spa=spa)
end

end # module RepairShopGradients
