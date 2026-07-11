# RepairShop: an end-to-end differentiation workflow on a machine-repair model.
#
# This example shows the three-step pattern that ChronoSim's record/replay
# machinery exists to support:
#
#   1. RUN a simulation once with the `RecordMinimal` execution policy, which
#      captures a `MinimalRecord` — the initializer, the ordered (clock key,
#      firing time) sequence, the horizon, and a flag saying whether any `fire!`
#      body drew randomness.
#   2. CHECK the record with `effect_check`, which replays it through
#      `trace_likelihood` and demands EXACT floating-point equality with the
#      log-likelihood the forward run accumulated. Exact equality (not ≈) is the
#      point: forward execution and trace evaluation share one code path, so any
#      difference is a bug, not roundoff.
#   3. DIFFERENTIATE by evaluating `trace_likelihood` at a θ the forward run
#      never saw — here a vector of `ForwardDiff.Dual` numbers — to get the
#      score (the gradient of the trace log-likelihood in the parameters). The
#      θ (parameter) seam makes this possible: each event's four-argument
#      `enable(event, physical, θ, when)` builds its distribution from the θ the
#      caller supplies, so no global state changes between evaluations.
#
# The model: `machine_cnt` machines run until they break down and are repaired.
# Both clocks are exponential; θ = [breakdown rate, repair rate]. Exponential
# clocks keep the analytic score simple enough to verify by hand in the test
# suite (`test/test_repairshop.jl`), which is what makes this example an
# oracle-checked demonstration rather than a demo that merely runs.

module RepairShop

using ChronoSim
using ChronoSim.ObservedState
using CompetingClocks
using Distributions
using ForwardDiff
using Random
import ChronoSim: generators, precondition, enable, fire!

export record_repair_shop, repair_shop_loglik, repair_shop_score, repair_shop_demo

@enum MachineCondition running down

@keyedby Machine Int64 begin
    condition::MachineCondition
end

@observedphysical Shop begin
    machines::ObservedVector{Machine,Member}
end

function Shop(machine_cnt::Int)
    machines = ObservedArray{Machine,Member}(undef, machine_cnt)
    for i in 1:machine_cnt
        machines[i] = Machine(running)
    end
    return Shop(machines)
end

# The initializer writes every machine's condition so the write notifications
# propose the initial Breakdown events. `RecordMinimal` stores this function in
# the record, so a replay reconstructs the same initial state.
function init!(shop, when, rng)
    for i in eachindex(shop.machines)
        shop.machines[i].condition = running
    end
end

struct Breakdown <: SimEvent
    machine::Int64
end

@conditionsfor Breakdown begin
    @reactto changed(machines[m].condition) do shop
        generate(Breakdown(m))
    end
end

@guard precondition(evt::Breakdown, shop) =
    shop.machines[evt.machine].condition == running

# The four-argument θ seam: the breakdown rate is θ[1], read at enable time from
# the parameter vector the simulation carries. Distributions.jl's Exponential
# takes a scale, so rate θ[1] becomes Exponential(inv(θ[1])); when θ is a vector
# of ForwardDiff duals, `inv` promotes and the dual flows into the distribution.
enable(evt::Breakdown, shop, θ, when) = (Exponential(inv(θ[1])), when)

@fire function fire!(evt::Breakdown, shop, when, rng)
    shop.machines[evt.machine].condition = down
end

struct Repair <: SimEvent
    machine::Int64
end

@conditionsfor Repair begin
    @reactto changed(machines[m].condition) do shop
        generate(Repair(m))
    end
end

@guard precondition(evt::Repair, shop) =
    shop.machines[evt.machine].condition == down

# The repair rate is θ[2].
enable(evt::Repair, shop, θ, when) = (Exponential(inv(θ[2])), when)

@fire function fire!(evt::Repair, shop, when, rng)
    shop.machines[evt.machine].condition = running
end

_events() = [Breakdown, Repair]

"""
    record_repair_shop(θ; machine_cnt=3, horizon=20.0, seed=90210)

Run the repair shop once at parameters `θ = [breakdown_rate, repair_rate]` with
the `RecordMinimal` policy and return `(record, policy)`: the `MinimalRecord`
of the run (with its horizon, for censored evaluation) and the policy object
(which `effect_check` consumes). One master `seed` determines the whole
trajectory — CompetingClocks 0.4 derives every per-clock random stream from it.
"""
function record_repair_shop(θ; machine_cnt::Int=3, horizon::Float64=20.0, seed=90210)
    policy = RecordMinimal(; initializer=init!)
    sim = SimulationFSM(
        Shop(machine_cnt), _events();
        seed=seed, sampler=NextReactionMethod(), key_type=Tuple,
        step_likelihood=true, policy=policy, params=θ,
    )
    stop_condition = (physical, step_idx, event, when) -> when > horizon
    ChronoSim.run(sim, init!, stop_condition)
    return (record=minimal_record(policy; horizon=horizon), policy=policy)
end

"""
    repair_shop_loglik(θ, record; machine_cnt=3)

The log-likelihood of a recorded trajectory evaluated at an explicit `θ`.
`censor=true` adds the finite-horizon survival tail (no clock fired between the
last event and the record's horizon), so the value covers the whole observation
window, not just the firings. A FRESH evaluation simulation is built per call:
a `SimulationFSM` cannot currently evaluate two traces back to back, so the
closure-over-a-fresh-sim pattern is the supported one. `eltype(θ)` is passed as
the likelihood element type so ForwardDiff dual numbers accumulate without
truncation.
"""
function repair_shop_loglik(θ, record; machine_cnt::Int=3)
    sim = SimulationFSM(
        Shop(machine_cnt), _events();
        seed=1, sampler=NextReactionMethod(), key_type=Tuple,
        step_likelihood=true, likelihood_eltype=eltype(θ),
    )
    return trace_likelihood(sim, init!, record; params=θ, censor=true).loglikelihood
end

"""
    repair_shop_score(θ, record; machine_cnt=3)

The score — the gradient of the recorded trajectory's log-likelihood in θ —
computed by running `ForwardDiff.gradient` through `repair_shop_loglik`. The
trace is FIXED; only the parameters at which it is scored vary, which is why
the result is a deterministic function of `(θ, record)`.
"""
function repair_shop_score(θ, record; machine_cnt::Int=3)
    return ForwardDiff.gradient(
        p -> repair_shop_loglik(p, record; machine_cnt=machine_cnt), θ
    )
end

"""
    repair_shop_demo(; θ=[0.3, 1.0], machine_cnt=3, horizon=20.0, seed=90210)

The whole workflow in one call: record a trajectory at `θ`, evaluate its
log-likelihood, and print the score at the generating parameters. Returns
`(record, loglikelihood, score)`.
"""
function repair_shop_demo(; θ=[0.3, 1.0], machine_cnt::Int=3, horizon::Float64=20.0,
                          seed=90210)
    (; record, policy) = record_repair_shop(
        θ; machine_cnt=machine_cnt, horizon=horizon, seed=seed
    )
    ll = repair_shop_loglik(θ, record; machine_cnt=machine_cnt)
    g = repair_shop_score(θ, record; machine_cnt=machine_cnt)
    println("recorded $(length(record.firings)) firings over horizon $(record.horizon)")
    println("log-likelihood at θ=$(θ): $(ll)")
    println("score (dloglik/dθ) at θ=$(θ): $(g)")
    return (record=record, loglikelihood=ll, score=g)
end

end # module RepairShop

if abspath(PROGRAM_FILE) == @__FILE__
    using .RepairShop
    repair_shop_demo()
end
