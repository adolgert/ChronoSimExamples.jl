# The RepairShop example is the package's end-to-end differentiation workflow:
# record a trajectory with RecordMinimal, verify the record with effect_check's
# exact-equality replay, then evaluate trace_likelihood at a dual-valued θ with
# ForwardDiff to get the score. Exponential clocks make the score analytic, so
# the ForwardDiff gradient is compared against a hand-derived oracle computed by
# walking the record, not against another autodiff.

using ChronoSim
using CompetingClocks
using Test

using ChronoSimExamples: RepairShop

# The analytic score of a censored exponential-clock trajectory. For competing
# exponential clocks the trace log-likelihood is
#   loglik = n_break·log(λ_b) + n_repair·log(λ_r) − λ_b·E_run − λ_r·E_down
# where E_run is the total machine-time spent running (each running machine has
# an enabled Breakdown clock at rate λ_b) and E_down the total machine-time
# spent down, both accumulated over the FULL window [0, horizon] because the
# evaluation is censored. Hence dloglik/dλ_b = n_break/λ_b − E_run and
# dloglik/dλ_r = n_repair/λ_r − E_down. The exposures come from replaying the
# record's firing sequence: every machine starts running, Breakdown flips one
# machine down, Repair flips it back.
function analytic_score(θ, record, machine_cnt)
    n_running = machine_cnt
    n_break = 0
    n_repair = 0
    e_run = 0.0
    e_down = 0.0
    prev = 0.0
    for (ck, when) in record.firings
        e_run += n_running * (when - prev)
        e_down += (machine_cnt - n_running) * (when - prev)
        if ck[1] == :Breakdown
            n_running -= 1
            n_break += 1
        elseif ck[1] == :Repair
            n_running += 1
            n_repair += 1
        else
            error("unexpected clock key $ck")
        end
        prev = when
    end
    # The censoring tail: exposure continues from the last firing to the horizon.
    e_run += n_running * (record.horizon - prev)
    e_down += (machine_cnt - n_running) * (record.horizon - prev)
    return [n_break / θ[1] - e_run, n_repair / θ[2] - e_down]
end

@testset "repairshop: the recorded trajectory is deterministic, draw-free, and long enough to test" begin
    θ = [0.3, 1.0]
    (; record, policy) = RepairShop.record_repair_shop(θ; machine_cnt=3, horizon=20.0, seed=90210)
    # fire! bodies draw no randomness, so the firing sequence is a deterministic
    # function of the initial state and the record is trustworthy for replay.
    @test record.fire_random == false
    @test record.horizon == 20.0
    # Enough events that the score test below is not vacuous.
    @test length(record.firings) > 10
    # The same seed reproduces the identical record (master-seed determinism).
    again = RepairShop.record_repair_shop(θ; machine_cnt=3, horizon=20.0, seed=90210)
    @test again.record.firings == record.firings
end

@testset "repairshop: effect_check replays the record to the exact forward log-likelihood" begin
    θ = [0.3, 1.0]
    (; record, policy) = RepairShop.record_repair_shop(θ; machine_cnt=3, horizon=20.0, seed=90210)
    factory = () -> SimulationFSM(
        RepairShop.Shop(3), [RepairShop.Breakdown, RepairShop.Repair];
        seed=1, sampler=NextReactionMethod(), key_type=Tuple,
        step_likelihood=true, params=θ,
    )
    res = ChronoSim.effect_check(factory, RepairShop.init!, policy)
    @test res.applicable
    @test res.passed
    # Exact Float64 identity: forward accumulation and replay share a code path.
    @test res.forward === res.replay
    @test isfinite(res.forward)
end

@testset "repairshop: the ForwardDiff score of the censored trace matches the analytic exponential score" begin
    θ = [0.3, 1.0]
    machine_cnt = 3
    (; record, policy) = RepairShop.record_repair_shop(
        θ; machine_cnt=machine_cnt, horizon=20.0, seed=90210
    )
    g = RepairShop.repair_shop_score(θ, record; machine_cnt=machine_cnt)
    oracle = analytic_score(θ, record, machine_cnt)
    @test isapprox(g, oracle; rtol=1e-9)
    # The score is also correct away from the generating θ — the trace is fixed,
    # only the evaluation parameters move. This is the property estimators lean on.
    θ2 = [0.5, 0.7]
    g2 = RepairShop.repair_shop_score(θ2, record; machine_cnt=machine_cnt)
    oracle2 = analytic_score(θ2, record, machine_cnt)
    @test isapprox(g2, oracle2; rtol=1e-9)
end
