# The estimator family on the repair shop, each against the closed-form
# oracle E[#down(T)] = n·(λ/(λ+μ))·(1 − exp(−(λ+μ)T)). Statistical acceptance
# follows the WorldTimer convention: fixed seeds, agreement within four
# standard errors, and a standard-error smallness guard so the comparison
# bites.

using Test
using ChronoSimExamples.RepairShopGradients
using ChronoSimExamples.RepairShopGradients: ndown_twin, ShopLawState

const _RSG_θ = [0.3, 1.0]
const _RSG_N = 3
const _RSG_T = 8.0

@testset "repairshop gradients: the closed-form oracle matches a direct check at the test parameters" begin
    # n·(λ/(λ+μ))·(1−e^{−(λ+μ)T}) differentiated by hand at λ=0.3, μ=1.0:
    # cross-check the ForwardDiff oracle against finite differences.
    g = expected_down_gradient(_RSG_θ, _RSG_N, _RSG_T)
    h = 1e-6
    for j in 1:2
        θp = copy(_RSG_θ); θp[j] += h
        θm = copy(_RSG_θ); θm[j] -= h
        fd = (expected_down(θp, _RSG_N, _RSG_T) - expected_down(θm, _RSG_N, _RSG_T)) / 2h
        @test abs(g[j] - fd) < 1e-6
    end
end

@testset "repairshop gradients: the pairing flags the pathwise zero and the score recovers the oracle" begin
    oracle = expected_down_gradient(_RSG_θ, _RSG_N, _RSG_T)
    pair = repair_shop_pairing(_RSG_θ)
    for j in 1:2
        @test pair.ipa[j] == 0.0                       # frozen discrete read
        @test abs(pair.score[j] - oracle[j]) < 4 * pair.score_stderr[j]
        @test pair.score_stderr[j] < abs(oracle[j]) / 5
    end
    @test any(pair.bias_detected)
end

@testset "repairshop gradients: branching through the live ChronoSim simulation matches the oracle" begin
    oracle = expected_down_gradient(_RSG_θ, _RSG_N, _RSG_T)
    br = repair_shop_branching(_RSG_θ)
    for j in 1:2
        @test abs(br.estimate[j] - oracle[j]) < 4 * br.stderr[j]
        @test br.stderr[j] < abs(oracle[j]) / 4
    end
end

@testset "repairshop gradients: SPA proves every swap harmless on independent machines and carries the derivative in its horizon term" begin
    oracle = expected_down_gradient(_RSG_θ, _RSG_N, _RSG_T)
    spa = repair_shop_spa(_RSG_θ)
    for j in 1:2
        @test abs(spa.estimate[j] - oracle[j]) < 4 * spa.stderr[j]
        @test spa.stderr[j] < abs(oracle[j]) / 5
    end
    # Independent machines: the criticality gate certifies every candidate
    # pair, no clone is ever spawned, and the pathwise part is identically
    # zero — the whole estimate is horizon crossings.
    @test spa.skip_fraction == 1.0
    @test spa.clones_per_rep == 0.0
    @test spa.ipa_part == [0.0, 0.0]
end
