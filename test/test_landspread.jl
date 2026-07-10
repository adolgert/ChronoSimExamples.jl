# Landspread previously had no functional test, which is how a silent
# no-write bug (an @obswrite assignment whose right-hand side the macro
# discarded) left it running zero events without anyone noticing. These tests
# pin that the process actually spreads and that the θ seam evaluates the same
# trace at different parameters.

using Logging
using Test

using ChronoSimExamples: LandSpread

@testset "landspread spreads from the seed point to every point" begin
    traj = with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        LandSpread.run_landspread(10)
    end
    # One InitializeEvent plus one Spread firing per remaining point: the
    # process is monotone (marks never clear), so it must saturate.
    @test length(traj) == 10
    @test count(te -> te.event[1] == :Spread, traj) == 9
end

@testset "landspread trace log-likelihood is finite and moves with θ through the seam" begin
    ll, ll_shifted = with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        (LandSpread.landspread_likelihood(10),
         LandSpread.landspread_likelihood(10; θ=[0.2, 1.0]))
    end
    @test isfinite(ll)
    @test isfinite(ll_shifted)
    # Same recorded trace, different θ: the evaluation must respond to the
    # parameters, which is the θ-seam property estimators rely on.
    @test ll != ll_shifted
end
