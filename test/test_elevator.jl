using Logging

@testset "Elevator smoke" begin
    using ChronoSimExamples.ElevatorExample
    using ChronoSim: CheckInvariants
    # Debug policy on in tests (off by default in run_elevator): the CI smoke run
    # checks every declared invariant after every fired event.
    with_logger(ConsoleLogger(stderr, Logging.Debug)) do
        run_duration = ElevatorExample.run_elevator(policy=CheckInvariants(ElevatorExample))
        @assert run_duration > 9.9
    end
end

@testset "Elevator tlaplus" begin
    using ChronoSimExamples.ElevatorExample
    # TLC validation shells out to a locally-installed tla2tools.jar, which CI
    # runners do not have; the trace machinery is exercised only locally.
    if continuous_integration()
        @info "Skipping TLA+ trace validation on CI (no tla2tools.jar)"
    else
        with_logger(ConsoleLogger(stderr, Logging.Info)) do
            ElevatorExample.run_with_trace()
        end
    end
end
