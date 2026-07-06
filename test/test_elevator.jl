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

# The bespoke TLA+/TLC trace validation (ElevatorExample.run_with_trace, retired to
# attic/elevatortla.jl in Phase 4) is superseded by the generic model-checking path in
# test_quint.jl, which compiles the elevator to Quint and validates a recorded
# trajectory against it. That removes the tla2tools.jar shell-out (never present on CI)
# and its ProcessFailedException noise.
