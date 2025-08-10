using Logging

@testset "Elevator smoke" begin
    using ChronoSimExamples.ElevatorExample
    with_logger(ConsoleLogger(stderr, Logging.Debug)) do
        run_duration = ElevatorExample.run_elevator()
        @assert run_duration > 9.9
    end
end

@testset "Elevator tlaplus" begin
    using ChronoSimExamples.ElevatorExample
    with_logger(ConsoleLogger(stderr, Logging.Info)) do
        ElevatorExample.run_with_trace()
    end
end
