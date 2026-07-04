# Example of running elevator simulation with TLA+ trace recording

using ChronoSim
include("elevator.jl")
using .ElevatorExample
using CompetingClocks

# Create the simulation with TLA+ trace recorder
function run_elevator_with_tla_validation()
    # Initialize physical state
    physical = ElevatorSystem(
        person = ObservedVector{Person}(),
        calls = ObservedDict{Tuple{Int64,ElevatorDirection},Call}(),
        elevator = ObservedVector{Elevator}(),
        floor_cnt = 5,
        people_cnt = 3
    )
    
    # Add people
    push!(physical.person, Person(1, location=1, destination=1, waiting=false))
    push!(physical.person, Person(2, location=2, destination=2, waiting=false))
    push!(physical.person, Person(3, location=3, destination=3, waiting=false))
    
    # Add elevators
    push!(physical.elevator, Elevator(1, floor=1, direction=Stationary, doorsOpen=false, buttons_pressed=Set{Int64}()))
    push!(physical.elevator, Elevator(2, floor=3, direction=Stationary, doorsOpen=false, buttons_pressed=Set{Int64}()))
    
    # Create events list
    events = SimEvent[
        PickNewDestination(1),
        PickNewDestination(2),
        PickNewDestination(3),
        CallElevator(1),
        CallElevator(2),
        CallElevator(3),
        OpenElevatorDoor(1),
        OpenElevatorDoor(2),
        EnterElevator(1),
        EnterElevator(2),
        ExitElevator(1),
        ExitElevator(2),
        CloseElevatorDoors(1),
        CloseElevatorDoors(2),
        MoveElevator(1),
        MoveElevator(2),
        StopElevator(1),
        StopElevator(2),
        DispatchElevator(1, Up),
        DispatchElevator(2, Down),
    ]
    
    # Create TLA+ trace recorder
    recorder = TLATraceRecorder()
    
    # Create sampler
    sampler = CombinedNextReaction()
    
    # Create simulation with recorder as observer
    sim = SimulationFSM(physical, events; sampler=sampler, seed=42,
                        observer=(phys, when, evt, changed) -> recorder(sim, phys, when, evt, changed))
    
    # Initialize the simulation
    initialize!(sim) do phys
        # Initial state is already set up
    end
    
    # Run for a short time
    run(sim, phys -> nothing, (phys, step, evt, when) -> step > 20 || when > 10.0)
    
    # Export the trace
    export_tlc_trace(recorder, "elevator_trace.txt")
    
    # Create TLC config
    create_tlc_config(3, 2, 5, "elevator.cfg")
    
    # Check invariants
    println("Type invariant violations: ", validate_type_invariant(physical))
    println("Safety invariant violations: ", check_safety_invariant(physical))
    
    # Show final state and enabled events
    export_current_state(sim, physical, "elevator_final_state.txt")
    
    return recorder, sim, physical
end

# Run the example
recorder, sim, physical = run_elevator_with_tla_validation()

println("Simulation complete!")
println("States recorded: ", length(recorder.states))
println("Transitions recorded: ", length(recorder.transitions))
println("\nTrace exported to: elevator_trace.txt")
println("Config exported to: elevator.cfg")
println("Final state exported to: elevator_final_state.txt")
println("\nTo check with TLC, run:")
println("  java -cp tla2tools.jar tlc2.TLC -trace elevator_trace.txt -config elevator.cfg elevator.tla")
