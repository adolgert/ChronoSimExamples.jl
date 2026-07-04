# Example DSL for more compact elevator definition

@event PickNewDestination(person::Int64) begin
    @triggers
        person[who].location => generate(who)
    
    @precondition
        !person.waiting && person.location == person.destination
    
    @timing
        Exponential(1.0)  # Could be any complex distribution
    
    @action
        dests = setdiff(1:system.floor_cnt, person.location)
        person.destination = rand(rng, dests)
end

@event CallElevator(person::Int64) begin
    @triggers
        person[who].waiting => generate(who)
    
    @precondition
        person.location != person.destination && !person.waiting
    
    @timing
        Exponential(1.0)
    
    @action
        person.waiting = true
        direction = person.destination > person.location ? Up : Down
        calls[(person.location, direction)].requested = true
end

@event OpenElevatorDoor(elevator_idx::Int64) begin
    @triggers
        elevator[elidx].floor => generate(elidx)
        elevator[elidx].buttons_pressed => generate(elidx)
        calls[callkey].requested => foreach(1:length(elevator), generate)
    
    @precondition
        !elevator.doorsOpen && 
        (haskey(calls, (elevator.floor, elevator.direction)) && 
         calls[(elevator.floor, elevator.direction)].requested ||
         elevator.floor ∈ elevator.buttons_pressed)
    
    @timing
        Exponential(1.0)
    
    @action
        elevator.doorsOpen = true
        elevator.buttons_pressed = setdiff(elevator.buttons_pressed, elevator.floor)
        if haskey(calls, (elevator.floor, elevator.direction))
            calls[(elevator.floor, elevator.direction)].requested = false
        end
end

# Alternative: State machine DSL
@statemachine Elevator begin
    @states [Idle, Moving(direction), DoorsOpen, DoorsClosing]
    
    @transitions begin
        Idle + CallReceived => Moving(toward_call)
        Moving + ArrivedAtFloor[should_stop] => DoorsOpen
        DoorsOpen + NoPeopleWaiting => DoorsClosing
        DoorsClosing + DoorsClosed => Idle | Moving(continue_direction)
    end
    
    @timing
        Moving => Exponential(floor_travel_time)
        DoorsOpen => Exponential(door_wait_time)
        DoorsClosing => Exponential(door_close_time)
end

# Or a more functional style:
elevator_events = @events begin
    PickNewDestination(person) = 
        when(person.at_destination && !person.waiting) >>
        wait(Exponential(1.0)) >>
        set!(person.destination, random_floor_except(person.location))
    
    CallElevator(person) = 
        when(person.needs_elevator && !person.waiting) >>
        wait(Exponential(1.0)) >>
        do
            person.waiting = true
            request_call(person.location, person.direction)
        end
    
    OpenDoors(elevator) =
        when(elevator.at_requested_floor && !elevator.doorsOpen) >>
        wait(Exponential(1.0)) >>
        do
            elevator.doorsOpen = true
            clear_call(elevator.floor, elevator.direction)
            clear_button(elevator.floor)
        end
end