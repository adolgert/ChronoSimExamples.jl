---- MODULE ElevatorTrace ----
(* This specification defines a specific trace to be checked against the Elevator spec *)
EXTENDS Sequences, TLC, Integers

(* Import constants from Elevator *)
CONSTANTS   Person,     \* The set of all people using the elevator system
            Elevator,   \* The set of all elevators
            FloorCount  \* The number of floors serviced by the elevator system

(* Define Floor and Direction for invariant checking *)
Floor == 1..FloorCount
Direction == {"Up", "Down"}

(* The recorded trace as a sequence of states *)
TraceStates == <<
    (* Initial state *)
    [
        PersonState |-> [
            p3 |-> [location |-> 2, destination |-> 2, waiting |-> FALSE],
            p2 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
            p1 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE]
        ],
        ActiveElevatorCalls |-> {},
        ElevatorState |-> [
            e2 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}],
            e1 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}]
        ]
    ],
    (* After PickNewDestination(p3) *)
    [
        PersonState |-> [
            p3 |-> [location |-> 2, destination |-> 5, waiting |-> FALSE],
            p2 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
            p1 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE]
        ],
        ActiveElevatorCalls |-> {},
        ElevatorState |-> [
            e2 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}],
            e1 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}]
        ]
    ]
>>

(* Dummy variables for TLC - we're just checking a static trace *)
VARIABLES dummy

(* Dummy initial state *)
Init == dummy = 0

(* Dummy next state - never actually used *)
Next == dummy' = dummy

(* Type invariant: check structural constraints *)
TraceTypeInvariant == 
    \A i \in 1..Len(TraceStates) :
        LET s == TraceStates[i] IN
        /\ \A p \in Person : 
            /\ \/ s.PersonState[p].location \in Floor
               \/ s.PersonState[p].location \in {"e1", "e2"}  \* In an elevator
            /\ s.PersonState[p].destination \in Floor
            /\ s.PersonState[p].waiting \in BOOLEAN
        /\ \A e \in Elevator :
            /\ s.ElevatorState[e].floor \in Floor
            /\ s.ElevatorState[e].direction \in (Direction \cup {"Stationary"})
            /\ s.ElevatorState[e].doorsOpen \in BOOLEAN
            /\ s.ElevatorState[e].buttonsPressed \subseteq Floor

(* Safety invariant: check semantic constraints *)
TraceSafetyInvariant == 
    \A i \in 1..Len(TraceStates) :
        LET s == TraceStates[i] IN
        (* Elevator buttons pressed only if passenger going there *)
        /\ \A e \in Elevator : \A f \in s.ElevatorState[e].buttonsPressed :
            \E p \in Person :
                /\ s.PersonState[p].location = e  \* In elevator e
                /\ s.PersonState[p].destination = f
        (* Active calls must have someone waiting *)
        /\ \A call \in s.ActiveElevatorCalls :
            \E p \in Person :
                LET pState == s.PersonState[p] IN
                /\ pState.location = call.floor
                /\ pState.waiting = TRUE
                /\ (IF pState.destination > call.floor 
                    THEN "Up" ELSE "Down") = call.direction

=============================================================================
