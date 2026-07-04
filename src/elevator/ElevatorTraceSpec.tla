---------------------------- MODULE ElevatorTraceSpec ----------------------------
(* This specification defines a specific trace to be checked against the Elevator spec *)
EXTENDS Elevator, TLC

(* Import the constants from the original spec *)
CONSTANTS Person, Elevator, FloorCount

(* Define the trace as a sequence of states *)
TraceStates == <<
    (* State 1 - Initial *)
    [PersonState |-> [p1 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
                     p2 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
                     p3 |-> [location |-> 2, destination |-> 2, waiting |-> FALSE]],
     ActiveElevatorCalls |-> {},
     ElevatorState |-> [e1 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}],
                       e2 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}]]],
    
    (* State 2 - After PickNewDestination(p3) *)
    [PersonState |-> [p1 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
                     p2 |-> [location |-> 3, destination |-> 3, waiting |-> FALSE],
                     p3 |-> [location |-> 2, destination |-> 5, waiting |-> FALSE]],
     ActiveElevatorCalls |-> {},
     ElevatorState |-> [e1 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}],
                       e2 |-> [floor |-> 1, direction |-> "Stationary", doorsOpen |-> FALSE, buttonsPressed |-> {}]]]
>>

(* Define variables to track our position in the trace *)
VARIABLES traceIndex

(* Initial state *)
TraceInit == 
    /\ traceIndex = 1
    /\ PersonState = TraceStates[1].PersonState
    /\ ActiveElevatorCalls = TraceStates[1].ActiveElevatorCalls
    /\ ElevatorState = TraceStates[1].ElevatorState

(* Next state relation - move to the next state in the trace *)
TraceNext ==
    /\ traceIndex < Len(TraceStates)
    /\ traceIndex' = traceIndex + 1
    /\ PersonState' = TraceStates[traceIndex'].PersonState
    /\ ActiveElevatorCalls' = TraceStates[traceIndex'].ActiveElevatorCalls
    /\ ElevatorState' = TraceStates[traceIndex'].ElevatorState

(* The trace specification *)
TraceSpec == TraceInit /\ [][TraceNext]_<<PersonState, ActiveElevatorCalls, ElevatorState, traceIndex>>

(* Theorem: The trace satisfies the original specification's safety properties *)
THEOREM TraceSpec => [](TypeInvariant /\ SafetyInvariant)

=============================================================================