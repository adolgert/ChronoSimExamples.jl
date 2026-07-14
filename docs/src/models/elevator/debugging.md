# Debugging with the Elevator

The elevator is the main fixture for ChronoSim's "why-verb" debugging tools,
tested in `test/test_whyverbs.jl`. Those tests are more involved than a smoke
run, and they double as worked examples of the questions you can ask a
simulation when it does not do what you expect: *why did this event never fire?*,
*why was it rejected?*, and *why did the run stop?*

The scenarios below use a larger configuration than `run_elevator` — three
people, two elevators, five floors — built directly rather than through the
entry point:

```julia
_elev_events(M) = [M.PickNewDestination, M.CallElevator, M.OpenElevatorDoors,
    M.EnterElevator, M.ExitElevator, M.CloseElevatorDoors, M.MoveElevator,
    M.StopElevator, M.DispatchElevator]

P, E, F, seed = 3, 2, 5, 93472934
sim = SimulationFSM(_ED.ElevatorSystem(P, E, F), _elev_events(_ED);
    sampler=NextReactionMethod(), key_type=Tuple, rng=Xoshiro(seed),
    policy=RecordSkeleton())
ChronoSim.run(sim, _ED.init_physical, (p, i, e, w) -> w > 120.0)
```

`RecordSkeleton` is a policy that records a lightweight trace of the run so the
diagnostic tools have a history to point at.

## `whynot`: an event that never fired

The first scenario asks why `StopElevator` was never even proposed. It uses a
deliberately broken fixture, `ElevatorStopBug` (in
`test/whyverbs_elevator_stopbug.jl`), a copy of the hand-written model with the
extra `StopElevator` triggers reverted — exactly the bug the derived twin once
exposed. Because `StopElevator` is missing a trigger, no state change ever
proposes it, and `whynot` reports it as `:never_proposed`, pointing at the
missing dependency.

## `whynot`: an event proposed but rejected

The second scenario uses `ElevatorNoButton` (in
`test/whyverbs_elevator_nobutton.jl`), a copy of the *derived* model with the
button-press clause deleted from `EnterElevator`'s precondition. Here the event
is proposed but its precondition returns false, so `whynot` reports `:rejected`
and breaks the precondition down clause by clause — showing that
`call_exists || button_pressed` failed because the button branch was removed.

The contrast between the two scenarios is the point: `:never_proposed` means the
framework never considered the event (a missing trigger), while `:rejected` means
it considered it and the precondition said no (a logic error). The debugging
tool distinguishes the two so you know whether to look at your triggers or your
precondition.

## `whystopped`: an invariant violation names the culprit

The last scenario shows how a violated invariant is traced back to the event that
caused it. A test-local `CorruptPerson` event is added that deliberately breaks
the `"person location xor elevator"` invariant. The run stacks two policies —
recording *and* invariant checking — so that when the invariant fails, the error
carries a replayable prefix:

```julia
policy = PolicyStack(RecordSkeleton(), CheckInvariants(ElevatorExample))
```

When the `CorruptPerson` event fires and the invariant fails, `ChronoSim.run`
throws an `InvariantViolation`. Passing that error to `whystopped` names the
guilty writer, the clock that fired, and the command to replay the run up to the
violation:

```julia
try
    ChronoSim.run(sim, _ED.init_physical, stop_condition)
catch err
    whystopped(err)   # names the event, the guilty address, and a replay command
end
```

## Why this lives in the examples

These debugging tools are ChronoSim features, not elevator features — but the
elevator is the model complex enough to demonstrate them honestly. Nine
interacting event types, precondition recursion, and a hand-vs-derived twin give
the tools real bugs to diagnose. The deliberately broken fixtures
(`ElevatorStopBug`, `ElevatorNoButton`) are full copies of the model with one
line changed, which is how you can see the tool point at exactly that line.
