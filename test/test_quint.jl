# Phase 4: compile the example models to Quint and validate recorded trajectories.
#
# Emission tests (pure Julia) always run. Checker tests are gated on the pinned
# toolchain (`CHRONOSIM_QUINT` -> quint; `JAVA_HOME`/PATH -> Apalache): absent, they
# skip loudly (never a silent pass, never a hard failure in the core suite).

using ChronoSim
using ChronoSim: compile_quint, validate_trace, QuintCompileError, find_quint_toolchain
using Random
using CompetingClocks: NextReactionMethod
import ChronoSim

const _QTC = find_quint_toolchain()
const _HAVE_QUINT = ChronoSim.quint_available(_QTC)
const _HAVE_JAVA = ChronoSim.java_available(_QTC)

_qwork(name) = mktempdir(; prefix="quint_$(name)_")

# ---- event lists / factories ---------------------------------------------------

module _QElev
    import ..ChronoSimExamples
    const E = ChronoSimExamples.ElevatorDerivedExample
    const H = ChronoSimExamples.ElevatorExample
    const EVENTS = [E.PickNewDestination, E.CallElevator, E.OpenElevatorDoors, E.EnterElevator,
        E.ExitElevator, E.CloseElevatorDoors, E.MoveElevator, E.StopElevator, E.DispatchElevator]
end

@testset "quint: elevator compiles clean" begin
    E = _QElev.E
    qm = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
        invariants=_QElev.H)
    rep = qm.report
    @test all(e -> e.status in (:clean, :widened), rep.events)          # no refusals
    @test count(e -> e.status === :clean, rep.events) == 9
    @test isempty(rep.widenings)                                        # D8 removes the divergence
    @test :floor_cnt in rep.promoted                                   # D6
    @test (:ElevatorCall => :requested) in rep.collapsed               # D7
    @test count(i -> i.status === :clean, rep.invariants) == 8
    # widening-marker reconciliation (zero silent widenings)
    @test count(_ -> true, eachmatch(r"// WIDENED", qm.text)) == length(rep.widenings)
    # deterministic emission (recompile is byte-identical)
    qm2 = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
        invariants=_QElev.H)
    @test qm.text == qm2.text
end

@testset "quint: reliability compiles (StartDay assumed, floats erased)" begin
    R = ChronoSimExamples.ReliabilityDerivedSim
    qm = compile_quint(R, [R.StartDay, R.EndDay, R.Break, R.Repair], R.IndividualState(5, 3);
        name="reliability", assume_true_guards=[:StartDay])
    rep = qm.report
    @test any(e -> e.name === :StartDay && e.status === :assumed_true_guard, rep.events)
    @test all(e -> e.status in (:clean, :widened, :assumed_true_guard), rep.events)
    @test any(occursin("work_age", e) for e in rep.erased)
    @test any(occursin("started_working_time", e) for e in rep.erased)
end

@testset "quint: sirvillage compiles partial (float invariant refused)" begin
    S = ChronoSimExamples.SIRVillageDerived
    qm = compile_quint(S, [S.InitEvent, S.Travel, S.Infect, S.Recover, S.Reset, S.Mutate],
        S.Village(10, 10, 1.0, Xoshiro(1)); name="sirvillage",
        assume_true_guards=[:Travel], skip_events=[:InitEvent, :Travel, :Infect, :Mutate],
        invariants=ChronoSimExamples.SIRVillage)
    rep = qm.report
    @test qm.partial
    # D11: a PARTIAL module says so in its header, naming every skipped event
    @test occursin("// PARTIAL: skipped events (skip_events): InitEvent Travel Infect Mutate",
        qm.text)
    @test any(e -> e.name === :Recover && e.status === :clean, rep.events)
    @test any(e -> e.name === :Reset && e.status === :clean, rep.events)
    @test any(i -> i.name == "strain rates nonnegative" && i.status === :refused, rep.invariants)
end

# ---- checker-gated tests -------------------------------------------------------

@testset "quint: golden typecheck + run (elevator)" begin
    if !_HAVE_QUINT
        @info "skipping quint typecheck/run: no toolchain (set CHRONOSIM_QUINT)"
        @test_skip _HAVE_QUINT
    else
        E = _QElev.E
        qm = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
            invariants=_QElev.H)
        wd = _qwork("golden")
        path = ChronoSim.write_quint(joinpath(wd, "elevator.qnt"), qm)
        log = joinpath(wd, "tc.log")
        @test ChronoSim._run_quint(_QTC, String["typecheck", path], log; dir=wd) === :ok
        @test !occursin("error", lowercase(read(log, String)))
        # randomized run: no invariant violation (mirrors RESULTS.md §2)
        runlog = joinpath(wd, "run.log")
        ChronoSim._run_quint(_QTC, String["run", "--max-steps=200", "--max-samples=2000",
            "--seed=42", "--invariant=inv", path], runlog; dir=wd)
        @test ChronoSim._run_passed(runlog)
    end
end

# One recorded elevator factory reused by the trace + mutation tests.
function _elev_factory(policy)
    E = _QElev.E
    phys = E.ElevatorSystem(3, 2, 5)
    sim = SimulationFSM(phys, _QElev.EVENTS; sampler=NextReactionMethod(), key_type=Tuple,
        rng=Xoshiro(93472934), policy=policy)
    (sim, E.init_physical)
end

function _record_elev(nsteps)
    rec = RecordSkeleton()
    sim, init = _elev_factory(rec)
    ChronoSim.run(sim, init, (phys, step, evt, when) -> step >= nsteps)
    return recorded_skeleton(rec)
end

@testset "quint: trace validation passes (elevator) — elevatortla parity" begin
    if !_HAVE_QUINT
        @info "skipping trace validation: no quint toolchain"
        @test_skip _HAVE_QUINT
    else
        E = _QElev.E
        skel = _record_elev(5)
        # the convenience form: compile keywords route to compile_quint (incl.
        # `invariants`), checker keywords to the checker run
        rep = validate_trace(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5), skel,
            _elev_factory; name="elevator", invariants=_QElev.H, maxsteps=5,
            toolchain=_QTC, workdir=_qwork("trace"))
        @test rep.invariants === :passed
        @test rep.first_failure === nothing
        if _HAVE_JAVA
            @test rep.transitions === :passed   # every recorded transition accepted
        else
            @test rep.transitions === :skipped
        end
    end
end

@testset "quint: bounded Apalache verify (elevator inv)" begin
    if !(_HAVE_QUINT && _HAVE_JAVA)
        @info "skipping bounded Apalache verify: needs quint + a JVM"
        @test_skip _HAVE_QUINT && _HAVE_JAVA
    else
        # A shallow symbolic proof over the compiled module (~25 s at depth 5 on
        # the spike machine). The RESULTS.md full depths (10/8, ~10 min) are the
        # CI-budget follow-up recorded in the phase report.
        E = _QElev.E
        qm = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
            invariants=_QElev.H)
        wd = _qwork("verify")
        path = ChronoSim.write_quint(joinpath(wd, "elevator.qnt"), qm)
        log = joinpath(wd, "verify.log")
        r = ChronoSim._run_quint(_QTC, String["verify", "--max-steps=5",
            "--invariant=inv", path], log; dir=wd)
        @test r === :ok
        @test ChronoSim._apalache_outcome(log) == "NoError"
    end
end

@testset "quint: mutation makes trace validation fail" begin
    if !(_HAVE_QUINT && _HAVE_JAVA)
        @info "skipping mutation test: needs quint + a JVM (Apalache stage 2)"
        @test_skip _HAVE_QUINT && _HAVE_JAVA
    else
        E = _QElev.E
        skel = _record_elev(5)
        # flip the recorded CallElevator's `location != destination` guard to `==`
        qmm = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
            invariants=_QElev.H,
            mutate_for_test=(event=:CallElevator, from=:!=, to=:(==), occurrence=2))
        @test occursin("CallElevator", qmm.report.mutated)
        rep = validate_trace(qmm, skel, _elev_factory; maxsteps=5, toolchain=_QTC,
            workdir=_qwork("mutation"))
        @test rep.transitions === :failed
        # the ascending localization loop names the exact rejected transition:
        # the recorded CallElevator step (step 3 of this seed's trajectory)
        @test rep.first_failure !== nothing
        @test rep.first_failure.stage === :transitions
        @test rep.first_failure.event === :CallElevator
        @test rep.first_failure.step == 3
        # and the unmutated compile of the same skeleton passes
        qm = compile_quint(E, _QElev.EVENTS, E.ElevatorSystem(3, 2, 5); name="elevator",
            invariants=_QElev.H)
        rep2 = validate_trace(qm, skel, _elev_factory; maxsteps=5, toolchain=_QTC,
            workdir=_qwork("unmutated"))
        @test rep2.transitions === :passed
        @test rep2.first_failure === nothing
    end
end
