using ChronoSimExamples
using Documenter

DocMeta.setdocmeta!(ChronoSimExamples, :DocTestSetup, :(using ChronoSimExamples); recursive=true)

makedocs(;
    modules=[ChronoSimExamples],
    authors="Andrew Dolgert <github@dolgert.com>",
    sitename="ChronoSimExamples.jl",
    format=Documenter.HTML(;
        canonical="https://adolgert.github.io/ChronoSimExamples.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Elevator" => [
            "The Model" => "models/elevator/model.md",
            "Running It" => "models/elevator/running.md",
            "Debugging Tools" => "models/elevator/debugging.md",
        ],
        "Reliability" => [
            "The Model" => "models/reliability/model.md",
            "Running It" => "models/reliability/usage.md",
        ],
        "Repair Shop" => [
            "The Model" => "models/repairshop/model.md",
            "The Likelihood Workflow" => "models/repairshop/likelihood.md",
            "Gradient Estimators" => "models/repairshop/gradients.md",
        ],
        "Landspread" => [
            "The Model" => "models/landspread/model.md",
            "Running It" => "models/landspread/usage.md",
        ],
        "SIR Village" => [
            "The Model" => "models/sirvillage/model.md",
            "Running It" => "models/sirvillage/usage.md",
            "Characterizing Travel" => "dwell.md",
        ],
        "Other Simulations" => "possible.md",
    ],
    # The example models live in submodules (ElevatorExample, SIRVillage, ...)
    # whose per-event docstrings are not meant to be enumerated in the manual;
    # the top-level @autodocs block covers the package's own bindings. Keep the
    # missing-docstring check as a warning rather than a hard build failure.
    warnonly=[:missing_docs],
)

deploydocs(;
    repo="github.com/adolgert/ChronoSimExamples.jl",
    devbranch="main",
)
