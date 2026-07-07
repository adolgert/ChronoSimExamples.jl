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
