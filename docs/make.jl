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
)

deploydocs(;
    repo="github.com/adolgert/ChronoSimExamples.jl",
    devbranch="main",
)
