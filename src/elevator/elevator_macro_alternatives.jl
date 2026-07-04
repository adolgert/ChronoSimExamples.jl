# Alternative syntaxes for @reactto macro

# Current syntax - unclear that 'system' is the physical state parameter
@reactto changed(person[who].location) begin
    system
    generate(PickNewDestination(who))
end

# Option 1: Do-block style (most Julian)
@reactto changed(person[who].location) do system
    generate(PickNewDestination(who))
end

# Option 2: Parameter syntax
@reactto changed(person[who].location) (system) begin
    generate(PickNewDestination(who))
end

# Option 3: Arrow syntax
@reactto changed(person[who].location) => system begin
    generate(PickNewDestination(who))
end

# Option 4: With clause
@reactto changed(person[who].location) with system begin
    generate(PickNewDestination(who))
end

# Option 5: As clause
@reactto changed(person[who].location) as system begin
    generate(PickNewDestination(who))
end

# Option 6: Let-style binding
@reactto changed(person[who].location) let system
    generate(PickNewDestination(who))
end

# For comparison with actual Julia patterns:

# Julia's do-block pattern:
# map(collection) do item
#     process(item)
# end

# Julia's let binding:
# let x = value
#     use(x)
# end

# Julia's where clause (for types):
# function foo(x::T) where T
#     ...
# end