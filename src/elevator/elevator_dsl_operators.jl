# Using Julia's built-in |> (pipe) operator for event chaining

struct EventStep
    data
    step_type::Symbol  # :condition, :timing, :action
end

# Helper functions that work with pipe operator
when(condition) = EventStep(condition, :condition)
wait(distribution) = prev -> EventStep((prev, distribution), :timing)
perform(action) = prev -> EventStep((prev, action), :action)

# Usage example with pipe operator:
# when(condition) |> wait(distribution) |> perform(action)

# Example of how it chains:
# event = when(!person.waiting) |> 
#         wait(Exponential(1.0)) |> 
#         perform() do system, person
#             person.waiting = true
#         end

# Or use Julia's do syntax more idiomatically:
@event CallElevator(person) begin
    @condition !person.waiting && person.location != person.destination
    @timing Exponential(1.0)
    @action begin
        person.waiting = true
        direction = person.destination > person.location ? Up : Down
        calls[(person.location, direction)].requested = true
    end
end