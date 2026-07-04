using Gen
  using Distributions

  # Assume this function exists
  # landspread_log_likelihood([(event1, time1), (event2, time2),...]) -> Float64

  @gen function landspread_model(observations, max_time::Float64, n_points::Int)
      # Prior on number of events
      n_events ~ poisson(3.0)  # Expected 3 spread events

      # Generate variable number of events
      events = []
      for i in 1:n_events
          # Sample source point (1 to n_points)
          source ~ categorical(fill(1.0/n_points, n_points)) |> (s -> Int(s))

          # Sample destination point (different from source) 
          dest_weights = fill(1.0/(n_points-1), n_points)
          dest_weights[source] = 0.0
          dest ~ categorical(dest_weights) |> (d -> Int(d))

          # Sample event time uniformly over simulation period
          time ~ uniform(0.0, max_time)

          push!(events, (source, dest, time))

          # Address each variable uniquely
          {(:source, i)} ~ dirac(source)
          {(:dest, i)} ~ dirac(dest)
          {(:time, i)} ~ dirac(time)
      end

      # Sort events by time
      events = sort(events, by=x -> x[3])

      # Score the event sequence using the likelihood function
      log_likelihood = landspread_log_likelihood(events)
      {(:log_likelihood)} ~ dirac(log_likelihood)

      return (n_events, events, log_likelihood)
  end

  # Birth move: add a new event
  @gen function birth_proposal(trace, n_points::Int, max_time::Float64)
      current_n_events = trace[:n_events]
      new_n_events = current_n_events + 1

      # Sample new event parameters
      new_source ~ categorical(fill(1.0/n_points, n_points)) |> (s -> Int(s))
      dest_weights = fill(1.0/(n_points-1), n_points)
      dest_weights[new_source] = 0.0
      new_dest ~ categorical(dest_weights) |> (d -> Int(d))
      new_time ~ uniform(0.0, max_time)

      return (new_n_events, new_source, new_dest, new_time)
  end

  # Death move: remove an existing event  
  @gen function death_proposal(trace)
      current_n_events = trace[:n_events]
      if current_n_events == 0
          return (0, nothing)
      end

      # Choose which event to remove
      event_to_remove ~ categorical(fill(1.0/current_n_events, current_n_events))
      new_n_events = current_n_events - 1

      return (new_n_events, event_to_remove)
  end

  # Time shift move: change timing of existing event
  @gen function time_shift_proposal(trace, max_time::Float64)
      current_n_events = trace[:n_events]
      if current_n_events == 0
          return (nothing, nothing)
      end

      # Choose which event to shift
      event_to_shift ~ categorical(fill(1.0/current_n_events, current_n_events))

      # Sample new time
      new_time ~ uniform(0.0, max_time)

      return (event_to_shift, new_time)
  end

  # Custom RJMCMC update using Gen's involution framework
  function rjmcmc_update(trace, observations, n_points, max_time)
      # Choose move type
      move_type = rand(["birth", "death", "time_shift"])

      if move_type == "birth"
          # Birth move
          (new_trace, weight, _) = metropolis_hastings(
              trace,
              birth_proposal,
              (n_points, max_time)
          )

      elseif move_type == "death"
          # Death move  
          (new_trace, weight, _) = metropolis_hastings(
              trace,
              death_proposal,
              ()
          )

      elseif move_type == "time_shift"
          # Time shift move
          (new_trace, weight, _) = metropolis_hastings(
              trace,
              time_shift_proposal,
              (max_time,)
          )
      end

      return new_trace
  end

  # Main inference function
  function infer_landspread_events(observations, n_points::Int, max_time::Float64;
                                  n_iterations::Int=10000, n_chains::Int=3)

      # Convert observations to constraints
      constraints = choicemap()
      # Note: You'd need to add constraints based on observations
      # This would constrain the model to be consistent with observed infections

      # Initialize trace
      (initial_trace, _) = generate(landspread_model, (observations, max_time, n_points), constraints)

      # Run RJMCMC chains
      all_traces = []

      for chain in 1:n_chains
          trace = initial_trace
          chain_traces = [trace]

          for iter in 1:n_iterations
              # Standard MH moves on existing variables
              trace = metropolis_hastings(trace, select(:n_events))[1]

              # RJMCMC moves
              trace = rjmcmc_update(trace, observations, n_points, max_time)

              if iter % 100 == 0  # Collect every 100th sample
                  push!(chain_traces, trace)
              end
          end

          append!(all_traces, chain_traces[end-50:end])  # Last 50 samples from each chain
      end

      # Extract event sequences from traces
      event_sequences = []
      for trace in all_traces
          (n_events, events, log_likelihood) = get_retval(trace)
          push!(event_sequences, (events, log_likelihood))
      end

      return event_sequences
  end

  # Example usage
  function main()
      # Partial observations: (point_id, time, state)
      observations = [
          (1, 0.0, 1),   # Point 1 infected at time 0 (source)
          (3, 2.5, 1),   # Point 3 infected at time 2.5  
          (5, 4.0, 0),   # Point 5 still uninfected at time 4.0
          (7, 6.0, 1),   # Point 7 infected by time 6.0
      ]

      n_points = 10
      max_time = 10.0

      # Run inference
      event_sequences = infer_landspread_events(
          observations,
          n_points,
          max_time,
          n_iterations=5000,
          n_chains=3
      )

      # Analyze results
      println("Inferred $(length(event_sequences)) possible event sequences:")

      # Group by likelihood and show most probable sequences
      sorted_sequences = sort(event_sequences, by=x -> x[2], rev=true)

      for (i, (events, ll)) in enumerate(sorted_sequences[1:min(10, length(sorted_sequences))])
          println("\nSequence $i (log-likelihood: $(round(ll, digits=3))):")
          for (src, dest, time) in events
              println("  Point $src → Point $dest at time $(round(time, digits=2))")
          end
      end

      return event_sequences
  end

  # Run the inference
  event_distribution = main()

  
 Generative Model:
  - Variable number of events (Poisson prior)
  - Each event: source → destination at time t
  - Uses your landspread_log_likelihood to score sequences

  RJMCMC Moves:
  - Birth: Add new spread event
  - Death: Remove existing event
  - Time shift: Change event timing

  Integration:
  - Takes partial observations: [(point_id, time, infected_state), ...]
  - Returns distribution over possible event sequences
  - Sorts by likelihood to find most probable explanations

  Usage:
  observations = [
      (1, 0.0, 1),   # Point 1 infected at start
      (3, 2.5, 1),   # Point 3 infected by time 2.5
      (5, 4.0, 0),   # Point 5 still uninfected at time 4.0
  ]

  event_sequences = infer_landspread_events(observations, 10, 10.0)
