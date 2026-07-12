# Changelog

All notable changes to this project will be documented in this file. The
format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
the project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased] — 0.1.0

First coherent cut of the workspace, after a full safety/API review. Not yet
published to crates.io.

### Added

- CP-SAT (`oxidor-cpsat`): model builder with typed handles and
  operator-overloaded linear expressions; solve with default parameters, a
  first-class time limit (`solve_with_time_limit`), or full `SatParameters`;
  interruptible solves (`StopToken`/`Stopper`); solution enumeration;
  interval/no-overlap/cumulative scheduling constraints;
  `add_max_equality`/`add_min_equality` for fairness objectives.
- CP-SAT, full constraint catalog: `element`, allowed/forbidden assignment
  tables, `circuit`/`multiple_circuit`, automaton, reservoir (with optional
  events), `no_overlap_2d`, integer product/division/modulo, absolute value,
  `bool_xor`, inverse permutations — plus solution hints and assumptions.
- CP-SAT streaming solution callbacks (`callbacks` feature):
  `solve_with_solution_callback` observes every feasible solution as the
  search finds it; `ControlFlow::Break` stops the search early, and callback
  panics resume safely on the calling thread after the solver winds down.
- CP-SAT raw-proto solve (`solve_model_proto`,
  `solve_model_proto_interruptible`): the write side of the wire-level
  escape hatch for hand-built or hand-modified `CpModelProto`s.
- MathOpt (`oxidor-mathopt`): LP/MIP model builder; per-call solver choice
  (Glop, SCIP, CP-SAT, PDLP); cross-thread interruption
  (`SolveInterrupter`); `primal_solution()` returns only solver-certified
  feasible solutions.
- MathOpt per-solve parameters: `solve_with_parameters`
  (`SolveParametersProto` — limits, gap tolerances, threads, seed),
  `solve_with_time_limit`, and `registered_solvers()` to probe which
  backends the linked library ships; an unregistered solver is a clean
  `SolveError`. Solves now route through Oxidor's C shim (the upstream
  MathOpt C API takes no per-solve parameters).
- Routing (`oxidor-routing`): TSP and capacitated VRP over a distance
  matrix through an exception-safe C++ shim; search parameters as protos;
  `solve_with_time_limit`.
- Routing, richer problems: `TimeDimension` (travel times, service times,
  per-node time windows, waiting) with per-stop arrival times in the
  solution; pickup-and-delivery pairs (same vehicle, pickup first);
  per-vehicle fixed costs.
- Algorithms (`oxidor-algorithms`): 0-1 knapsack (multi-dimensional branch
  and bound), max flow, min cost flow, linear sum assignment
  (`LinearSumAssignment` — minimum-cost perfect matching).
- Distribution: `download-prebuilt` — a SHA-256-pinned static OR-Tools
  bundle fetched from this project's releases at build time (three targets),
  end-to-end tested in CI; `ORTOOLS_PREFIX` dynamic linking; pure-Rust model
  building with `--no-default-features`.

### Safety and correctness (post-review hardening)

- Every input contract upstream merely `CHECK`s (or silently miscomputes on)
  is validated on the Rust side: negative flow capacities, negative routing
  demands/capacities/costs, node indices beyond `i32`, buffer lengths beyond
  `c_int`.
- Variable/interval handles are branded with their model's identity;
  cross-model misuse panics instead of silently corrupting a model or
  reading the wrong answer.
- One CP-SAT solve per stop environment is enforced at compile time
  (`&mut StopToken`; only `Stopper` clones cross threads).
- Native output buffers are freed before decoding (no leak on a decode
  panic) and null-guarded.
