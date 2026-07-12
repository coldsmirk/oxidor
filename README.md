# Oxidor

**Unofficial Rust bindings for [Google OR-Tools](https://developers.google.com/optimization).**

> Status: early development (0.1.x, **not yet published to crates.io** — use a
> git dependency until the first release lands). Four solver families work end
> to end — CP-SAT, MathOpt (LP/MIP), routing (TSP/VRP), and the classic
> algorithms — verified by CI on three platforms. MSRV 1.85.

Oxidor binds the OR-Tools C++ solvers through a deliberately thin boundary:
models are built as protobuf messages in pure Rust, and only `solve()`
crosses the FFI line — through the official C API where it suffices (CP-SAT;
the same architecture Google chose for its official Go bindings), and through
Oxidor's own exception-safe C shim where the C API is missing or too narrow
(routing, the algorithms, MathOpt solve parameters, CP-SAT solution
callbacks). No C++ type ever surfaces in the API.

## Quick start

```toml
[dependencies]
# Zero local setup: the build downloads a SHA-256-verified static OR-Tools
# bundle from this project's releases (see the platform table below).
oxidor = { git = "https://github.com/coldsmirk/oxidor", features = ["download-prebuilt"] }
```

```rust
use oxidor::CpModelBuilder;

let mut model = CpModelBuilder::new();
let x = model.new_int_var(0..=10);
let y = model.new_int_var(0..=10);
model.add_less_or_equal(x + y, 14);
model.maximize(2 * x + 3 * y);

let response = model.solve();
if let Some(solution) = response.solution() {
    println!("x = {}, y = {}", solution.value(x), solution.value(y));
}
```

Long solves take a first-class time limit
(`model.solve_with_time_limit(Duration::from_secs(10))`), and every other
CP-SAT knob is reachable through `solve_with_parameters(&SatParameters { … })`.
The constraint catalog is complete: linear (in)equalities, Booleans,
`all_different`, min/max/abs, integer product/division/modulo, `element`,
tables, circuits, automata, reservoirs, intervals with `no_overlap` /
`no_overlap_2d` / `cumulative`, inverse permutations, plus solution hints and
assumptions. Streaming solution callbacks
(`solve_with_solution_callback`) observe every feasible solution as the
search finds it and can stop early; a raw `solve_model_proto` entry solves
hand-built or hand-modified `CpModelProto`s.

For a real scheduling model — nurses, days, shifts, even workloads — see
[`oxidor-cpsat/examples/nurse_scheduling.rs`](oxidor-cpsat/examples/nurse_scheduling.rs):

```text
cargo run -p oxidor-cpsat --example nurse_scheduling
```

## Getting the native library

Solving needs the OR-Tools native library, obtained one of three ways:

- **`download-prebuilt` feature** — fetches a static OR-Tools bundle built by
  this project's CI from its GitHub releases (SHA-256 pinned in the crate,
  cached under `~/.cache/oxidor`) and links it statically: no local setup,
  self-contained binaries. Covers CP-SAT, routing, and the algorithms;
  MathOpt's solver registry needs the dynamic library, so use
  `ORTOOLS_PREFIX` for it.
- **`ORTOOLS_PREFIX`** — point it at an extracted official [C++ release
  archive](https://github.com/google/or-tools/releases) (dynamic linking, all
  solvers; always wins when set):

  ```sh
  export ORTOOLS_PREFIX=/path/to/extracted/or-tools
  ```

- **Model building alone** (`default-features = false`) — pure Rust, needs
  nothing.

Prebuilt bundles currently exist for:

| Target | `download-prebuilt` |
|---|---|
| `aarch64-apple-darwin` | ✅ |
| `x86_64-unknown-linux-gnu` | ✅ |
| `aarch64-unknown-linux-gnu` | ✅ |
| `x86_64-apple-darwin` | ❌ — use `ORTOOLS_PREFIX` |
| Windows | ❌ — untested altogether; `ORTOOLS_PREFIX` may work but is not covered by CI |

## Beyond CP-SAT

Reach for CP-SAT when the problem is combinatorial (discrete choices,
scheduling rules, logical conditions); reach for **MathOpt** when it is a
classic LP/MIP over continuous or integer quantities, with the solver chosen
per call (Glop, SCIP, CP-SAT, and PDLP ship in the official archives):

```rust
use oxidor::mathopt::{Model, SolverType};

let mut model = Model::new();
let x = model.new_continuous_variable(0.0..=10.0);
let y = model.new_continuous_variable(0.0..=10.0);
model.add_less_or_equal(x + y, 14.0);
model.maximize(2.0 * x + 3.0 * y);

let result = model.solve(SolverType::Glop)?;
if let Some(solution) = result.primal_solution() {
    println!("x = {}, y = {}", solution.value(x), solution.value(y));
}
```

MathOpt solves take per-call parameters (`solve_with_parameters` — time and
solution limits, gap tolerances, threads, seed) and a first-class
`solve_with_time_limit`. Picking a `SolverType` the linked library does not
register fails cleanly with a `SolveError`; probe availability up front with
`oxidor::mathopt::registered_solvers()`.

Long solves can be stopped from another thread — CP-SAT via
`StopToken`/`Stopper`, MathOpt via `SolveInterrupter` — and CP-SAT can
enumerate a model's full solution set (`SolveResponse::solutions()`).

Vehicle routing (`routing` feature) works over a distance matrix and scales
from a plain TSP to a VRP with capacities, time windows (a `TimeDimension`
with travel times, service times, and per-node windows — solutions then
report per-stop arrival times), pickup-and-delivery pairs, and per-vehicle
fixed costs. The classic algorithms (`algorithms` feature) come as plain
calls:

```rust
use oxidor::routing::RoutingProblem;
use oxidor::algorithms::solve_knapsack;

let response = RoutingProblem::from_matrix(matrix)?
    .with_vehicles(2)
    .with_capacities(demands, capacities)
    .solve()?;
if let Some(tour) = response.solution() {
    println!("cost {}: {:?}", tour.objective_value(), tour.routes());
}

let packing = solve_knapsack(&[60, 100, 120], &[10, 20, 30], 50)?;
```

Solving compiles Oxidor's own small C++ shim (routing, the algorithm
classes, MathOpt parameters, and CP-SAT observers have no — or too narrow an
— upstream C API), which needs the OR-Tools headers and a C++20 compiler;
official archives and the prebuilt bundles both ship the headers. Every shim
entry point catches C++ exceptions; they never cross into Rust.

## Workspace layout

| Crate | Role |
|---|---|
| [`oxidor`](oxidor/) | Umbrella crate: re-exports the per-solver APIs behind feature flags |
| [`oxidor-cpsat`](oxidor-cpsat/) | Idiomatic API for the CP-SAT constraint programming solver |
| [`oxidor-mathopt`](oxidor-mathopt/) | Idiomatic API for MathOpt: LP/MIP over Glop, SCIP, CP-SAT, PDLP |
| [`oxidor-routing`](oxidor-routing/) | TSP / VRP with capacities, time windows, pickups-deliveries |
| [`oxidor-algorithms`](oxidor-algorithms/) | Knapsack, max flow, min cost flow, linear sum assignment |
| [`oxidor-protos`](oxidor-protos/) | Generated protobuf model types (pure Rust, committed to the repo) |
| [`oxidor-sys`](oxidor-sys/) | Native library location, linkage, raw FFI, and the C++ shim |
| [`xtask`](xtask/) | Maintainer tasks (`cargo run -p xtask -- gen-protos`); not published |

## Development

```sh
# One-time setup: fetch the OR-Tools v9.15 C++ archive for your platform from
#   https://github.com/google/or-tools/releases/tag/v9.15
# and extract it to .ortools/v9.15 (gitignored), e.g. on Apple silicon:
mkdir -p .ortools && cd .ortools
curl -L -o ortools.tar.gz https://github.com/google/or-tools/releases/download/v9.15/or-tools_arm64_macOS-26.2_cpp_v9.15.6755.tar.gz
tar xzf ortools.tar.gz && mv or-tools_* v9.15 && rm ortools.tar.gz && cd ..

cargo test --workspace   # .cargo/config.toml points ORTOOLS_PREFIX at .ortools/v9.15
```

`cargo run -p xtask -- gen-protos` regenerates `oxidor-protos/src/generated/`
from the vendored `.proto` files (pure Rust via protox — no `protoc` needed);
the output is committed.

## Roadmap

1. **CP-SAT** — ✅ model builder with the full constraint catalog, solve
   (+ first-class time limit and full `SatParameters`), interruptible solve
   (`StopToken`/`Stopper`), solution enumeration, streaming solution
   callbacks with early stop, hints/assumptions, raw-proto solve. Next:
   possibly typed accessors for more response statistics.
2. **Linear solving (MathOpt)** — ✅ LP/MIP model builder, per-call solver
   choice (Glop / SCIP / CP-SAT / PDLP), per-solve parameters + time limit,
   solver-registry probing, interruption, clean error paths. Next: dual
   solution accessors (available today via `SolveResult::raw`).
3. **Routing (VRP/TSP)** — ✅ TSP/CVRP over a distance matrix, time windows
   with per-stop arrival times, pickups/deliveries, per-vehicle fixed costs;
   search parameters as protos. Next: Rust transit callbacks (the callback
   trampoline infrastructure already exists for CP-SAT), disjunctions /
   optional visits.
4. **Algorithms** — ✅ knapsack (multi-dimensional branch and bound), max
   flow, min cost flow, linear sum assignment.
5. **Distribution** — ✅ CI test matrix on three platforms; prebuilt static
   bundles consumed by `download-prebuilt` (checksums pinned in-crate, e2e
   tested in CI). Next: more targets (Intel macOS, Windows), a `vendored`
   source-build mode, and the first crates.io release.

## License

Apache-2.0, the same license as OR-Tools. This project is not affiliated
with or endorsed by Google.
