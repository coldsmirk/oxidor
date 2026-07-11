# Oxidor

**Unofficial Rust bindings for [Google OR-Tools](https://developers.google.com/optimization).**

> Status: early development. CP-SAT solving works end to end; the native
> library is currently located via `ORTOOLS_PREFIX` (prebuilt-download and
> vendored builds are on the roadmap).

Oxidor binds the OR-Tools C++ solvers through a deliberately thin boundary:
models are built as protobuf messages in pure Rust, and only `solve()`
crosses the FFI line — through the official CP-SAT C API, the same
architecture Google chose for its official Go bindings. No C++ type ever
surfaces in the API.

## Quick start

```toml
[dependencies]
oxidor = "0.0.1"
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

Solving needs the OR-Tools native library, obtained one of three ways:

- **`download-prebuilt` feature** — fetches a static OR-Tools bundle built by
  this project's CI from its GitHub releases (SHA-256 verified, cached under
  `~/.cache/oxidor`) and links it statically: no local setup, self-contained
  binaries. Covers CP-SAT, routing, and the algorithms; MathOpt's solver
  registry needs the dynamic library, so use `ORTOOLS_PREFIX` for it.
- **`ORTOOLS_PREFIX`** — point it at an extracted official [C++ release
  archive](https://github.com/google/or-tools/releases) (dynamic linking, all
  solvers; always wins when set).
- **Model building alone** (`default-features = false`) — pure Rust, needs
  nothing.

For a real scheduling model — nurses, days, shifts, even workloads — see
[`oxidor-cpsat/examples/nurse_scheduling.rs`](oxidor-cpsat/examples/nurse_scheduling.rs):

```text
cargo run -p oxidor-cpsat --example nurse_scheduling
```

Linear and mixed-integer programming go through MathOpt, with the solver
chosen per call (Glop, SCIP, CP-SAT, and PDLP ship in the official archives):

```rust
use oxidor::mathopt::{Model, SolverType};

let mut model = Model::new();
let x = model.add_continuous_variable(0.0..=10.0);
let y = model.add_continuous_variable(0.0..=10.0);
model.add_less_or_equal(x + y, 14.0);
model.maximize(2.0 * x + 3.0 * y);

let result = model.solve(SolverType::Glop)?;
if let Some(solution) = result.primal_solution() {
    println!("x = {}, y = {}", solution.value(x), solution.value(y));
}
```

Long solves can be stopped from another thread — CP-SAT via `StopToken`,
MathOpt via `SolveInterrupter` — and CP-SAT can enumerate a model's full
solution set (`SolveResponse::solutions()`).

Vehicle routing (TSP/VRP, `routing` feature) works over a distance matrix,
and the classic algorithms (`algorithms` feature) come as plain calls:

```rust
use oxidor::routing::RoutingProblem;
use oxidor::algorithms::solve_knapsack;

let tour = RoutingProblem::from_matrix(matrix)?
    .with_vehicles(2)
    .with_capacities(demands, capacities)
    .solve()?;

let packing = solve_knapsack(&[60, 100, 120], &[10, 20, 30], 50)?;
```

These two features compile Oxidor's own small C++ shim (routing and the
algorithm classes have no upstream C API), which needs the OR-Tools headers
and a C++20 compiler — hence they are opt-in rather than default. Every shim
entry point catches C++ exceptions; they never cross into Rust.

## Workspace layout

| Crate | Role |
|---|---|
| [`oxidor`](oxidor/) | Umbrella crate: re-exports the per-solver APIs behind feature flags |
| [`oxidor-cpsat`](oxidor-cpsat/) | Idiomatic API for the CP-SAT constraint programming solver |
| [`oxidor-mathopt`](oxidor-mathopt/) | Idiomatic API for MathOpt: LP/MIP over Glop, SCIP, CP-SAT, PDLP |
| [`oxidor-routing`](oxidor-routing/) | TSP / capacitated VRP over a distance matrix |
| [`oxidor-algorithms`](oxidor-algorithms/) | Knapsack, max flow, min cost flow |
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

1. **CP-SAT** — ✅ model builder, solve, interruptible solve (`StopToken`),
   solution enumeration; next: streaming solution callbacks (via the shim).
2. **Linear solving (MathOpt)** — ✅ LP/MIP model builder, per-call solver
   choice (Glop / SCIP / CP-SAT / PDLP), interruption, clean error paths.
3. **Routing (VRP/TSP)** — ✅ v1: TSP and capacitated VRP over a distance
   matrix through Oxidor's C++ shim; search parameters as protos. Next:
   time windows, pickups/deliveries, Rust transit callbacks.
4. **Algorithms** — ✅ knapsack (multi-dimensional branch and bound), max
   flow, min cost flow.
5. **Distribution** — CI test matrix ✅; a `prebuilt-ortools` workflow builds
   static libraries per platform, and a `download-prebuilt` mode in
   `oxidor-sys` will consume them (with checksums) once published — the goal
   is `cargo add oxidor` with no setup at all.

## License

Apache-2.0, the same license as OR-Tools. This project is not affiliated
with or endorsed by Google.
