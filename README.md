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

Solving needs the OR-Tools native library: download an official [C++ release
archive](https://github.com/google/or-tools/releases) for your platform,
extract it, and set `ORTOOLS_PREFIX` to the extracted directory. Model
building alone (`default-features = false`) is pure Rust and needs nothing.

For a real scheduling model — nurses, days, shifts, even workloads — see
[`oxidor-cpsat/examples/nurse_scheduling.rs`](oxidor-cpsat/examples/nurse_scheduling.rs):

```text
cargo run -p oxidor-cpsat --example nurse_scheduling
```

## Workspace layout

| Crate | Role |
|---|---|
| [`oxidor`](oxidor/) | Umbrella crate: re-exports the per-solver APIs behind feature flags |
| [`oxidor-cpsat`](oxidor-cpsat/) | Idiomatic API for the CP-SAT constraint programming solver |
| [`oxidor-protos`](oxidor-protos/) | Generated protobuf model types (pure Rust, committed to the repo) |
| [`oxidor-sys`](oxidor-sys/) | Native library location, linkage, and raw `extern "C"` declarations |
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

1. **CP-SAT** — ✅ model builder + solve over the official `cp_solver_c.h` C
   API; next: interruptible solves, solution callbacks (which the official Go
   bindings lack), prebuilt static libraries so `cargo add oxidor` needs no
   setup at all.
2. **Linear solving** — MathOpt / linear solver protos.
3. **Routing (VRP/TSP)** — `cxx` bridge over the imperative C++ API.
4. Graph algorithms, knapsack, and the remaining modules.

## License

Apache-2.0, the same license as OR-Tools. This project is not affiliated
with or endorsed by Google.
