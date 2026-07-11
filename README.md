# Oxidor

**Unofficial Rust bindings for [Google OR-Tools](https://developers.google.com/optimization).**

> Status: early development — project skeleton, no functional release yet.

Oxidor binds the OR-Tools C++ solvers to Rust through a deliberately thin
boundary: models are built as protobuf messages in pure Rust, and only the
`solve()` call crosses the FFI line (the same architecture Google chose for
its official Go bindings). The goal is `cargo add oxidor` with no system
dependencies — the native library is statically linked, fetched prebuilt or
built from source via Cargo features.

## Workspace layout

| Crate | Role |
|---|---|
| [`oxidor`](oxidor/) | Umbrella crate: re-exports the per-solver APIs behind feature flags |
| [`oxidor-cpsat`](oxidor-cpsat/) | Idiomatic API for the CP-SAT constraint programming solver |
| [`oxidor-protos`](oxidor-protos/) | Generated protobuf model types (pure Rust, committed to the repo) |
| [`oxidor-sys`](oxidor-sys/) | Native library acquisition, linkage, and raw `extern "C"` declarations |

## Roadmap

1. **CP-SAT** — proto pipeline + the official `cp_solver_c.h` C API, plus
   solution callbacks (which the official Go bindings lack).
2. **Linear solving** — MathOpt / linear solver protos.
3. **Routing (VRP/TSP)** — `cxx` bridge over the imperative C++ API.
4. Graph algorithms, knapsack, and the remaining modules.

## License

Apache-2.0, the same license as OR-Tools. This project is not affiliated
with or endorsed by Google.
