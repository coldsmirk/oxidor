# AGENTS.md

Conventions and invariants for working in this repo. See README.md for the
architecture and setup.

## Commands

- Test: `cargo test --workspace` (needs `.ortools/v9.15` — see README
  *Development*; `.cargo/config.toml` points `ORTOOLS_PREFIX` at it; solving
  compiles the C++ shim, so a C++20 compiler is required too). The umbrella's
  default features enable `oxidor-cpsat/callbacks`, so feature unification
  runs the callback tests in the plain workspace test. Note: an explicit
  `cargo test --workspace --doc` fails in `oxidor-protos` (upstream proto
  comments are not Rust; the crate sets `doctest = false`, which `--doc`
  overrides) — the plain workspace test is the supported entry point.
- Lint: `cargo clippy --workspace --all-targets` (keep it warning-clean) and
  `cargo fmt --all --check`
- Regenerate protos: `cargo run -p xtask -- gen-protos` (commit the output)
- Example: `cargo run -p oxidor-cpsat --example nurse_scheduling`

## API conventions

- **Variables are `new_*`, constraints are `add_*`** in every model-building
  crate (`new_int_var`, `new_continuous_variable` / `add_less_or_equal`,
  `add_linear_constraint`).
- **"How did it end" is `status()`** on every response/result type; **the
  objective is `objective_value()`**; Boolean extraction is `bool_value`.
- **No-solution accessors return `Option`**, never a silent 0/empty
  (`SolveResponse::solution`, `SolveResult::primal_solution`,
  `RoutingResponse::solution`, `MinCostFlowResponse::solution`).
- **Wire-mirroring enums and error types are `#[non_exhaustive]`** so new
  upstream values are not breaking changes.

## Design invariants

- **The FFI boundary is serialized protos and POD only.** Models cross into
  the native library as proto bytes — CP-SAT through the official C API
  (`ortools/sat/c_api/cp_solver_c.h`), MathOpt through the shim (the
  official C API takes no per-solve parameters, so `oxidor-mathopt`'s
  `solve` requires `oxidor-sys/shim`) — and results come back as proto
  bytes. No C++ type ever surfaces in any public API. Buffers from the C API
  and the shim are freed with the C allocator (`libc::free`); the (unused by
  our crates, still declared) upstream MathOpt C API frees with
  `MathOptFree`. Output buffers are copied out and **freed before
  decoding**, so a decode panic cannot leak them; null/empty outputs are
  tolerated defensively.
- **Validate inputs before they reach native code.** Anything upstream
  documents as a precondition (non-negative capacities/demands/costs, node
  indices within `i32`, buffer lengths within `c_int`) is checked on the Rust
  side and surfaces as an ordinary error (or, for CP-SAT's infallible solve,
  a documented panic). Upstream `CHECK`s abort the whole process and release
  builds strip the `DCHECK`s entirely — never rely on them.
- **Handles are branded with their model's identity.** `CpModelBuilder` /
  `Model` stamp a process-unique id onto every variable/literal/interval
  handle; expressions carry it, and every constraint, objective, and
  solution-accessor checks it (documented panic on mismatch). This turns
  cross-model misuse — otherwise a silently wrong model or answer — into an
  immediate programmer-error panic. Cloning a builder preserves its identity.
- **One CP-SAT solve per stop environment, enforced by types.** The C API
  requires it, so `StopToken` is not `Clone` and the interruptible solves
  take `&mut StopToken`; only `Stopper` clones (which can merely stop) cross
  threads. Never reintroduce a shared-token solve path.
- **C++ exceptions cannot cross the boundary.** A thrown exception aborts the
  process. Every shim entry point is wrapped in try/catch; never add an API
  whose misuse triggers an uncaught throw when a clean status path exists.
- **Rust panics cannot cross the boundary either.** Any Rust closure invoked
  from native code (the CP-SAT solution-callback trampoline in
  `oxidor-cpsat/src/callbacks.rs`) runs behind `catch_unwind`; the caught
  payload asks the search to stop, is carried back across the FFI return,
  and resumes on the calling thread. A new callback-style entry point must
  copy this pattern: `extern "C"` trampoline + `user_data` state struct +
  panic barrier + `Send` bound (solvers may call from worker threads).
- **The shim owns the C boundary where the upstream C API is missing or too
  narrow** (routing, knapsack, flows, assignment, MathOpt parameters, CP-SAT
  observers): `oxidor-sys/cpp/oxidor_shim.cc`, compiled under the sys `shim`
  feature against the installation's headers (with the compile definitions
  the official CMake config exports — see build.rs). Shim rules: POD
  scalars/arrays, serialized protos, and C function pointers only; every
  entry point wrapped in try/catch; outputs malloc-allocated and freed by
  the Rust side with `libc::free`. A new entry point means updating the
  `.cc`, the `#[cfg(feature = "shim")]` decls in sys, and this file
  together. Structured requests may cross as `#[repr(C)]` structs mirrored
  field-for-field in the `.cc` (see `OxidorRoutingProblem`) — safe because
  both sides ship in one crate and compile in lockstep; it is an internal
  contract, not a wire format. The shim also links the unversioned
  absl/protobuf shared libraries next to libortools (inlined template code;
  the linker won't resolve those transitively).
- **Generated proto code is committed.** `oxidor-protos/src/generated/` is
  produced offline by `xtask gen-protos` (protox, no `protoc`) from the
  vendored `.proto` files of a pinned OR-Tools release. Users never need
  protoc, Bazel, or the network at build time. Bump protos and the native
  library version together — the encoding must match the linked library.
- **Model building is pure Rust.** Only the `solve` feature (default) pulls
  in `oxidor-sys`/linking. `--no-default-features` must always build without
  any native library present. (`oxidor-algorithms` is the exception: it is
  FFI calls only, with no pure-model subset — enabling it always links.)
  `oxidor-cpsat`'s `callbacks` feature is additive on top of `solve` (it
  adds the shim); keep it optional there, but the umbrella enables it by
  default.
- **`i64::MIN`/`i64::MAX` are CP-SAT's ±infinity sentinels.** Domain
  arithmetic (e.g. constant folding in `add_linear_constraint`) pins them
  instead of shifting, matching the C++ CpModelBuilder.
- **Only `oxidor-sys` links OR-Tools** (`links = "ortools"`). It publishes
  the library directory to *direct* dependents as `DEP_ORTOOLS_LIBDIR`, and
  each crate whose binaries load the shared library (`oxidor-cpsat`,
  `oxidor-mathopt`, `oxidor-routing`, `oxidor-algorithms`, the umbrella)
  declares a direct `oxidor-sys` dependency plus a build script turning that
  into an rpath link arg. This covers tests, examples, and doctests — the
  edition-2024 merged doctest binary loads the library even for `no_run`
  examples.
- **Statuses are outcomes, not errors.** Infeasible/Unknown are values of
  `SolveStatus`; `solution()` returns `Option`. No panics in the public API
  apart from documented programmer errors (mismatched slice lengths,
  cross-model handles, >2 GiB serialized models).
- **Public API carries docs.** `#![warn(missing_docs)]` in the published
  crates (except `oxidor-protos`, which is generated code); doc examples
  that solve are runnable in this repo (kept `no_run` only where they'd
  mislead users without the native library), and each `no_run` golden value
  has an executed integration-test mirror.

- **Prebuilt bundles are release assets with pinned checksums.** The
  `prebuilt-ortools` workflow (manual dispatch) builds static OR-Tools per
  platform — patching upstream's hard-coded shared dependency builds and
  vendoring bzip2 — and merges everything into one `libortools.a`. Publishing
  = upload the artifacts to the `ortools-v9.15` release and regenerate
  `oxidor-sys/prebuilt-checksums.txt`; the checksum is part of the local
  cache key, so republished bundles invalidate stale caches. **Assets are
  immutable once a crate pinning their checksums has shipped** — republish
  under a new tag + new checksums + new crate version. Known limitation:
  MathOpt's solver registry (global initializers) does not survive selective
  static linking — `download-prebuilt` covers CP-SAT, routing, and
  algorithms; MathOpt needs `ORTOOLS_PREFIX`.

## Collateral steps for a change

- Bumping the OR-Tools version — the version literal is deliberately pinned
  in several places; update ALL of them together:
  1. re-vendor `.proto` files and run `xtask gen-protos` (commit the output);
  2. `.cargo/config.toml` (`ORTOOLS_PREFIX` path) and the README
     *Development* section;
  3. `.github/workflows/ci.yml` (archive names, download URL, cache key);
  4. `.github/workflows/prebuilt.yml` (`ORTOOLS_TAG`);
  5. `oxidor-sys/build.rs` (`RELEASE_TAG`, `ORTOOLS_VERSION`) plus new
     bundles + `prebuilt-checksums.txt`;
  6. `oxidor-protos/src/lib.rs` (version note in the module docs);
  then re-run the full test suite against the matching native archive.
- Adding public API to a solver crate → consider re-exporting it from the
  umbrella `oxidor` crate, keep feature gating consistent (`solve`-gated
  items stay gated in both crates), and follow the API conventions above.
- Adding a `no_run` doc example with a golden value → add an integration
  test that executes the same model.
- Any change → `cargo fmt --all`, `cargo clippy --workspace --all-targets`,
  `cargo test --workspace` before done.
