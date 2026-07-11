# AGENTS.md

Conventions and invariants for working in this repo. See README.md for the
architecture and setup.

## Commands

- Test: `cargo test --workspace` (needs `.ortools/v9.15` — see README
  *Development*; `.cargo/config.toml` points `ORTOOLS_PREFIX` at it; the
  routing/algorithms crates compile the C++ shim, so a C++20 compiler is
  required too)
- Lint: `cargo clippy --workspace --all-targets` (keep it warning-clean) and
  `cargo fmt --all --check`
- Regenerate protos: `cargo run -p xtask -- gen-protos` (commit the output)
- Example: `cargo run -p oxidor-cpsat --example nurse_scheduling`

## Design invariants

- **The FFI boundary is serialized protos only.** Models cross into the
  native library as proto bytes through the official C APIs — CP-SAT
  (`ortools/sat/c_api/cp_solver_c.h`) and MathOpt
  (`ortools/math_opt/core/c_api/solver.h`), both declared in `oxidor-sys` —
  and results come back as proto bytes. No C++ type ever surfaces in any
  public API. CP-SAT buffers are freed with the C allocator (`libc::free`);
  MathOpt buffers with `MathOptFree`.
- **C++ exceptions cannot cross the boundary.** A thrown exception aborts the
  process (seen with MathOpt solver types whose backend isn't linked). Never
  add an API whose misuse triggers one when a clean status path exists;
  document the hazard when upstream leaves no choice (see
  `SolverType`'s docs).
- **The shim owns the C boundary for callback-free bridging.** APIs without
  an upstream C API (routing, knapsack, flows) go through
  `oxidor-sys/cpp/oxidor_shim.cc`, compiled under the sys `shim` feature
  against the installation's headers (with the compile definitions the
  official CMake config exports — see build.rs). Shim rules: POD arrays and
  serialized protos only; every entry point wrapped in try/catch; outputs
  malloc-allocated and freed by the Rust side with `libc::free`. A new entry
  point means updating the `.cc`, the `#[cfg(feature = "shim")]` decls in
  sys, and this file together. The shim also links the unversioned
  absl/protobuf shared libraries next to libortools (inlined template code;
  the linker won't resolve those transitively).
- **Generated proto code is committed.** `oxidor-protos/src/generated/` is
  produced offline by `xtask gen-protos` (protox, no `protoc`) from the
  vendored `.proto` files of a pinned OR-Tools release. Users never need
  protoc, Bazel, or the network at build time. Bump protos and the native
  library version together — the encoding must match the linked library.
- **Model building is pure Rust.** Only the `solve` feature (default) pulls
  in `oxidor-sys`/linking. `--no-default-features` must always build without
  any native library present.
- **`i64::MIN`/`i64::MAX` are CP-SAT's ±infinity sentinels.** Domain
  arithmetic (e.g. constant folding in `add_linear_constraint`) pins them
  instead of shifting, matching the C++ CpModelBuilder.
- **Only `oxidor-sys` links OR-Tools** (`links = "ortools"`). It publishes
  the install prefix to *direct* dependents as `DEP_ORTOOLS_ROOT`, and each
  crate whose binaries load the library (`oxidor-cpsat`, `oxidor-mathopt`,
  the umbrella) declares a direct `oxidor-sys` dependency plus a build script
  turning that into an rpath link arg. This covers tests, examples, and
  doctests — the edition-2024 merged doctest binary loads the library even
  for `no_run` examples.
- **Statuses are outcomes, not errors.** Infeasible/Unknown are values of
  `SolveStatus`; `solution()` returns `Option`. No panics in the public API
  apart from documented programmer errors (e.g. mismatched slice lengths).
- **Public API carries docs.** `#![warn(missing_docs)]` in the published
  crates; doc examples that solve are runnable in this repo (kept `no_run`
  only where they'd mislead users without the native library).

- **Prebuilt bundles are release assets with pinned checksums.** The
  `prebuilt-ortools` workflow (manual dispatch) builds static OR-Tools per
  platform — patching upstream's hard-coded shared dependency builds and
  vendoring bzip2 — and merges everything into one `libortools.a`. Publishing
  = upload the artifacts to the `ortools-v9.15` release and regenerate
  `oxidor-sys/prebuilt-checksums.txt`; the checksum is part of the local
  cache key, so republished bundles invalidate stale caches. Known
  limitation: MathOpt's solver registry (global initializers) does not
  survive selective static linking — `download-prebuilt` covers CP-SAT,
  routing, and algorithms; MathOpt needs `ORTOOLS_PREFIX`.

## Collateral steps for a change

- Bumping the OR-Tools version → re-vendor `.proto` files, run
  `xtask gen-protos`, update `.ortools/` setup docs and
  `.cargo/config.toml` paths, and re-run the full test suite against the
  matching native archive.
- Adding public API to `oxidor-cpsat` → consider re-exporting it from the
  umbrella `oxidor` crate, and keep feature gating consistent
  (`solve`-gated items stay gated in both crates).
- Any change → `cargo fmt --all`, `cargo clippy --workspace --all-targets`,
  `cargo test --workspace` before done.
