# AGENTS.md

Conventions and invariants for working in this repo. See README.md for the
architecture and setup.

## Commands

- Test: `cargo test --workspace` (needs `.ortools/v9.15` — see README
  *Development*; `.cargo/config.toml` points `ORTOOLS_PREFIX` at it)
- Lint: `cargo clippy --workspace --all-targets` (keep it warning-clean) and
  `cargo fmt --all --check`
- Regenerate protos: `cargo run -p xtask -- gen-protos` (commit the output)
- Example: `cargo run -p oxidor-cpsat --example nurse_scheduling`

## Design invariants

- **The FFI boundary is serialized protos only.** Models cross into the
  native library as `CpModelProto` bytes through the official C API
  (`ortools/sat/c_api/cp_solver_c.h`, declared in `oxidor-sys`); responses
  come back as `CpSolverResponse` bytes. No C++ type ever surfaces in any
  public API. Buffers returned by the C API are freed with the C allocator.
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
  crate whose binaries load the library (`oxidor-cpsat`, the umbrella)
  declares a direct `oxidor-sys` dependency plus a build script turning that
  into an rpath link arg. This covers tests, examples, and doctests — the
  edition-2024 merged doctest binary loads the library even for `no_run`
  examples.
- **Statuses are outcomes, not errors.** Infeasible/Unknown are values of
  `SolveStatus`; `solution()` returns `Option`. No panics in the public API
  apart from documented programmer errors (e.g. mismatched slice lengths).
- **Public API carries docs.** `#![warn(missing_docs)]` in the published
  crates; doc examples that solve are runnable in this repo (kept `no_run`
  only where they'd mislead users without the native library).

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
