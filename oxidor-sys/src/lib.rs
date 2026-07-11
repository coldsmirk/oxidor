//! Native library linkage and raw FFI declarations for Google OR-Tools.
//!
//! This crate is responsible for locating or obtaining the OR-Tools native
//! library and exposing the `extern "C"` entry points (starting with the
//! official CP-SAT C API, `ortools/sat/c_api/cp_solver_c.h`). Higher-level
//! crates never link OR-Tools directly — they go through this crate.
//!
//! Planned acquisition modes, selected via Cargo features:
//! - `download-prebuilt` (default) — fetch a static library built by this
//!   project's CI from its GitHub releases.
//! - `vendored` — build OR-Tools from source with CMake.
//! - `system` — link an existing installation via `ORTOOLS_PREFIX`.
