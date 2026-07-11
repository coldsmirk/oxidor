//! Unofficial Rust bindings for [Google OR-Tools](https://developers.google.com/optimization).
//!
//! This umbrella crate re-exports the per-solver crates behind feature flags:
//!
//! - `cpsat` (default) — the CP-SAT constraint programming solver.
//!
//! Planned: linear solving (MathOpt), routing (VRP/TSP), graph algorithms.

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat as cpsat;
