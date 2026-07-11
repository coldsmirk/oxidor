//! Unofficial Rust bindings for [Google
//! OR-Tools](https://developers.google.com/optimization).
//!
//! The first supported solver is [CP-SAT](mod@cpsat) — constraint programming
//! for scheduling, rostering, packing, and other combinatorial problems. Its
//! everyday types are re-exported at the crate root:
//!
//! ```no_run
//! use oxidor::CpModelBuilder;
//!
//! let mut model = CpModelBuilder::new();
//! let x = model.new_int_var(0..=10);
//! let y = model.new_int_var(0..=10);
//! model.add_less_or_equal(x + y, 14);
//! model.maximize(2 * x + 3 * y);
//!
//! let response = model.solve();
//! if let Some(solution) = response.solution() {
//!     println!("x = {}, y = {}", solution.value(x), solution.value(y));
//! }
//! ```
//!
//! # Features
//!
//! - `cpsat` *(default)* — the CP-SAT API, [`cpsat`](mod@cpsat).
//! - `solve` *(default)* — links the native OR-Tools library. Disable to
//!   build and serialize models on platforms without it.
//!
//! Planned: linear solving (MathOpt), routing (VRP/TSP), graph algorithms.

#![warn(missing_docs)]

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat as cpsat;

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat::{
    BoolVar, Constraint, CpModelBuilder, Domain, IntVar, IntervalVar, LinearExpr, SatParameters,
};

#[cfg(all(feature = "cpsat", feature = "solve"))]
pub use oxidor_cpsat::{Solution, SolveResponse, SolveStatus};
