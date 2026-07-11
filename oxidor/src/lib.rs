//! Unofficial Rust bindings for [Google
//! OR-Tools](https://developers.google.com/optimization).
//!
//! Two solver families are supported so far:
//!
//! - [CP-SAT](mod@cpsat) — constraint programming for scheduling, rostering,
//!   packing, and other combinatorial problems. Its everyday types are
//!   re-exported at the crate root.
//! - [MathOpt](mod@mathopt) — linear and mixed-integer optimization (Glop,
//!   SCIP, CP-SAT, PDLP behind one model), under the [`mathopt`](mod@mathopt)
//!   module.
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
//! Linear programming goes through [`mathopt`](mod@mathopt):
//!
//! ```no_run
//! use oxidor::mathopt::{Model, SolverType};
//!
//! let mut model = Model::new();
//! let x = model.add_continuous_variable(0.0..=10.0);
//! let y = model.add_continuous_variable(0.0..=10.0);
//! model.add_less_or_equal(x + y, 14.0);
//! model.maximize(2.0 * x + 3.0 * y);
//!
//! let result = model.solve(SolverType::Glop).expect("Glop is available");
//! if let Some(solution) = result.primal_solution() {
//!     println!("x = {}, y = {}", solution.value(x), solution.value(y));
//! }
//! ```
//!
//! # Features
//!
//! - `cpsat` *(default)* — the CP-SAT API, [`cpsat`](mod@cpsat).
//! - `mathopt` *(default)* — the LP/MIP API, [`mathopt`](mod@mathopt).
//! - `solve` *(default)* — links the native OR-Tools library. Disable to
//!   build and serialize models on platforms without it.
//!
//! Planned: routing (VRP/TSP), graph algorithms.

#![warn(missing_docs)]

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat as cpsat;

#[cfg(feature = "mathopt")]
pub use oxidor_mathopt as mathopt;

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat::{
    BoolVar, Constraint, CpModelBuilder, Domain, IntVar, IntervalVar, LinearExpr, SatParameters,
};

#[cfg(all(feature = "cpsat", feature = "solve"))]
pub use oxidor_cpsat::{Solution, SolveResponse, SolveStatus, StopToken};
