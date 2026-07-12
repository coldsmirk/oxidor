//! Unofficial Rust bindings for [Google
//! OR-Tools](https://developers.google.com/optimization).
//!
//! Four solver families are supported:
//!
//! - [CP-SAT](mod@cpsat) — constraint programming for scheduling, rostering,
//!   packing, and other combinatorial problems. Its everyday types are
//!   re-exported at the crate root.
//! - [MathOpt](mod@mathopt) — linear and mixed-integer optimization (Glop,
//!   SCIP, CP-SAT, PDLP behind one model).
//! - [Routing](mod@routing) — TSP and capacitated VRP over a distance matrix.
//! - [Algorithms](mod@algorithms) — knapsack, max flow, min cost flow.
//!
//! Reach for CP-SAT when the problem is combinatorial (discrete choices,
//! scheduling rules, logical conditions); reach for MathOpt when it is a
//! classic LP/MIP over continuous or integer quantities.
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
//! let x = model.new_continuous_variable(0.0..=10.0);
//! let y = model.new_continuous_variable(0.0..=10.0);
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
//! - `routing` — TSP/VRP over a distance matrix, [`routing`](mod@routing).
//!   Solving compiles Oxidor's C++ shim (needs the OR-Tools headers and a
//!   C++20 compiler), so it is not in the default set.
//! - `algorithms` — knapsack, max flow, min cost flow,
//!   [`algorithms`](mod@algorithms). Compiles the same C++ shim, and — unlike
//!   the model-building crates — always links the native library (there is no
//!   pure-model subset to fall back to).
//! - `solve` *(default)* — links the native OR-Tools library. Disable to
//!   build and serialize models on platforms without it.
//! - `download-prebuilt` — when `ORTOOLS_PREFIX` is not set, fetch a
//!   SHA-256-verified static OR-Tools bundle from this project's GitHub
//!   releases and link it: zero local setup. Covers CP-SAT, routing, and the
//!   algorithms; MathOpt needs a dynamic library via `ORTOOLS_PREFIX`.

#![warn(missing_docs)]

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat as cpsat;

#[cfg(feature = "mathopt")]
pub use oxidor_mathopt as mathopt;

#[cfg(feature = "routing")]
pub use oxidor_routing as routing;

#[cfg(feature = "algorithms")]
pub use oxidor_algorithms as algorithms;

#[cfg(feature = "cpsat")]
pub use oxidor_cpsat::{
    BoolVar, Constraint, CpModelBuilder, Domain, IntVar, IntervalVar, LinearExpr, SatParameters,
};

#[cfg(all(feature = "cpsat", feature = "solve"))]
pub use oxidor_cpsat::{Solution, SolveResponse, SolveStatus, StopToken, Stopper};
