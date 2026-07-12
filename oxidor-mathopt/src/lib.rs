//! Idiomatic Rust API for [Google OR-Tools
//! MathOpt](https://developers.google.com/optimization/math_opt) — linear and
//! mixed-integer optimization with a choice of solvers (Glop, SCIP, CP-SAT,
//! PDLP, …) behind one model.
//!
//! Model building is pure Rust: [`Model`] assembles MathOpt's wire format
//! (`ModelProto`) with typed variable handles and operator-overloaded
//! [`LinearExpr`]s. Solving crosses the FFI boundary once per call through
//! Oxidor's C shim (the upstream MathOpt C API takes no per-solve
//! parameters), exchanging serialized protobuf bytes — no C++ type ever
//! surfaces here.
//!
//! # Example: production planning
//!
//! ```no_run
//! use oxidor_mathopt::{Model, SolverType, TerminationReason};
//!
//! let mut model = Model::new();
//! let chairs = model.new_continuous_variable(0.0..=f64::INFINITY);
//! let tables = model.new_continuous_variable(0.0..=f64::INFINITY);
//!
//! // Wood and labor budgets.
//! model.add_less_or_equal(5.0 * chairs + 20.0 * tables, 400.0);
//! model.add_less_or_equal(10.0 * chairs + 15.0 * tables, 450.0);
//! model.maximize(45.0 * chairs + 80.0 * tables);
//!
//! let result = model.solve(SolverType::Glop).expect("Glop is available");
//! assert_eq!(result.status(), TerminationReason::Optimal);
//! let solution = result.primal_solution().expect("optimal has a solution");
//! println!("chairs = {}, tables = {}", solution.value(chairs), solution.value(tables));
//! ```
//!
//! # Features
//!
//! - `solve` *(default)* — links the native OR-Tools library via `oxidor-sys`
//!   and compiles Oxidor's C++ shim (needs the OR-Tools headers and a C++20
//!   compiler). Disable it to build and serialize models on platforms without
//!   the library; hand [`Model::proto`] to a solver elsewhere.

#![warn(missing_docs)]

mod expr;
mod model;
#[cfg(feature = "solve")]
mod solve;

pub use expr::{LinearExpr, Variable};
pub use model::{LinearConstraint, Model};
#[cfg(feature = "solve")]
pub use solve::{
    PrimalSolution, SolveError, SolveInterrupter, SolveResult, SolverType, TerminationReason,
    registered_solvers,
};

/// The generated OR-Tools proto types this API builds on, for advanced use
/// (inspecting [`Model::proto`], reading [`SolveResult::raw`]).
pub use oxidor_protos as protos;

pub use protos::math_opt::SolveParametersProto;
