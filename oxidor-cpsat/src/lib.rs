//! Idiomatic Rust API for the [Google OR-Tools CP-SAT
//! solver](https://developers.google.com/optimization/cp) — constraint
//! programming for scheduling, rostering, packing, and other combinatorial
//! problems.
//!
//! Model building is pure Rust: [`CpModelBuilder`] assembles the CP-SAT wire
//! format (`CpModelProto`) with typed variable handles and
//! operator-overloaded [`LinearExpr`]s. Solving crosses the FFI boundary
//! once, through the official CP-SAT C API, exchanging serialized protobuf
//! bytes — no C++ type ever surfaces here.
//!
//! # Example: pick shifts
//!
//! ```no_run
//! use oxidor_cpsat::CpModelBuilder;
//!
//! let mut model = CpModelBuilder::new();
//!
//! // Three candidate shifts, one worker: pick exactly two, not both of the
//! // clashing morning pair, maximizing paid hours.
//! let shifts = [model.new_bool_var(), model.new_bool_var(), model.new_bool_var()];
//! let hours = [6, 4, 8];
//!
//! model.add_at_most_one([shifts[0], shifts[1]]);
//! model.add_linear_constraint(shifts.into_iter().sum::<oxidor_cpsat::LinearExpr>(), 2..=2);
//! model.maximize(
//!     shifts.iter().zip(hours).map(|(&s, h)| s * h).sum::<oxidor_cpsat::LinearExpr>(),
//! );
//!
//! let response = model.solve();
//! let solution = response.solution().expect("feasible");
//! assert_eq!(response.objective_value(), 14.0);
//! assert!(solution.bool_value(shifts[2]));
//! ```
//!
//! # Features
//!
//! - `solve` *(default)* — links the native OR-Tools library via `oxidor-sys`.
//!   Disable it to build and serialize models on platforms without the
//!   library; hand [`CpModelBuilder::proto`] to a solver elsewhere.
//! - `callbacks` — streaming solution callbacks
//!   (`solve_with_solution_callback`): observe every feasible solution as
//!   the search finds it and stop early at will. Compiles Oxidor's C++ shim,
//!   which needs the OR-Tools headers and a C++20 compiler.

#![warn(missing_docs)]

#[cfg(feature = "callbacks")]
mod callbacks;
mod domain;
mod expr;
mod model;
#[cfg(feature = "solve")]
mod solver;

pub use domain::Domain;
pub use expr::{BoolVar, IntVar, LinearExpr};
pub use model::{Constraint, CpModelBuilder, IntervalVar};
#[cfg(feature = "solve")]
pub use solver::{
    Solution, SolveResponse, SolveStatus, StopToken, Stopper, solve_model_proto,
    solve_model_proto_interruptible,
};

/// The generated OR-Tools proto types this API builds on, for advanced use
/// (inspecting [`CpModelBuilder::proto`], tuning [`SatParameters`], reading
/// [`SolveResponse::raw`](SolveResponse::raw)).
pub use oxidor_protos as protos;

pub use protos::sat::SatParameters;
