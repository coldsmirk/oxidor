//! Google OR-Tools' classic standalone algorithms for Rust: 0-1 knapsack,
//! maximum flow, minimum-cost flow, and linear sum assignment.
//!
//! These solvers are small imperative C++ classes with no upstream C API, so
//! they go through Oxidor's own C shim (compiled by `oxidor-sys` under its
//! `shim` feature — needs the OR-Tools headers and a C++20 compiler). Only
//! flat arrays cross the FFI boundary, and every native entry point catches
//! C++ exceptions.
//!
//! - [`solve_knapsack`] / [`solve_knapsack_multidimensional`] — branch and
//!   bound 0-1 knapsack.
//! - [`MaxFlow`] — maximum flow on a directed graph.
//! - [`MinCostFlow`] — minimum-cost flow with supplies and demands.
//! - [`LinearSumAssignment`] — minimum-cost perfect matching.

#![warn(missing_docs)]

mod assignment;
mod error;
mod flow;
mod knapsack;

pub use assignment::{
    AssignmentResponse, AssignmentSolution, AssignmentStatus, LinearSumAssignment,
};
pub use error::AlgorithmError;
pub use flow::{
    ArcId, MaxFlow, MaxFlowSolution, MinCostFlow, MinCostFlowResponse, MinCostFlowSolution,
    MinCostFlowStatus,
};
pub use knapsack::{KnapsackSolution, solve_knapsack, solve_knapsack_multidimensional};
