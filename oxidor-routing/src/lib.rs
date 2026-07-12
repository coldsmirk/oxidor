//! Idiomatic Rust API for [Google OR-Tools vehicle
//! routing](https://developers.google.com/optimization/routing) — TSP and
//! capacitated VRP over a dense distance matrix.
//!
//! The routing library is imperative C++ with no upstream C API, so this
//! crate ships its own small C shim (compiled by `oxidor-sys` under its
//! `shim` feature). The surface deliberately takes **matrices** (arc costs,
//! travel times) rather than callbacks: no Rust closure ever crosses the FFI
//! boundary, and search parameters travel as a serialized
//! `RoutingSearchParameters` proto, merged over OR-Tools' defaults.
//!
//! Beyond plain TSP/CVRP, a problem can carry a [`TimeDimension`] (travel
//! times, service times, time windows — a VRPTW), pickup-and-delivery pairs,
//! and per-vehicle fixed costs.
//!
//! # Example: TSP
//!
//! ```no_run
//! use oxidor_routing::RoutingProblem;
//!
//! let matrix = vec![
//!     vec![0, 10, 15, 20],
//!     vec![10, 0, 35, 25],
//!     vec![15, 35, 0, 30],
//!     vec![20, 25, 30, 0],
//! ];
//! let response = RoutingProblem::from_matrix(matrix)?.solve()?;
//! let tour = response.solution().expect("a tour was found");
//! println!("tour cost {}: {:?}", tour.objective_value(), tour.routes()[0]);
//! # Ok::<(), oxidor_routing::RoutingError>(())
//! ```
//!
//! For a capacitated VRP, add vehicles and a capacity dimension:
//!
//! ```no_run
//! # use oxidor_routing::RoutingProblem;
//! # let matrix = vec![vec![0; 5]; 5];
//! let response = RoutingProblem::from_matrix(matrix)?
//!     .with_vehicles(2)
//!     .with_capacities(vec![0, 1, 1, 1, 1], vec![2, 2])
//!     .solve()?;
//! # Ok::<(), oxidor_routing::RoutingError>(())
//! ```
//!
//! # Features
//!
//! - `solve` *(default)* — links the native OR-Tools library and compiles the
//!   C++ shim (needs the OR-Tools headers and a C++20 compiler). Without it
//!   the crate only defines the problem types.

#![warn(missing_docs)]

mod problem;
#[cfg(feature = "solve")]
mod solve;

pub use problem::{RoutingError, RoutingProblem, TimeDimension};
#[cfg(feature = "solve")]
pub use solve::{RoutingResponse, RoutingSolution, RoutingStatus};

/// The generated OR-Tools proto types this API builds on.
pub use oxidor_protos as protos;

pub use protos::operations_research::RoutingSearchParameters;
