use core::ffi::{c_int, c_void};

use oxidor_protos::prost::Message;
use oxidor_protos::sat::{CpSolverResponse, CpSolverStatus, SatParameters};

use crate::expr::{BoolVar, LinearExpr};
use crate::model::CpModelBuilder;

impl CpModelBuilder {
    /// Solves the model with default parameters.
    pub fn solve(&self) -> SolveResponse {
        self.solve_with_parameters(&SatParameters::default())
    }

    /// Solves the model with the given [`SatParameters`] (time limits, worker
    /// count, search logging, …).
    ///
    /// ```no_run
    /// use oxidor_cpsat::{CpModelBuilder, SatParameters};
    ///
    /// # let model = CpModelBuilder::new();
    /// let params = SatParameters {
    ///     max_time_in_seconds: Some(10.0),
    ///     ..Default::default()
    /// };
    /// let response = model.solve_with_parameters(&params);
    /// ```
    pub fn solve_with_parameters(&self, parameters: &SatParameters) -> SolveResponse {
        let response = solve_serialized(&self.proto().encode_to_vec(), &parameters.encode_to_vec());
        SolveResponse { proto: response }
    }
}

/// Calls the native solver over the CP-SAT C API: serialized protos in,
/// serialized proto out.
fn solve_serialized(model_bytes: &[u8], parameter_bytes: &[u8]) -> CpSolverResponse {
    let mut response_pointer: *mut c_void = core::ptr::null_mut();
    let mut response_length: c_int = 0;
    // SAFETY: the input pointers are valid for their stated lengths and hold
    // protos we just encoded; the output pointer/length pair is written by the
    // call before it returns.
    unsafe {
        oxidor_sys::SolveCpModelWithParameters(
            model_bytes.as_ptr().cast(),
            model_bytes.len() as c_int,
            parameter_bytes.as_ptr().cast(),
            parameter_bytes.len() as c_int,
            &mut response_pointer,
            &mut response_length,
        );
    }
    // SAFETY: the solver hands us a malloc-allocated buffer of exactly
    // response_length bytes; we copy it out and release it with the C
    // allocator, as the API requires.
    let response_bytes = unsafe {
        core::slice::from_raw_parts(response_pointer.cast::<u8>(), response_length as usize)
    };
    let response = CpSolverResponse::decode(response_bytes)
        .expect("OR-Tools returned an undecodable CpSolverResponse; version mismatch between oxidor-protos and the linked library");
    unsafe { libc::free(response_pointer) };
    response
}

/// The outcome of a solve: a status, timing and search statistics, and — when
/// the status says so — a solution.
#[derive(Debug, Clone)]
pub struct SolveResponse {
    proto: CpSolverResponse,
}

impl SolveResponse {
    /// What the search established.
    pub fn status(&self) -> SolveStatus {
        match self.proto.status() {
            CpSolverStatus::Unknown => SolveStatus::Unknown,
            CpSolverStatus::ModelInvalid => SolveStatus::ModelInvalid,
            CpSolverStatus::Feasible => SolveStatus::Feasible,
            CpSolverStatus::Infeasible => SolveStatus::Infeasible,
            CpSolverStatus::Optimal => SolveStatus::Optimal,
        }
    }

    /// The solution, when one was found ([`Optimal`](SolveStatus::Optimal) or
    /// [`Feasible`](SolveStatus::Feasible)).
    pub fn solution(&self) -> Option<Solution<'_>> {
        match self.status() {
            SolveStatus::Optimal | SolveStatus::Feasible => Some(Solution { proto: &self.proto }),
            _ => None,
        }
    }

    /// The objective value of the returned solution.
    pub fn objective_value(&self) -> f64 {
        self.proto.objective_value
    }

    /// The best proven bound on the objective.
    pub fn best_objective_bound(&self) -> f64 {
        self.proto.best_objective_bound
    }

    /// Wall-clock seconds spent in the solve.
    pub fn wall_time(&self) -> f64 {
        self.proto.wall_time
    }

    /// The raw `CpSolverResponse` for statistics this wrapper does not surface.
    pub fn raw(&self) -> &CpSolverResponse {
        &self.proto
    }
}

/// How a solve ended. Statuses are outcomes, not errors — an infeasible
/// roster is an answer, not a failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SolveStatus {
    /// Search stopped (e.g. by a time limit) before reaching any conclusion.
    Unknown,
    /// The model failed validation; see the solver log for the reason.
    ModelInvalid,
    /// A solution was found, but optimality was not proven.
    Feasible,
    /// The model was proven to have no solution.
    Infeasible,
    /// An optimal solution was found (or, without an objective, the
    /// feasibility question was fully settled).
    Optimal,
}

/// A variable assignment satisfying the model, borrowed from a
/// [`SolveResponse`].
#[derive(Debug, Clone, Copy)]
pub struct Solution<'response> {
    proto: &'response CpSolverResponse,
}

impl Solution<'_> {
    /// The value of a variable or linear expression under this solution.
    pub fn value(&self, expr: impl Into<LinearExpr>) -> i64 {
        let expr = expr.into();
        let terms: i64 = expr
            .terms
            .iter()
            .map(|&(var, coeff)| coeff * self.proto.solution[var as usize])
            .sum();
        terms + expr.constant
    }

    /// Whether a Boolean literal is true under this solution.
    pub fn boolean_value(&self, literal: BoolVar) -> bool {
        self.value(literal) == 1
    }
}
