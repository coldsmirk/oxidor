use core::ffi::{c_int, c_void};
use std::sync::{Arc, Mutex};

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
        let response = solve_serialized(
            &self.proto().encode_to_vec(),
            &parameters.encode_to_vec(),
            None,
        );
        SolveResponse { proto: response }
    }

    /// Solves the model; the search can be stopped early from another thread
    /// through the [`StopToken`].
    ///
    /// ```no_run
    /// use oxidor_cpsat::{CpModelBuilder, StopToken};
    ///
    /// # let model = CpModelBuilder::new();
    /// let token = StopToken::new();
    /// let stopper = token.clone();
    /// std::thread::spawn(move || {
    ///     std::thread::sleep(std::time::Duration::from_secs(5));
    ///     stopper.stop();
    /// });
    /// let response = model.solve_interruptible(&token);
    /// ```
    pub fn solve_interruptible(&self, token: &StopToken) -> SolveResponse {
        self.solve_interruptible_with_parameters(token, &SatParameters::default())
    }

    /// Like [`solve_interruptible`](Self::solve_interruptible), with explicit
    /// [`SatParameters`].
    pub fn solve_interruptible_with_parameters(
        &self,
        token: &StopToken,
        parameters: &SatParameters,
    ) -> SolveResponse {
        let response = solve_serialized(
            &self.proto().encode_to_vec(),
            &parameters.encode_to_vec(),
            Some(token),
        );
        SolveResponse { proto: response }
    }
}

/// Stops a running [`solve_interruptible`](CpModelBuilder::solve_interruptible)
/// from another thread.
///
/// Clone the token and move the clone wherever the stop decision is made
/// (another thread, a signal handler bridge, a timeout task). Once stopped, a
/// token stays stopped: a solve started with it afterwards returns
/// immediately, so create a fresh token per solve.
#[derive(Clone)]
pub struct StopToken {
    environment: Arc<SolveEnvironment>,
}

impl StopToken {
    /// A fresh token, not yet stopped.
    pub fn new() -> Self {
        // SAFETY: SolveCpNewEnv has no preconditions; the returned pointer is
        // owned by the SolveEnvironment and destroyed exactly once on drop.
        let pointer = unsafe { oxidor_sys::SolveCpNewEnv() };
        Self {
            environment: Arc::new(SolveEnvironment {
                pointer: Mutex::new(pointer),
            }),
        }
    }

    /// Asks the solve driven by this token to stop as soon as possible. The
    /// solve returns with the best solution found so far (status
    /// [`Feasible`](SolveStatus::Feasible) or
    /// [`Unknown`](SolveStatus::Unknown)). Idempotent.
    pub fn stop(&self) {
        let pointer = self.environment.pointer.lock().expect("not poisoned");
        // SAFETY: the pointer is live for as long as any token clone exists;
        // the C side allows stopping from a thread other than the solver's.
        unsafe { oxidor_sys::SolveCpStopSearch(*pointer) };
    }
}

impl Default for StopToken {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for StopToken {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.debug_struct("StopToken").finish_non_exhaustive()
    }
}

/// Owns the C-side solve environment backing a [`StopToken`].
struct SolveEnvironment {
    pointer: Mutex<*mut c_void>,
}

// SAFETY: the raw pointer is only dereferenced by the C API, which
// synchronizes stop requests against the running solve internally (the same
// contract the official Go bindings rely on); Rust-side access to the pointer
// value itself is serialized by the mutex.
unsafe impl Send for SolveEnvironment {}
unsafe impl Sync for SolveEnvironment {}

impl Drop for SolveEnvironment {
    fn drop(&mut self) {
        let pointer = self.pointer.get_mut().expect("not poisoned");
        // SAFETY: dropping the last Arc means no solve or stop can still use
        // the environment; destroy is called exactly once.
        unsafe { oxidor_sys::SolveCpDestroyEnv(*pointer) };
    }
}

/// Calls the native solver over the CP-SAT C API: serialized protos in,
/// serialized proto out.
fn solve_serialized(
    model_bytes: &[u8],
    parameter_bytes: &[u8],
    token: Option<&StopToken>,
) -> CpSolverResponse {
    let mut response_pointer: *mut c_void = core::ptr::null_mut();
    let mut response_length: c_int = 0;
    // SAFETY: the input pointers are valid for their stated lengths and hold
    // protos we just encoded; the output pointer/length pair is written by the
    // call before it returns; an environment pointer, when present, is kept
    // live by the token borrow for the whole call.
    unsafe {
        match token {
            None => oxidor_sys::SolveCpModelWithParameters(
                model_bytes.as_ptr().cast(),
                model_bytes.len() as c_int,
                parameter_bytes.as_ptr().cast(),
                parameter_bytes.len() as c_int,
                &mut response_pointer,
                &mut response_length,
            ),
            Some(token) => {
                let environment = *token.environment.pointer.lock().expect("not poisoned");
                oxidor_sys::SolveCpInterruptible(
                    environment,
                    model_bytes.as_ptr().cast(),
                    model_bytes.len() as c_int,
                    parameter_bytes.as_ptr().cast(),
                    parameter_bytes.len() as c_int,
                    &mut response_pointer,
                    &mut response_length,
                )
            }
        }
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
/// the status says so — one or more solutions.
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

    /// The best solution, when one was found ([`Optimal`](SolveStatus::Optimal)
    /// or [`Feasible`](SolveStatus::Feasible)).
    pub fn solution(&self) -> Option<Solution<'_>> {
        match self.status() {
            SolveStatus::Optimal | SolveStatus::Feasible => Some(Solution {
                values: &self.proto.solution,
            }),
            _ => None,
        }
    }

    /// Every solution carried by the response: the best one first, then the
    /// additional solutions collected during search.
    ///
    /// CP-SAT only stores additional solutions when asked; to enumerate a
    /// feasibility problem's full solution set:
    ///
    /// ```no_run
    /// use oxidor_cpsat::{CpModelBuilder, SatParameters};
    ///
    /// # let model = CpModelBuilder::new();
    /// let params = SatParameters {
    ///     enumerate_all_solutions: Some(true),
    ///     fill_additional_solutions_in_response: Some(true),
    ///     ..Default::default()
    /// };
    /// for solution in model.solve_with_parameters(&params).solutions() {
    ///     // inspect solution.value(...)
    /// }
    /// ```
    pub fn solutions(&self) -> impl Iterator<Item = Solution<'_>> {
        self.solution()
            .into_iter()
            .chain(
                self.proto
                    .additional_solutions
                    .iter()
                    .map(|additional| Solution {
                        values: &additional.values,
                    }),
            )
    }

    /// The objective value of the best solution.
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
    values: &'response [i64],
}

impl Solution<'_> {
    /// The value of a variable or linear expression under this solution.
    pub fn value(&self, expr: impl Into<LinearExpr>) -> i64 {
        let expr = expr.into();
        let terms: i64 = expr
            .terms
            .iter()
            .map(|&(var, coeff)| coeff * self.values[var as usize])
            .sum();
        terms + expr.constant
    }

    /// Whether a Boolean literal is true under this solution.
    pub fn boolean_value(&self, literal: BoolVar) -> bool {
        self.value(literal) == 1
    }
}
