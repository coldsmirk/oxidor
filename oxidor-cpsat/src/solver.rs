use core::ffi::{c_int, c_void};
use std::sync::Arc;
use std::time::Duration;

use oxidor_protos::prost::Message;
use oxidor_protos::sat::{CpSolverResponse, CpSolverStatus, SatParameters};

use crate::expr::{BoolVar, LinearExpr};
use crate::model::CpModelBuilder;

impl CpModelBuilder {
    /// Solves the model with default parameters.
    ///
    /// # Panics
    ///
    /// Panics if the serialized model exceeds 2 GiB (a limit of the CP-SAT
    /// C API) — as do all the other solve methods.
    pub fn solve(&self) -> SolveResponse {
        self.solve_with_parameters(&SatParameters::default())
    }

    /// Solves the model, giving up after `limit` and returning the best
    /// solution found so far ([`Feasible`](SolveStatus::Feasible)) or
    /// [`Unknown`](SolveStatus::Unknown) when none was.
    ///
    /// ```no_run
    /// use std::time::Duration;
    /// # let model = oxidor_cpsat::CpModelBuilder::new();
    /// let response = model.solve_with_time_limit(Duration::from_secs(10));
    /// ```
    ///
    /// For any other tuning knob, reach for
    /// [`solve_with_parameters`](Self::solve_with_parameters).
    pub fn solve_with_time_limit(&self, limit: Duration) -> SolveResponse {
        self.solve_with_parameters(&SatParameters {
            max_time_in_seconds: Some(limit.as_secs_f64()),
            ..Default::default()
        })
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
    ///     num_workers: Some(8),
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
        SolveResponse {
            model: self.id(),
            proto: response,
        }
    }

    /// Solves the model; the search can be stopped early from another thread
    /// through a [`Stopper`] obtained from the [`StopToken`].
    ///
    /// The exclusive borrow enforces the C API's contract that one stop
    /// environment drives at most one solve at a time.
    ///
    /// ```no_run
    /// use oxidor_cpsat::{CpModelBuilder, StopToken};
    ///
    /// # let model = CpModelBuilder::new();
    /// let mut token = StopToken::new();
    /// let stopper = token.stopper();
    /// std::thread::spawn(move || {
    ///     std::thread::sleep(std::time::Duration::from_secs(5));
    ///     stopper.stop();
    /// });
    /// let response = model.solve_interruptible(&mut token);
    /// ```
    pub fn solve_interruptible(&self, token: &mut StopToken) -> SolveResponse {
        self.solve_interruptible_with_parameters(token, &SatParameters::default())
    }

    /// Like [`solve_interruptible`](Self::solve_interruptible), with explicit
    /// [`SatParameters`].
    pub fn solve_interruptible_with_parameters(
        &self,
        token: &mut StopToken,
        parameters: &SatParameters,
    ) -> SolveResponse {
        let response = solve_serialized(
            &self.proto().encode_to_vec(),
            &parameters.encode_to_vec(),
            Some(token),
        );
        SolveResponse {
            model: self.id(),
            proto: response,
        }
    }
}

/// Drives interruptible solves
/// ([`solve_interruptible`](CpModelBuilder::solve_interruptible)); hand out
/// [`Stopper`]s to whoever makes the stop decision.
///
/// A token backs **one solve at a time** — the solve methods take `&mut self`
/// to enforce that at compile time. Once stopped, a token stays stopped: a
/// solve started with it afterwards returns immediately, so create a fresh
/// token per solve.
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
            environment: Arc::new(SolveEnvironment { pointer }),
        }
    }

    /// A cloneable handle that can stop this token's solve from anywhere —
    /// another thread, a signal handler bridge, a timeout task.
    pub fn stopper(&self) -> Stopper {
        Stopper {
            environment: Arc::clone(&self.environment),
        }
    }

    /// Stops the token before a solve even starts (equivalent to
    /// [`Stopper::stop`]).
    pub fn stop(&self) {
        self.environment.stop();
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

/// Stops the running solve of the [`StopToken`] it was created from.
///
/// Cheap to clone; send clones wherever the stop decision is made. Stopping
/// is idempotent, and stopping when no solve is running simply makes the
/// token's next solve return immediately.
#[derive(Clone)]
pub struct Stopper {
    environment: Arc<SolveEnvironment>,
}

impl Stopper {
    /// Asks the solve driven by this token to stop as soon as possible. The
    /// solve returns with the best solution found so far (status
    /// [`Feasible`](SolveStatus::Feasible) or
    /// [`Unknown`](SolveStatus::Unknown)).
    pub fn stop(&self) {
        self.environment.stop();
    }
}

impl std::fmt::Debug for Stopper {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.debug_struct("Stopper").finish_non_exhaustive()
    }
}

/// Owns the C-side solve environment behind a [`StopToken`] / [`Stopper`].
struct SolveEnvironment {
    /// Set at creation, never reassigned; only the C side dereferences it.
    pointer: *mut c_void,
}

impl SolveEnvironment {
    fn stop(&self) {
        // SAFETY: the pointer is live while any StopToken/Stopper holds the
        // Arc; the C side explicitly supports stopping from a thread other
        // than the solver's (it only flips an internally synchronized time
        // limit).
        unsafe { oxidor_sys::SolveCpStopSearch(self.pointer) };
    }
}

// SAFETY: the pointer value is immutable after creation, so sharing it needs
// no Rust-side synchronization. Of the two C entry points that take it,
// SolveCpStopSearch is documented thread-safe against a running solve, and
// SolveCpInterruptible's "one solve at a time" contract is enforced at
// compile time: only `&mut StopToken` reaches it, and StopToken is not Clone
// (Stopper clones can merely stop). Destruction happens after the last Arc
// drops, when no borrow can still be alive.
unsafe impl Send for SolveEnvironment {}
unsafe impl Sync for SolveEnvironment {}

impl Drop for SolveEnvironment {
    fn drop(&mut self) {
        // SAFETY: dropping the last Arc means no solve or stop can still use
        // the environment; destroy is called exactly once.
        unsafe { oxidor_sys::SolveCpDestroyEnv(self.pointer) };
    }
}

/// A buffer length as the `c_int` the C API takes; the API cannot represent
/// larger inputs, so exceeding it is a documented panic.
fn c_length(bytes: &[u8]) -> c_int {
    c_int::try_from(bytes.len())
        .expect("the serialized input exceeds the CP-SAT C API's 2 GiB limit")
}

/// Copies a malloc-allocated C buffer into owned memory and frees it.
fn take_c_buffer(pointer: *mut c_void, length: c_int) -> Vec<u8> {
    if pointer.is_null() || length <= 0 {
        return Vec::new();
    }
    // SAFETY: non-null with a positive length means the C side handed us a
    // readable buffer of exactly `length` bytes that we own; we copy it out
    // and release it with the C allocator, as the API requires.
    unsafe {
        let bytes = core::slice::from_raw_parts(pointer.cast::<u8>(), length as usize).to_vec();
        libc::free(pointer);
        bytes
    }
}

/// Calls the native solver over the CP-SAT C API: serialized protos in,
/// serialized proto out.
fn solve_serialized(
    model_bytes: &[u8],
    parameter_bytes: &[u8],
    token: Option<&mut StopToken>,
) -> CpSolverResponse {
    let mut response_pointer: *mut c_void = core::ptr::null_mut();
    let mut response_length: c_int = 0;
    // SAFETY: the input pointers are valid for their stated lengths and hold
    // protos we just encoded; the output pointer/length pair is written by the
    // call before it returns; an environment pointer, when present, is kept
    // live by the exclusive token borrow for the whole call.
    unsafe {
        match token {
            None => oxidor_sys::SolveCpModelWithParameters(
                model_bytes.as_ptr().cast(),
                c_length(model_bytes),
                parameter_bytes.as_ptr().cast(),
                c_length(parameter_bytes),
                &mut response_pointer,
                &mut response_length,
            ),
            Some(token) => oxidor_sys::SolveCpInterruptible(
                token.environment.pointer,
                model_bytes.as_ptr().cast(),
                c_length(model_bytes),
                parameter_bytes.as_ptr().cast(),
                c_length(parameter_bytes),
                &mut response_pointer,
                &mut response_length,
            ),
        }
    }
    // Free before decoding so a decode panic cannot leak the C buffer.
    let response_bytes = take_c_buffer(response_pointer, response_length);
    CpSolverResponse::decode(response_bytes.as_slice())
        .expect("OR-Tools returned an undecodable CpSolverResponse; version mismatch between oxidor-protos and the linked library")
}

/// The outcome of a solve: a status, timing and search statistics, and — when
/// the status says so — one or more solutions.
#[derive(Debug, Clone)]
pub struct SolveResponse {
    model: u32,
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
                model: self.model,
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
                        model: self.model,
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
#[non_exhaustive]
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
    model: u32,
    values: &'response [i64],
}

impl Solution<'_> {
    /// The value of a variable or linear expression under this solution.
    ///
    /// # Panics
    ///
    /// Panics on a handle from a different model, or on one created after
    /// the solve that produced this solution — both programmer errors.
    #[track_caller]
    pub fn value(&self, expr: impl Into<LinearExpr>) -> i64 {
        let expr = expr.into();
        assert!(
            expr.model.is_none_or(|model| model == self.model),
            "the expression uses variables from a different model",
        );
        let terms: i64 = expr
            .terms
            .iter()
            .map(|&(var, coeff)| {
                let value = self.values.get(var as usize).copied().unwrap_or_else(|| {
                    panic!("the variable was created after the solve that produced this solution")
                });
                coeff * value
            })
            .sum();
        terms + expr.constant
    }

    /// Whether a Boolean literal is true under this solution.
    ///
    /// # Panics
    ///
    /// As for [`value`](Self::value).
    #[track_caller]
    pub fn bool_value(&self, literal: BoolVar) -> bool {
        self.value(literal) == 1
    }
}
