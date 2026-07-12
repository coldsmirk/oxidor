use core::ffi::{CStr, c_char, c_void};
use std::sync::Arc;
use std::time::Duration;

use oxidor_protos::math_opt::{
    PrimalSolutionProto, SolutionStatusProto, SolveParametersProto, SolveResultProto,
    TerminationReasonProto,
};
use oxidor_protos::prost::Message;

use crate::expr::LinearExpr;
use crate::model::Model;

impl Model {
    /// Solves the model with the given solver and default parameters.
    ///
    /// Model shortcomings (infeasible, unbounded, …) are *outcomes*, reported
    /// through [`SolveResult::status`]; `Err` means the solve could not run
    /// at all — most commonly a solver not compiled into the linked OR-Tools
    /// library (see [`registered_solvers`]), or an invalid model.
    pub fn solve(&self, solver: SolverType) -> Result<SolveResult, SolveError> {
        self.solve_with_everything(solver, None, None)
    }

    /// Solves the model with the given [`SolveParametersProto`] (time and
    /// solution limits, gap tolerances, thread count, seed, …).
    ///
    /// ```no_run
    /// use oxidor_mathopt::{Model, SolveParametersProto, SolverType};
    ///
    /// # let model = Model::new();
    /// let parameters = SolveParametersProto {
    ///     threads: Some(4),
    ///     relative_gap_tolerance: Some(1e-4),
    ///     ..Default::default()
    /// };
    /// let result = model.solve_with_parameters(SolverType::Gscip, &parameters);
    /// ```
    pub fn solve_with_parameters(
        &self,
        solver: SolverType,
        parameters: &SolveParametersProto,
    ) -> Result<SolveResult, SolveError> {
        self.solve_with_everything(solver, Some(parameters), None)
    }

    /// Solves the model, giving up after `limit` and returning the best
    /// solution found so far ([`Feasible`](TerminationReason::Feasible)) or
    /// [`NoSolutionFound`](TerminationReason::NoSolutionFound) when none was.
    pub fn solve_with_time_limit(
        &self,
        solver: SolverType,
        limit: Duration,
    ) -> Result<SolveResult, SolveError> {
        let parameters = SolveParametersProto {
            time_limit: Some(oxidor_protos::prost_types::Duration {
                seconds: i64::try_from(limit.as_secs()).unwrap_or(i64::MAX),
                nanos: limit.subsec_nanos() as i32,
            }),
            ..Default::default()
        };
        self.solve_with_everything(solver, Some(&parameters), None)
    }

    /// Like [`solve`](Model::solve); the solve can be interrupted from
    /// another thread through the [`SolveInterrupter`].
    pub fn solve_interruptible(
        &self,
        solver: SolverType,
        interrupter: &SolveInterrupter,
    ) -> Result<SolveResult, SolveError> {
        self.solve_with_everything(solver, None, Some(interrupter))
    }

    /// Like [`solve_with_parameters`](Model::solve_with_parameters), with an
    /// interrupter.
    pub fn solve_interruptible_with_parameters(
        &self,
        solver: SolverType,
        interrupter: &SolveInterrupter,
        parameters: &SolveParametersProto,
    ) -> Result<SolveResult, SolveError> {
        self.solve_with_everything(solver, Some(parameters), Some(interrupter))
    }

    fn solve_with_everything(
        &self,
        solver: SolverType,
        parameters: Option<&SolveParametersProto>,
        interrupter: Option<&SolveInterrupter>,
    ) -> Result<SolveResult, SolveError> {
        let parameter_bytes = parameters.map(Message::encode_to_vec).unwrap_or_default();
        let proto = solve_serialized(
            &self.proto().encode_to_vec(),
            solver,
            &parameter_bytes,
            interrupter,
        )?;
        Ok(SolveResult {
            model: self.id(),
            proto,
        })
    }
}

/// The underlying solver MathOpt dispatches to.
///
/// Availability depends on what the linked OR-Tools library was built with.
/// The official release archives include [`Gscip`](Self::Gscip) (SCIP, for
/// MIP), [`Glop`](Self::Glop) (simplex LP), [`CpSat`](Self::CpSat) (integer
/// models), and [`Pdlp`](Self::Pdlp) (first-order LP).
///
/// Selecting a solver the linked library does not register fails cleanly
/// with a [`SolveError`]; probe availability up front with
/// [`registered_solvers`].
///
/// The discriminants are the wire values of MathOpt's `SolverTypeProto`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
#[repr(i32)]
pub enum SolverType {
    /// SCIP via GScip — mixed-integer programming.
    Gscip = 1,
    /// Gurobi (needs a commercial license and a Gurobi-enabled build).
    Gurobi = 2,
    /// Glop — Google's simplex LP solver.
    Glop = 3,
    /// CP-SAT — integer models with integer data.
    CpSat = 4,
    /// PDLP — first-order LP for very large problems.
    Pdlp = 5,
    /// GLPK (build-dependent).
    Glpk = 6,
    /// OSQP — quadratic objectives (build-dependent).
    Osqp = 7,
    /// ECOS (build-dependent).
    Ecos = 8,
    /// SCS (build-dependent).
    Scs = 9,
    /// HiGHS (build-dependent).
    Highs = 10,
    /// FICO Xpress (build-dependent).
    Xpress = 13,
}

/// A failure to run the solve at all (as opposed to an unfavorable
/// [`TerminationReason`], which is a result).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct SolveError {
    /// The [`absl::StatusCode`] numeric value reported by OR-Tools.
    ///
    /// [`absl::StatusCode`]: https://abseil.io/docs/cpp/guides/status-codes
    pub code: i32,
    /// Human-readable description from the solver layer.
    pub message: String,
}

impl std::fmt::Display for SolveError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "MathOpt solve failed (status code {}): {}",
            self.code, self.message
        )
    }
}

impl std::error::Error for SolveError {}

/// Interrupts MathOpt solves from another thread.
///
/// Clone it and move the clone wherever the stop decision is made. One
/// interrupter may serve several (even concurrent) solves; once triggered it
/// stays triggered, and solves started with it return immediately.
#[derive(Clone)]
pub struct SolveInterrupter {
    handle: Arc<InterrupterHandle>,
}

impl SolveInterrupter {
    /// A fresh, untriggered interrupter.
    pub fn new() -> Self {
        // SAFETY: no preconditions; the pointer is owned by InterrupterHandle
        // and freed exactly once on drop.
        let pointer = unsafe { oxidor_sys::OxidorMathOptNewInterrupter() };
        assert!(!pointer.is_null(), "allocating a SolveInterrupter failed");
        Self {
            handle: Arc::new(InterrupterHandle { pointer }),
        }
    }

    /// Asks every solve using this interrupter to stop as soon as possible.
    /// Idempotent and thread-safe.
    pub fn interrupt(&self) {
        // SAFETY: the pointer is non-null and lives while any clone exists;
        // the underlying SolveInterrupter documents triggering as
        // thread-safe.
        unsafe { oxidor_sys::OxidorMathOptInterrupt(self.handle.pointer) };
    }

    /// Whether [`interrupt`](Self::interrupt) has been called.
    pub fn is_interrupted(&self) -> bool {
        // SAFETY: as for `interrupt`.
        unsafe { oxidor_sys::OxidorMathOptIsInterrupted(self.handle.pointer) != 0 }
    }
}

impl Default for SolveInterrupter {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for SolveInterrupter {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SolveInterrupter")
            .field("is_interrupted", &self.is_interrupted())
            .finish()
    }
}

struct InterrupterHandle {
    pointer: *mut c_void,
}

// SAFETY: the underlying SolveInterrupter documents trigger/check as
// thread-safe and permits sharing one interrupter across concurrent solves;
// the pointer value itself is immutable after creation.
unsafe impl Send for InterrupterHandle {}
unsafe impl Sync for InterrupterHandle {}

impl Drop for InterrupterHandle {
    fn drop(&mut self) {
        // SAFETY: dropping the last Arc; the interrupter outlived every solve
        // borrowing it (solves hold a &SolveInterrupter for their duration).
        unsafe { oxidor_sys::OxidorMathOptFreeInterrupter(self.pointer) };
    }
}

/// Copies a malloc-allocated C buffer into owned memory and frees it.
fn take_c_buffer(pointer: *mut c_void, length: usize) -> Vec<u8> {
    if pointer.is_null() || length == 0 {
        return Vec::new();
    }
    // SAFETY: non-null with a nonzero length means the C side handed us a
    // readable buffer of exactly `length` bytes that we own; we copy it out
    // and release it with the C allocator, as the shim contract requires.
    unsafe {
        let bytes = core::slice::from_raw_parts(pointer.cast::<u8>(), length).to_vec();
        libc::free(pointer);
        bytes
    }
}

/// Copies a malloc-allocated C error string into owned memory and frees it.
fn take_c_error(pointer: *mut c_char) -> String {
    if pointer.is_null() {
        return String::new();
    }
    // SAFETY: a non-null error from the shim is a malloc-allocated,
    // null-terminated string we own.
    unsafe {
        let message = CStr::from_ptr(pointer).to_string_lossy().into_owned();
        libc::free(pointer.cast());
        message
    }
}

fn solve_serialized(
    model_bytes: &[u8],
    solver: SolverType,
    parameter_bytes: &[u8],
    interrupter: Option<&SolveInterrupter>,
) -> Result<SolveResultProto, SolveError> {
    let mut result_pointer: *mut c_void = core::ptr::null_mut();
    let mut result_length: usize = 0;
    let mut error_pointer: *mut c_char = core::ptr::null_mut();

    // SAFETY: the buffers are valid for their lengths and hold protos we just
    // encoded; the interrupter, when present, outlives the call via the
    // borrow; output locations are valid for writes.
    let status_code = unsafe {
        oxidor_sys::OxidorMathOptSolveWithParameters(
            model_bytes.as_ptr().cast(),
            model_bytes.len(),
            solver as i32,
            parameter_bytes.as_ptr().cast(),
            i32::try_from(parameter_bytes.len())
                .expect("the serialized parameters exceed the shim's 2 GiB limit"),
            interrupter.map_or(core::ptr::null(), |i| i.handle.pointer.cast_const()),
            &mut result_pointer,
            &mut result_length,
            &mut error_pointer,
        )
    };

    if status_code != 0 {
        return Err(SolveError {
            code: status_code,
            message: take_c_error(error_pointer),
        });
    }

    // Free before decoding so a decode panic cannot leak the C buffer.
    let result_bytes = take_c_buffer(result_pointer, result_length);
    Ok(SolveResultProto::decode(result_bytes.as_slice()).expect(
        "OR-Tools returned an undecodable SolveResultProto; version mismatch between oxidor-protos and the linked library",
    ))
}

/// The MathOpt solvers registered in the linked OR-Tools library — the ones
/// a [`Model::solve`] can actually dispatch to.
///
/// The official release archives register SCIP, Glop, CP-SAT, PDLP, GLPK,
/// OSQP, and HiGHS; commercial backends (Gurobi, Xpress) and future solver
/// types only appear in builds that include them.
///
/// # Panics
///
/// Panics if the registry cannot be read (an out-of-memory or C++ exception
/// inside OR-Tools).
pub fn registered_solvers() -> Vec<SolverType> {
    let mut length: i32 = 0;
    let mut error_pointer: *mut c_char = core::ptr::null_mut();
    // SAFETY: the output locations are valid for writes; the returned buffer
    // is copied and freed by take_c_buffer's contract below.
    let pointer =
        unsafe { oxidor_sys::OxidorMathOptRegisteredSolvers(&mut length, &mut error_pointer) };
    if pointer.is_null() {
        panic!(
            "listing the registered MathOpt solvers failed natively: {}",
            take_c_error(error_pointer),
        );
    }
    // SAFETY: non-null means a readable buffer of exactly `length` i32s that
    // we own; copy it out and release it with the C allocator.
    let values = unsafe {
        let values =
            core::slice::from_raw_parts(pointer, usize::try_from(length).unwrap_or(0)).to_vec();
        libc::free(pointer.cast());
        values
    };
    values
        .into_iter()
        .filter_map(SolverType::from_wire)
        .collect()
}

impl SolverType {
    /// Maps a `SolverTypeProto` wire value onto the enum; `None` for values
    /// this crate does not know (a newer library's solvers).
    fn from_wire(value: i32) -> Option<Self> {
        Some(match value {
            1 => Self::Gscip,
            2 => Self::Gurobi,
            3 => Self::Glop,
            4 => Self::CpSat,
            5 => Self::Pdlp,
            6 => Self::Glpk,
            7 => Self::Osqp,
            8 => Self::Ecos,
            9 => Self::Scs,
            10 => Self::Highs,
            13 => Self::Xpress,
            _ => return None,
        })
    }
}

/// The outcome of a MathOpt solve: a termination reason and any solutions.
#[derive(Debug, Clone)]
pub struct SolveResult {
    model: u32,
    proto: SolveResultProto,
}

impl SolveResult {
    /// How the solve ended — why the solver stopped.
    pub fn status(&self) -> TerminationReason {
        let reason = self
            .proto
            .termination
            .as_ref()
            .map(|termination| termination.reason())
            .unwrap_or(TerminationReasonProto::TerminationReasonUnspecified);
        match reason {
            TerminationReasonProto::TerminationReasonOptimal => TerminationReason::Optimal,
            TerminationReasonProto::TerminationReasonInfeasible => TerminationReason::Infeasible,
            TerminationReasonProto::TerminationReasonUnbounded => TerminationReason::Unbounded,
            TerminationReasonProto::TerminationReasonInfeasibleOrUnbounded => {
                TerminationReason::InfeasibleOrUnbounded
            }
            TerminationReasonProto::TerminationReasonImprecise => TerminationReason::Imprecise,
            TerminationReasonProto::TerminationReasonFeasible => TerminationReason::Feasible,
            TerminationReasonProto::TerminationReasonNoSolutionFound => {
                TerminationReason::NoSolutionFound
            }
            TerminationReasonProto::TerminationReasonNumericalError => {
                TerminationReason::NumericalError
            }
            TerminationReasonProto::TerminationReasonOtherError
            | TerminationReasonProto::TerminationReasonUnspecified => TerminationReason::Other,
        }
    }

    /// Solver-specific detail accompanying the termination, often empty.
    pub fn detail(&self) -> &str {
        self.proto
            .termination
            .as_ref()
            .map(|termination| termination.detail.as_str())
            .unwrap_or_default()
    }

    /// The best *feasible* primal solution, when the solver found one
    /// ([`Optimal`](TerminationReason::Optimal) or
    /// [`Feasible`](TerminationReason::Feasible), and occasionally alongside
    /// other reasons).
    ///
    /// Solutions the solver marked infeasible or undetermined — e.g. the
    /// slightly-infeasible answers of an [`Imprecise`](TerminationReason::Imprecise)
    /// termination — are not returned here; they remain accessible through
    /// [`raw`](Self::raw).
    pub fn primal_solution(&self) -> Option<PrimalSolution<'_>> {
        // The result contract orders primal-feasible solutions first.
        self.proto
            .solutions
            .iter()
            .filter_map(|solution| solution.primal_solution.as_ref())
            .find(|primal| {
                primal.feasibility_status() == SolutionStatusProto::SolutionStatusFeasible
            })
            .map(|proto| PrimalSolution {
                model: self.model,
                proto,
            })
    }

    /// The raw `SolveResultProto` for anything this wrapper does not
    /// surface (dual solutions, rays, solve stats, infeasible solutions).
    pub fn raw(&self) -> &SolveResultProto {
        &self.proto
    }
}

/// Why a solve stopped. Unfavorable reasons are outcomes, not errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum TerminationReason {
    /// A provably optimal solution (within tolerances) was found.
    Optimal,
    /// The problem has no feasible solution.
    Infeasible,
    /// The objective can be improved without bound.
    Unbounded,
    /// The problem was shown to be infeasible or unbounded, undistinguished.
    InfeasibleOrUnbounded,
    /// The solver stopped near optimality without meeting its tolerances.
    Imprecise,
    /// A feasible solution was found before a limit (time, interruption) hit.
    Feasible,
    /// A limit hit before any solution was found.
    NoSolutionFound,
    /// The solve failed numerically.
    NumericalError,
    /// Any other termination.
    Other,
}

/// A primal variable assignment, borrowed from a [`SolveResult`].
#[derive(Debug, Clone, Copy)]
pub struct PrimalSolution<'result> {
    model: u32,
    proto: &'result PrimalSolutionProto,
}

impl PrimalSolution<'_> {
    /// The objective value as computed by the solver.
    pub fn objective_value(&self) -> f64 {
        self.proto.objective_value
    }

    /// The value of a variable or linear expression under this solution.
    ///
    /// Variables missing from the solver's (sparse) answer evaluate to `0.0`;
    /// solvers report every model variable unless explicitly filtered.
    ///
    /// # Panics
    ///
    /// Panics on a handle from a different model — a programmer error.
    #[track_caller]
    pub fn value(&self, expr: impl Into<LinearExpr>) -> f64 {
        let expr = expr.into();
        assert!(
            expr.model.is_none_or(|model| model == self.model),
            "the expression uses variables from a different model",
        );
        let values = self.proto.variable_values.as_ref();
        let lookup = |id: i64| -> f64 {
            let Some(values) = values else { return 0.0 };
            match values.ids.binary_search(&id) {
                Ok(index) => values.values[index],
                Err(_) => 0.0,
            }
        };
        let terms: f64 = expr
            .terms
            .iter()
            .map(|&(id, coefficient)| coefficient * lookup(id))
            .sum();
        terms + expr.constant
    }
}
