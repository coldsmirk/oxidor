use core::ffi::{CStr, c_char, c_int, c_void};
use std::sync::Arc;

use oxidor_protos::math_opt::{PrimalSolutionProto, SolveResultProto, TerminationReasonProto};
use oxidor_protos::prost::Message;

use crate::expr::LinearExpr;
use crate::model::Model;

impl Model {
    /// Solves the model with the given solver.
    ///
    /// Model shortcomings (infeasible, unbounded, …) are *outcomes*, reported
    /// through [`SolveResult::reason`]; `Err` means the solve could not run
    /// at all — most commonly a solver not compiled into the linked OR-Tools
    /// library, or an invalid model.
    pub fn solve(&self, solver: SolverType) -> Result<SolveResult, SolveError> {
        solve_serialized(&self.proto().encode_to_vec(), solver, None)
    }

    /// Like [`solve`](Model::solve); the solve can be interrupted from
    /// another thread through the [`SolveInterrupter`].
    pub fn solve_interruptible(
        &self,
        solver: SolverType,
        interrupter: &SolveInterrupter,
    ) -> Result<SolveResult, SolveError> {
        solve_serialized(&self.proto().encode_to_vec(), solver, Some(interrupter))
    }
}

/// The underlying solver MathOpt dispatches to.
///
/// Availability depends on what the linked OR-Tools library was built with.
/// The official release archives include [`Gscip`](Self::Gscip) (SCIP, for
/// MIP), [`Glop`](Self::Glop) (simplex LP), [`CpSat`](Self::CpSat) (integer
/// models), and [`Pdlp`](Self::Pdlp) (first-order LP).
///
/// **Warning:** selecting a solver whose backend is *not* compiled into the
/// linked library is not guaranteed to fail cleanly — some registry paths
/// raise a C++ exception, which cannot cross the C boundary and aborts the
/// process. Stick to the solvers your OR-Tools build ships.
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
pub struct SolveError {
    /// The `absl::StatusCode` numeric value reported by OR-Tools.
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
        let pointer = unsafe { oxidor_sys::MathOptNewInterrupter() };
        Self {
            handle: Arc::new(InterrupterHandle { pointer }),
        }
    }

    /// Asks every solve using this interrupter to stop as soon as possible.
    /// Idempotent and thread-safe.
    pub fn interrupt(&self) {
        // SAFETY: the pointer is non-null and lives while any clone exists;
        // the C API documents this call as thread-safe.
        unsafe { oxidor_sys::MathOptInterrupt(self.handle.pointer) };
    }

    /// Whether [`interrupt`](Self::interrupt) has been called.
    pub fn is_interrupted(&self) -> bool {
        // SAFETY: as for `interrupt`.
        unsafe { oxidor_sys::MathOptIsInterrupted(self.handle.pointer) != 0 }
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
    pointer: *mut oxidor_sys::MathOptInterrupter,
}

// SAFETY: the C API documents trigger/check as thread-safe and permits
// sharing one interrupter across concurrent solves; the pointer value itself
// is immutable after creation.
unsafe impl Send for InterrupterHandle {}
unsafe impl Sync for InterrupterHandle {}

impl Drop for InterrupterHandle {
    fn drop(&mut self) {
        // SAFETY: dropping the last Arc; the interrupter outlived every solve
        // borrowing it (solves hold a &SolveInterrupter for their duration).
        unsafe { oxidor_sys::MathOptFreeInterrupter(self.pointer) };
    }
}

fn solve_serialized(
    model_bytes: &[u8],
    solver: SolverType,
    interrupter: Option<&SolveInterrupter>,
) -> Result<SolveResult, SolveError> {
    let mut result_pointer: *mut c_void = core::ptr::null_mut();
    let mut result_length: usize = 0;
    let mut status_message: *mut c_char = core::ptr::null_mut();

    // SAFETY: the model buffer is valid for its length; the interrupter, when
    // present, outlives the call via the borrow; output locations are valid
    // for writes.
    let status_code = unsafe {
        oxidor_sys::MathOptSolve(
            model_bytes.as_ptr().cast(),
            model_bytes.len(),
            solver as c_int,
            interrupter.map_or(core::ptr::null_mut(), |i| i.handle.pointer),
            &mut result_pointer,
            &mut result_length,
            &mut status_message,
        )
    };

    if status_code != 0 {
        // SAFETY: on failure the API hands us a null-terminated message we
        // own; copy it out and free it with MathOptFree.
        let message = unsafe {
            let message = if status_message.is_null() {
                String::new()
            } else {
                CStr::from_ptr(status_message)
                    .to_string_lossy()
                    .into_owned()
            };
            oxidor_sys::MathOptFree(status_message.cast());
            message
        };
        return Err(SolveError {
            code: status_code,
            message,
        });
    }

    // SAFETY: on success the API hands us a buffer of exactly result_length
    // bytes; copy it out and free it with MathOptFree.
    let result_bytes =
        unsafe { core::slice::from_raw_parts(result_pointer.cast::<u8>(), result_length) };
    let proto = SolveResultProto::decode(result_bytes).expect(
        "OR-Tools returned an undecodable SolveResultProto; version mismatch between oxidor-protos and the linked library",
    );
    unsafe { oxidor_sys::MathOptFree(result_pointer) };

    Ok(SolveResult { proto })
}

/// The outcome of a MathOpt solve: a termination reason and any solutions.
#[derive(Debug, Clone)]
pub struct SolveResult {
    proto: SolveResultProto,
}

impl SolveResult {
    /// Why the solver stopped.
    pub fn reason(&self) -> TerminationReason {
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

    /// The best primal solution, when the solver found one
    /// ([`Optimal`](TerminationReason::Optimal) or
    /// [`Feasible`](TerminationReason::Feasible), and occasionally alongside
    /// other reasons).
    pub fn primal_solution(&self) -> Option<PrimalSolution<'_>> {
        // The result contract orders primal-feasible solutions first.
        self.proto
            .solutions
            .iter()
            .find_map(|solution| solution.primal_solution.as_ref())
            .map(|proto| PrimalSolution { proto })
    }

    /// The raw `SolveResultProto` for statistics this wrapper does not
    /// surface (dual solutions, rays, solve stats).
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
    pub fn value(&self, expr: impl Into<LinearExpr>) -> f64 {
        let expr = expr.into();
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
