//! Raw FFI declarations for the Google OR-Tools C APIs — CP-SAT
//! (`ortools/sat/c_api/cp_solver_c.h`) and MathOpt
//! (`ortools/math_opt/core/c_api/solver.h`) — plus the linkage against the
//! native OR-Tools library.
//!
//! Higher-level crates never link OR-Tools directly — they go through this
//! crate. All exchange happens as serialized protobuf bytes; no C++ type
//! crosses the boundary.
//!
//! # Library acquisition
//!
//! Currently the build script links an existing installation located via the
//! `ORTOOLS_PREFIX` environment variable (expects `$ORTOOLS_PREFIX/lib` to
//! contain the OR-Tools shared library, as laid out by the official release
//! archives). Planned additions: `download-prebuilt` (static libraries built
//! by this project's CI) and `vendored` (CMake source build).

#![no_std]

use core::ffi::{c_char, c_int, c_void};

/// Opaque interruption flag for [`MathOptSolve`]; create with
/// [`MathOptNewInterrupter`], release with [`MathOptFreeInterrupter`].
#[repr(C)]
pub struct MathOptInterrupter {
    _opaque: [u8; 0],
    _not_send_sync: core::marker::PhantomData<*mut u8>,
}

unsafe extern "C" {
    /// Solves a serialized `CpModelProto` with serialized `SatParameters`.
    ///
    /// `creq`/`creq_len` and `cparams`/`cparams_len` are the input buffers.
    /// On return, `*cres` points to a `malloc`-allocated buffer of `*cres_len`
    /// bytes holding a serialized `CpSolverResponse`.
    ///
    /// # Safety
    ///
    /// The input buffers must be valid for reads of their stated lengths and
    /// contain parseable protos (the C++ side `CHECK`-aborts on a parse
    /// failure). The caller owns `*cres` and must release it with the C
    /// allocator's `free`.
    pub fn SolveCpModelWithParameters(
        creq: *const c_void,
        creq_len: c_int,
        cparams: *const c_void,
        cparams_len: c_int,
        cres: *mut *mut c_void,
        cres_len: *mut c_int,
    );

    /// Creates a solve environment for use with [`SolveCpInterruptible`].
    ///
    /// # Safety
    ///
    /// The returned pointer is owned by the caller and must be released with
    /// [`SolveCpDestroyEnv`] exactly once.
    pub fn SolveCpNewEnv() -> *mut c_void;

    /// Destroys an environment created by [`SolveCpNewEnv`].
    ///
    /// # Safety
    ///
    /// `cenv` must be a pointer from [`SolveCpNewEnv`] (or null) that has not
    /// already been destroyed, and must not be used afterwards.
    pub fn SolveCpDestroyEnv(cenv: *mut c_void);

    /// Asks the solve running on `cenv` to stop as soon as possible.
    ///
    /// # Safety
    ///
    /// `cenv` must be a live environment from [`SolveCpNewEnv`]. May be called
    /// from a different thread than the solve.
    pub fn SolveCpStopSearch(cenv: *mut c_void);

    /// Like [`SolveCpModelWithParameters`], but runs inside `cenv` so the
    /// search can be interrupted from another thread via
    /// [`SolveCpStopSearch`].
    ///
    /// # Safety
    ///
    /// Same buffer contract as [`SolveCpModelWithParameters`]; `cenv` must be
    /// a live environment from [`SolveCpNewEnv`], used by one solve at a time.
    pub fn SolveCpInterruptible(
        cenv: *mut c_void,
        creq: *const c_void,
        creq_len: c_int,
        cparams: *const c_void,
        cparams_len: c_int,
        cres: *mut *mut c_void,
        cres_len: *mut c_int,
    );
}

unsafe extern "C" {
    /// Returns a new, untriggered interrupter. Release it with
    /// [`MathOptFreeInterrupter`].
    ///
    /// # Safety
    ///
    /// No preconditions. The returned pointer is owned by the caller.
    pub fn MathOptNewInterrupter() -> *mut MathOptInterrupter;

    /// Frees an interrupter; no effect on null.
    ///
    /// # Safety
    ///
    /// `interrupter` must come from [`MathOptNewInterrupter`], must not be
    /// freed twice, and must outlive every [`MathOptSolve`] call using it.
    pub fn MathOptFreeInterrupter(interrupter: *mut MathOptInterrupter);

    /// Triggers the interrupter. Thread-safe; `CHECK`-aborts on null.
    ///
    /// # Safety
    ///
    /// `interrupter` must be a live, non-null interrupter.
    pub fn MathOptInterrupt(interrupter: *mut MathOptInterrupter);

    /// Returns nonzero if triggered. Thread-safe; `CHECK`-aborts on null.
    ///
    /// # Safety
    ///
    /// `interrupter` must be a live, non-null interrupter.
    pub fn MathOptIsInterrupted(interrupter: *const MathOptInterrupter) -> c_int;

    /// Solves a serialized MathOpt `ModelProto` with the solver selected by
    /// `solver_type` (numeric values of `SolverTypeProto`).
    ///
    /// Returns 0 on success, else an `absl::StatusCode`. On success
    /// `*solve_result` holds a serialized `SolveResultProto` of
    /// `*solve_result_size` bytes; on failure `*status_msg` holds a
    /// null-terminated error. Both buffers are released with [`MathOptFree`].
    ///
    /// # Safety
    ///
    /// `model` must be valid for reads of `model_size` bytes. `interrupter`
    /// is either null or a live interrupter that outlives the call. The
    /// output pointers, when non-null, must be valid for writes; the caller
    /// owns whatever they receive.
    pub fn MathOptSolve(
        model: *const c_void,
        model_size: usize,
        solver_type: c_int,
        interrupter: *mut MathOptInterrupter,
        solve_result: *mut *mut c_void,
        solve_result_size: *mut usize,
        status_msg: *mut *mut c_char,
    ) -> c_int;

    /// Frees memory allocated by the MathOpt C API (`solve_result`,
    /// `status_msg`); no effect on null.
    ///
    /// # Safety
    ///
    /// `ptr` must originate from the MathOpt C API and not be freed twice.
    pub fn MathOptFree(ptr: *mut c_void);
}
