//! Raw FFI declarations for the Google OR-Tools C APIs — CP-SAT
//! (`ortools/sat/c_api/cp_solver_c.h`) and MathOpt
//! (`ortools/math_opt/core/c_api/solver.h`) — plus the linkage against the
//! native OR-Tools library and, under the `shim` feature, Oxidor's own C
//! bridge for APIs without an upstream C API (routing, algorithms).
//!
//! Higher-level crates never link OR-Tools directly — they go through this
//! crate. All exchange happens as serialized protobuf bytes or flat POD
//! arrays; no C++ type crosses the boundary.
//!
//! # Library acquisition
//!
//! The build script obtains the native library one of two ways:
//!
//! - **`ORTOOLS_PREFIX`** (always wins when set) — links the shared library
//!   of an existing installation, as laid out by the official release
//!   archives. All solvers available.
//! - **`download-prebuilt` feature** — when `ORTOOLS_PREFIX` is unset,
//!   downloads a static OR-Tools bundle built by this project's CI from its
//!   GitHub releases (SHA-256 pinned in the crate, cached under
//!   `~/.cache/oxidor`) and links it statically. Covers CP-SAT, routing, and
//!   the algorithms; MathOpt's solver registry relies on global initializers
//!   that selective static linking drops, so MathOpt needs `ORTOOLS_PREFIX`.
//!
//! A `vendored` (CMake source build) mode remains on the roadmap.

#![no_std]
#![warn(missing_docs)]

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

// Oxidor's own C shim (`cpp/oxidor_shim.cc`, compiled under the `shim`
// feature) for OR-Tools APIs without an upstream C API.
#[cfg(feature = "shim")]
unsafe extern "C" {
    /// Solves a vehicle routing problem over a dense arc-cost matrix.
    ///
    /// `matrix` is row-major with `num_nodes²` entries. `demands` (length
    /// `num_nodes`) and `vehicle_capacities` (length `num_vehicles`) are
    /// either both non-null (adds a capacity dimension) or both null.
    /// `params_bytes` is a serialized `RoutingSearchParameters` merged over
    /// the defaults, or null.
    ///
    /// On success returns a `malloc`-allocated i64 buffer of `*out_len`
    /// entries laid out as `[status, objective, num_routes, route_len,
    /// nodes…, route_len, …]`; routes exclude the depot endpoints. On failure
    /// returns null and sets `*error_message` (`malloc`-allocated).
    ///
    /// # Safety
    ///
    /// Input arrays must be valid for the stated lengths; output locations
    /// must be valid for writes. The caller owns the returned buffer and
    /// `*error_message`, releasing both with the C allocator's `free`.
    pub fn OxidorRoutingSolveMatrix(
        num_nodes: i32,
        num_vehicles: i32,
        depot: i32,
        matrix: *const i64,
        demands: *const i64,
        vehicle_capacities: *const i64,
        params_bytes: *const c_void,
        params_len: i32,
        out_len: *mut i32,
        error_message: *mut *mut c_char,
    ) -> *mut i64;
}

#[cfg(feature = "shim")]
unsafe extern "C" {
    /// Solves a (multi-dimensional) 0-1 knapsack with branch and bound.
    ///
    /// `weights` is row-major `num_dims × num_items`; `capacities` has
    /// `num_dims` entries. Writes the best value and 0/1 selection flags
    /// (length `num_items`). Returns 0 on success; nonzero on failure with
    /// `*error_message` set (`malloc`-allocated).
    ///
    /// # Safety
    ///
    /// Arrays must be valid for the stated lengths; `out_selected` must be
    /// writable for `num_items` bytes. The caller frees `*error_message`.
    pub fn OxidorKnapsackSolve(
        profits: *const i64,
        num_items: i32,
        weights: *const i64,
        capacities: *const i64,
        num_dims: i32,
        out_best_value: *mut i64,
        out_selected: *mut u8,
        error_message: *mut *mut c_char,
    ) -> i32;

    /// Computes a maximum flow; returns the `SimpleMaxFlow` status
    /// (0 = OPTIMAL) or -1 on a caught C++ exception.
    ///
    /// # Safety
    ///
    /// Arc arrays and `out_flows` must be valid for `num_arcs` entries; the
    /// caller frees `*error_message`.
    pub fn OxidorMaxFlowSolve(
        tails: *const i32,
        heads: *const i32,
        capacities: *const i64,
        num_arcs: i32,
        source: i32,
        sink: i32,
        out_flows: *mut i64,
        out_max_flow: *mut i64,
        error_message: *mut *mut c_char,
    ) -> i32;

    /// Computes a minimum-cost flow; returns the `SimpleMinCostFlow` status
    /// (1 = OPTIMAL) or -1 on a caught C++ exception.
    ///
    /// # Safety
    ///
    /// Arc arrays and `out_flows` must be valid for `num_arcs` entries and
    /// `supplies` for `num_nodes`; the caller frees `*error_message`.
    pub fn OxidorMinCostFlowSolve(
        tails: *const i32,
        heads: *const i32,
        capacities: *const i64,
        unit_costs: *const i64,
        num_arcs: i32,
        supplies: *const i64,
        num_nodes: i32,
        out_flows: *mut i64,
        out_total_cost: *mut i64,
        error_message: *mut *mut c_char,
    ) -> i32;
}
