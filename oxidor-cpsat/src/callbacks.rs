//! Streaming solution callbacks: observe every feasible solution as the
//! search finds it, and optionally stop early — the CP-SAT feature the
//! official Go bindings lack.
//!
//! The observer crosses the FFI boundary through Oxidor's C shim: a boxed
//! Rust closure travels as `user_data` into a C trampoline, guarded by a
//! panic barrier (a Rust panic must never unwind into C++).

use core::ffi::{c_char, c_void};
use std::any::Any;
use std::ops::ControlFlow;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};

use oxidor_protos::prost::Message;
use oxidor_protos::sat::{CpSolverResponse, SatParameters};

use crate::model::CpModelBuilder;
use crate::solver::{SolveResponse, c_length, take_c_buffer};

impl CpModelBuilder {
    /// Solves the model, invoking `callback` on every feasible solution the
    /// search finds: each improving solution during optimization, or each
    /// solution when the parameters set `enumerate_all_solutions`.
    ///
    /// Returning [`ControlFlow::Break`] from the callback stops the search;
    /// the solve then returns with the best solution seen so far, exactly
    /// like a stopped [`solve_interruptible`](Self::solve_interruptible). A
    /// panic inside the callback stops the search and resumes on this thread
    /// once the solver has wound down.
    ///
    /// The callback must be [`Send`]: CP-SAT may report solutions from a
    /// worker thread (calls are serialized by the solver).
    ///
    /// ```no_run
    /// use std::ops::ControlFlow;
    /// use oxidor_cpsat::{CpModelBuilder, SatParameters};
    ///
    /// # let model = CpModelBuilder::new();
    /// let parameters = SatParameters {
    ///     enumerate_all_solutions: Some(true),
    ///     ..Default::default()
    /// };
    /// let mut seen = 0;
    /// model.solve_with_solution_callback(&parameters, |_solution| {
    ///     seen += 1;
    ///     if seen == 10 { ControlFlow::Break(()) } else { ControlFlow::Continue(()) }
    /// });
    /// ```
    pub fn solve_with_solution_callback<F>(
        &self,
        parameters: &SatParameters,
        mut callback: F,
    ) -> SolveResponse
    where
        F: FnMut(SolveResponse) -> ControlFlow<()> + Send,
    {
        let model_bytes = self.proto().encode_to_vec();
        let parameter_bytes = parameters.encode_to_vec();
        let mut state = CallbackState {
            callback: &mut callback,
            model: self.id(),
            panic: None,
        };

        let mut response_pointer: *mut c_void = core::ptr::null_mut();
        let mut response_length: i32 = 0;
        let mut error_pointer: *mut c_char = core::ptr::null_mut();
        // SAFETY: the input pointers are valid for their stated lengths and
        // hold protos we just encoded; the output locations are written
        // before the call returns; `state` outlives the (blocking) call and
        // is only touched through the trampoline, which never unwinds.
        let code = unsafe {
            oxidor_sys::OxidorCpSatSolveWithObserver(
                model_bytes.as_ptr().cast(),
                c_length(&model_bytes),
                parameter_bytes.as_ptr().cast(),
                c_length(&parameter_bytes),
                trampoline,
                (&raw mut state).cast(),
                &mut response_pointer,
                &mut response_length,
                &mut error_pointer,
            )
        };

        // Free before any panic below so the response buffer cannot leak.
        let response_bytes = take_c_buffer(response_pointer, response_length);
        if let Some(payload) = state.panic.take() {
            resume_unwind(payload);
        }
        if code != 0 {
            panic!(
                "the CP-SAT observer solve failed natively: {}",
                take_c_error(error_pointer),
            );
        }
        let proto = CpSolverResponse::decode(response_bytes.as_slice())
            .expect("OR-Tools returned an undecodable CpSolverResponse; version mismatch between oxidor-protos and the linked library");
        SolveResponse::from_parts(self.id(), proto)
    }
}

/// What travels through `user_data`: the observer closure, the identity its
/// solution handles must carry, and any panic caught behind the barrier.
struct CallbackState<'solve> {
    callback: &'solve mut (dyn FnMut(SolveResponse) -> ControlFlow<()> + Send),
    model: u32,
    panic: Option<Box<dyn Any + Send>>,
}

/// The C-side observer: decodes the solution, runs the Rust callback behind
/// a panic barrier, and reports "stop" to the search on `Break` or panic.
unsafe extern "C" fn trampoline(
    response_bytes: *const c_void,
    response_len: i32,
    user_data: *mut c_void,
) -> i32 {
    // SAFETY: `user_data` is the CallbackState owned by the blocking solve
    // call; the solver serializes observer invocations, so this exclusive
    // reference is unique.
    let state = unsafe { &mut *user_data.cast::<CallbackState<'_>>() };
    if state.panic.is_some() {
        // Already unwinding-to-be: keep asking the search to stop.
        return 1;
    }
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        let bytes = if response_bytes.is_null() || response_len <= 0 {
            &[]
        } else {
            // SAFETY: the shim hands a readable buffer of exactly
            // `response_len` bytes, valid for the duration of this call.
            unsafe {
                core::slice::from_raw_parts(response_bytes.cast::<u8>(), response_len as usize)
            }
        };
        let proto = CpSolverResponse::decode(bytes)
            .expect("OR-Tools streamed an undecodable CpSolverResponse; version mismatch between oxidor-protos and the linked library");
        (state.callback)(SolveResponse::from_parts(state.model, proto))
    }));
    match outcome {
        Ok(ControlFlow::Continue(())) => 0,
        Ok(ControlFlow::Break(())) => 1,
        Err(payload) => {
            state.panic = Some(payload);
            1
        }
    }
}

/// Copies a malloc-allocated C error string into owned memory and frees it.
fn take_c_error(pointer: *mut c_char) -> String {
    if pointer.is_null() {
        return "unknown error".into();
    }
    // SAFETY: a non-null error from the shim is a malloc-allocated,
    // null-terminated string we own; copy it out and release it with the C
    // allocator, as the shim contract requires.
    unsafe {
        let message = core::ffi::CStr::from_ptr(pointer)
            .to_string_lossy()
            .into_owned();
        libc::free(pointer.cast());
        message
    }
}
