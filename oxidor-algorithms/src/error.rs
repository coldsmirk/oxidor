use core::ffi::{CStr, c_char};

/// A failure to run an algorithm.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum AlgorithmError {
    /// The input violates the algorithm's contract (mismatched lengths,
    /// negative capacities, out-of-range node indices, …); detected before
    /// anything crosses into OR-Tools.
    InvalidInput(String),
    /// The native layer reported an error (invalid input detected by
    /// OR-Tools, or a caught C++ exception).
    Native(String),
}

impl std::fmt::Display for AlgorithmError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInput(message) => write!(formatter, "invalid input: {message}"),
            Self::Native(message) => {
                write!(formatter, "OR-Tools algorithm failed: {message}")
            }
        }
    }
}

impl std::error::Error for AlgorithmError {}

/// Copies and frees a malloc'd C error message from the shim.
///
/// # Safety
///
/// `message` is null or a malloc-allocated, null-terminated string the
/// caller owns.
pub(crate) unsafe fn take_error_message(message: *mut c_char) -> String {
    if message.is_null() {
        return "no error message".to_string();
    }
    // SAFETY: non-null implies a valid null-terminated string per the shim
    // contract; we free it with the C allocator after copying.
    unsafe {
        let text = CStr::from_ptr(message).to_string_lossy().into_owned();
        libc::free(message.cast());
        text
    }
}
