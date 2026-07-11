use core::ffi::{CStr, c_char};

/// A failure inside the native layer (invalid input detected by OR-Tools, or
/// a caught C++ exception).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AlgorithmError {
    /// Human-readable description.
    pub message: String,
}

impl std::fmt::Display for AlgorithmError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "OR-Tools algorithm failed: {}", self.message)
    }
}

impl std::error::Error for AlgorithmError {}

impl AlgorithmError {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

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
