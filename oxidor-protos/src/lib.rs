//! Generated protobuf types for Google OR-Tools models.
//!
//! Pure Rust, no native dependencies: model building and serialization work
//! without the OR-Tools library. The code in [`sat`] is generated offline
//! with prost from the `.proto` files of a pinned OR-Tools release (vendored
//! under `protos/`) and committed to the repository, so users need neither
//! `protoc` nor a network connection at build time.
//!
//! Regenerate with `cargo run -p xtask -- gen-protos` after bumping the
//! vendored protos.

/// Types from `ortools/sat/cp_model.proto` and `ortools/sat/sat_parameters.proto`
/// (protobuf package `operations_research.sat`), OR-Tools v9.15.
pub mod sat {
    // Generated code is exempt from style lints; upstream doc comments are
    // rendered as-is.
    #![allow(clippy::all)]
    include!("generated/operations_research.sat.rs");
}

pub use prost;
