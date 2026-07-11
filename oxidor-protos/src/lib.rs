//! Generated protobuf types for Google OR-Tools models.
//!
//! Pure Rust, no native dependencies: model building and serialization work
//! without the OR-Tools library. Everything under [`operations_research`] is
//! generated offline with prost from the `.proto` files of a pinned OR-Tools
//! release (vendored under `protos/`) and committed to the repository, so
//! users need neither `protoc` nor a network connection at build time.
//!
//! Regenerate with `cargo run -p xtask -- gen-protos` after changing the
//! vendored protos.

/// Generated types, nested by protobuf package (OR-Tools v9.15).
pub mod operations_research {
    #![allow(clippy::all)]
    // Package `operations_research` itself (GScip result types).
    include!("generated/operations_research.rs");

    /// `ortools/glop/parameters.proto` (package `operations_research.glop`).
    pub mod glop {
        #![allow(clippy::all)]
        include!("generated/operations_research.glop.rs");
    }

    /// The MathOpt model/solution/result protos
    /// (package `operations_research.math_opt`).
    pub mod math_opt {
        #![allow(clippy::all)]
        include!("generated/operations_research.math_opt.rs");
    }

    /// PDLP solve logs (package `operations_research.pdlp`).
    pub mod pdlp {
        #![allow(clippy::all)]
        include!("generated/operations_research.pdlp.rs");
    }

    /// `ortools/sat/cp_model.proto` and `ortools/sat/sat_parameters.proto`
    /// (package `operations_research.sat`).
    pub mod sat {
        #![allow(clippy::all)]
        include!("generated/operations_research.sat.rs");
    }
}

pub use operations_research::{math_opt, sat};

pub use prost;
pub use prost_types;
