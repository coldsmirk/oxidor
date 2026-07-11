//! Idiomatic Rust API for the Google OR-Tools CP-SAT solver.
//!
//! Model building is pure Rust: a builder assembles a `CpModelProto`
//! (from `oxidor-protos`) with typed variable handles and operator-overloaded
//! linear expressions. Solving crosses the FFI boundary once, through the
//! official CP-SAT C API exposed by `oxidor-sys`, exchanging serialized
//! protobuf bytes.
