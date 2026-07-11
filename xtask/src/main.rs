//! Maintainer tasks. Usage: `cargo run -p xtask -- <task>`
//!
//! Tasks:
//! - `gen-protos` — regenerate `oxidor-protos/src/generated/` from the vendored
//!   `.proto` files (pure Rust via protox; no `protoc` required).

use std::path::{Path, PathBuf};

fn main() {
    let task = std::env::args().nth(1);
    match task.as_deref() {
        Some("gen-protos") => gen_protos(),
        _ => {
            eprintln!("usage: cargo run -p xtask -- gen-protos");
            std::process::exit(2);
        }
    }
}

/// Compiles the vendored OR-Tools protos and writes the generated Rust code
/// into `oxidor-protos/src/generated/`, which is committed to the repository.
fn gen_protos() {
    let root = workspace_root();
    let proto_dir = root.join("oxidor-protos/protos");
    let out_dir = root.join("oxidor-protos/src/generated");
    std::fs::create_dir_all(&out_dir).expect("create out dir");

    let files = [
        proto_dir.join("ortools/sat/cp_model.proto"),
        proto_dir.join("ortools/sat/sat_parameters.proto"),
    ];

    let descriptors = protox::compile(&files, [&proto_dir]).expect("protox compile");
    prost_build::Config::new()
        .out_dir(&out_dir)
        .compile_fds(descriptors)
        .expect("prost codegen");

    println!("generated into {}", out_dir.display());
}

fn workspace_root() -> PathBuf {
    // xtask lives at <root>/xtask, so the workspace root is its parent.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .to_path_buf()
}
