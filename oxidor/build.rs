use std::env;

fn main() {
    // Set by oxidor-sys (links = "ortools") when it linked a shared library
    // from an installation prefix. Emit a run-time rpath so this crate's
    // linked binaries — including the merged doctest binary, which loads the
    // library even for `no_run` examples — can locate it.
    let Ok(root) = env::var("DEP_ORTOOLS_ROOT") else {
        return;
    };
    if matches!(
        env::var("CARGO_CFG_TARGET_OS").as_deref(),
        Ok("macos") | Ok("linux")
    ) {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{root}/lib");
    }
}
