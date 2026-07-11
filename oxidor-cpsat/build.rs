use std::env;

fn main() {
    // Set by oxidor-sys (links = "ortools") when it linked a shared library
    // from an installation prefix. Emit run-time rpath flags so this crate's
    // tests, examples, and binaries locate the library without env vars.
    // Doctests cannot receive link args; keep doc examples `no_run`.
    let Ok(root) = env::var("DEP_ORTOOLS_ROOT") else {
        return;
    };
    if matches!(
        env::var("CARGO_CFG_TARGET_OS").as_deref(),
        Ok("macos") | Ok("linux")
    ) {
        // Applies to every linked target of this package (tests, examples);
        // the per-kind variants are rejected for kinds the package lacks.
        println!("cargo::rustc-link-arg=-Wl,-rpath,{root}/lib");
    }
}
