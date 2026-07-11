use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-env-changed=ORTOOLS_PREFIX");

    // docs.rs builds documentation in an offline sandbox with no native
    // library present; declarations compile fine without linking.
    if env::var_os("DOCS_RS").is_some() {
        return;
    }

    let prefix = match env::var_os("ORTOOLS_PREFIX") {
        Some(path) => PathBuf::from(path),
        None => die_with_help("the ORTOOLS_PREFIX environment variable is not set"),
    };
    let lib_dir = prefix.join("lib");
    let lib_file = lib_dir.join(shared_library_name());
    if !lib_file.exists() {
        die_with_help(&format!("{} does not exist", lib_file.display()));
    }

    println!("cargo::rustc-link-search=native={}", lib_dir.display());
    println!("cargo::rustc-link-lib=dylib=ortools");
    // Run-time rpath for this package's own test binaries; link args do not
    // propagate to dependents.
    if matches!(
        env::var("CARGO_CFG_TARGET_OS").as_deref(),
        Ok("macos") | Ok("linux")
    ) {
        println!("cargo::rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
    }
    // Published to direct dependents as DEP_ORTOOLS_ROOT (via `links =
    // "ortools"`), so they can emit run-time rpath flags for their binaries.
    println!("cargo::metadata=root={}", prefix.display());
}

fn shared_library_name() -> &'static str {
    match env::var("CARGO_CFG_TARGET_OS").as_deref() {
        Ok("macos") => "libortools.dylib",
        Ok("windows") => "ortools.lib",
        _ => "libortools.so",
    }
}

fn die_with_help(cause: &str) -> ! {
    panic!(
        "\n\
         oxidor-sys could not locate the OR-Tools native library: {cause}.\n\
         \n\
         Point ORTOOLS_PREFIX at an OR-Tools installation whose `lib/` holds the\n\
         shared library — e.g. an extracted official C++ release archive from\n\
         https://github.com/google/or-tools/releases — then rebuild:\n\
         \n\
             export ORTOOLS_PREFIX=/path/to/or-tools\n\
         \n\
         If you only build models (no solving), disable the `solve` feature\n\
         instead: oxidor = {{ version = \"...\", default-features = false }}\n"
    );
}
