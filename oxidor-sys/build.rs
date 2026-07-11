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
    // The `shim` feature compiles our own C bridge (cpp/oxidor_shim.cc) for
    // OR-Tools APIs without an upstream C API; it needs the headers the
    // installation ships next to the library.
    if env::var_os("CARGO_FEATURE_SHIM").is_some() {
        let include_dir = prefix.join("include");
        if !include_dir.exists() {
            die_with_help(&format!(
                "the `shim` feature needs OR-Tools headers, but {} does not exist",
                include_dir.display()
            ));
        }
        println!("cargo::rerun-if-changed=cpp/oxidor_shim.cc");
        // The shim inlines absl/protobuf template code whose out-of-line
        // symbols live in the dependency libraries shipped next to
        // libortools; the linker does not resolve those transitively.
        for dependency in shim_native_dependencies(&lib_dir) {
            println!("cargo::rustc-link-lib=dylib={dependency}");
        }
        let mut shim = cc::Build::new();
        // The compile definitions the official ortools CMake config exports
        // (lib/cmake/ortools/ortoolsTargets.cmake); headers depend on them.
        for (key, value) in [
            ("OR_PROTO_DLL", Some("")),
            ("USE_MATH_OPT", None),
            ("USE_BOP", None),
            ("USE_CBC", None),
            ("USE_CLP", None),
            ("USE_GLOP", None),
            ("USE_HIGHS", None),
            ("USE_PDLP", None),
            ("USE_SCIP", None),
        ] {
            shim.define(key, value);
        }
        // A GNU `ar` on PATH (e.g. Homebrew binutils) produces archives
        // Apple's linker rejects ("not 8-byte aligned"); prefer the system
        // archiver unless the user chose one explicitly.
        if env::var_os("AR").is_none()
            && env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos")
            && std::path::Path::new("/usr/bin/ar").exists()
        {
            shim.archiver("/usr/bin/ar");
        }
        shim.cpp(true)
            .std("c++20")
            .include(&include_dir)
            .file("cpp/oxidor_shim.cc")
            // The bulk of any diagnostics would come from OR-Tools' own
            // headers; keep the build log readable.
            .flag_if_supported("-w")
            .compile("oxidor_shim");
    }
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

/// The unversioned absl / protobuf shared libraries in `lib_dir` (e.g.
/// `libabsl_base.dylib` but not `libabsl_base.2508.0.0.dylib`).
fn shim_native_dependencies(lib_dir: &std::path::Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(lib_dir)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let file_name = entry.file_name().into_string().ok()?;
            let stem = file_name
                .strip_prefix("lib")?
                .strip_suffix(".dylib")
                .or_else(|| file_name.strip_prefix("lib")?.strip_suffix(".so"))?;
            let relevant = stem.starts_with("absl_")
                || stem == "protobuf"
                || stem.starts_with("utf8_")
                || stem == "re2";
            // A dot in the stem means a versioned file; the unversioned
            // symlink is enough.
            (relevant && !stem.contains('.')).then(|| stem.to_string())
        })
        .collect();
    names.sort();
    names.dedup();
    names
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
