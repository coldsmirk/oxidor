use std::env;
use std::path::{Path, PathBuf};

/// How the OR-Tools native library is obtained and linked.
struct Installation {
    /// Directory holding `lib/` and `include/`.
    root: PathBuf,
    /// True for the prebuilt single-archive bundle, false for a shared
    /// library installation.
    static_bundle: bool,
}

fn main() {
    println!("cargo::rerun-if-env-changed=ORTOOLS_PREFIX");

    // docs.rs builds documentation in an offline sandbox with no native
    // library present; declarations compile fine without linking.
    if env::var_os("DOCS_RS").is_some() {
        return;
    }

    let installation = locate_installation();
    let lib_dir = library_directory(&installation.root);

    println!("cargo::rustc-link-search=native={}", lib_dir.display());
    if installation.static_bundle {
        // One merged archive (libortools.a) carrying OR-Tools and every
        // vendored dependency; only the C++ runtime and, on macOS, system
        // frameworks remain external.
        //
        // Known limitation: MathOpt registers its solvers through global
        // initializers, which a selective archive pull drops — MathOpt
        // solves return a clean "solver type … is not registered" error
        // under this mode. Use ORTOOLS_PREFIX with an official archive for
        // MathOpt.
        //
        // `-bundle`: keep the (huge) archive out of the rlib and hand it to
        // the final linker directly — the linker's demand-driven member
        // resolution then works across the whole archive (rlib-bundled
        // members failed to resolve intra-archive references like bzlib).
        println!("cargo::rustc-link-lib=static:-bundle=ortools");
        match env::var("CARGO_CFG_TARGET_OS").as_deref() {
            Ok("macos") => {
                println!("cargo::rustc-link-lib=dylib=c++");
                // absl's time-zone lookup uses CoreFoundation.
                println!("cargo::rustc-link-lib=framework=CoreFoundation");
            }
            Ok("linux") => println!("cargo::rustc-link-lib=dylib=stdc++"),
            _ => {}
        }
    } else {
        println!("cargo::rustc-link-lib=dylib=ortools");
        // Run-time rpath for this package's own test binaries; link args do
        // not propagate to dependents.
        if matches!(
            env::var("CARGO_CFG_TARGET_OS").as_deref(),
            Ok("macos") | Ok("linux")
        ) {
            println!("cargo::rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
        }
    }

    // The `shim` feature compiles our own C bridge (cpp/oxidor_shim.cc) for
    // OR-Tools APIs without an upstream C API; it needs the headers the
    // installation ships next to the library.
    if env::var_os("CARGO_FEATURE_SHIM").is_some() {
        compile_shim(&installation, &lib_dir);
    }

    // Published to direct dependents as DEP_ORTOOLS_LIBDIR (via `links =
    // "ortools"`), so they can emit run-time rpath flags for their binaries
    // (harmless for the static bundle, which needs no rpath).
    println!("cargo::metadata=libdir={}", lib_dir.display());
}

fn locate_installation() -> Installation {
    // An explicitly configured installation always wins; empty counts as
    // unset so the prebuilt path can be forced in an environment where the
    // variable would otherwise be injected (e.g. .cargo/config.toml).
    if let Some(prefix) = env::var_os("ORTOOLS_PREFIX").filter(|value| !value.is_empty()) {
        let root = PathBuf::from(prefix);
        let lib_file = library_directory(&root).join(shared_library_name());
        if !lib_file.exists() {
            die_with_help(&format!("{} does not exist", lib_file.display()));
        }
        return Installation {
            root,
            static_bundle: false,
        };
    }

    #[cfg(feature = "download-prebuilt")]
    {
        return Installation {
            root: prebuilt::obtain(),
            static_bundle: true,
        };
    }

    #[cfg(not(feature = "download-prebuilt"))]
    die_with_help("the ORTOOLS_PREFIX environment variable is not set");
}

fn compile_shim(installation: &Installation, lib_dir: &Path) {
    let include_dir = installation.root.join("include");
    if !include_dir.exists() {
        die_with_help(&format!(
            "the `shim` feature needs OR-Tools headers, but {} does not exist",
            include_dir.display()
        ));
    }
    if !installation.static_bundle {
        // The shim inlines absl/protobuf template code whose out-of-line
        // symbols live in the dependency libraries shipped next to
        // libortools; the linker does not resolve those transitively. (The
        // static bundle already contains them.)
        for dependency in shim_native_dependencies(lib_dir) {
            println!("cargo::rustc-link-lib=dylib={dependency}");
        }
    }
    println!("cargo::rerun-if-changed=cpp/oxidor_shim.cc");
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
    // archiver unless the user chose one explicitly. The archiver runs on
    // the build host, so gate on HOST rather than the target.
    if env::var_os("AR").is_none()
        && env::var("HOST").is_ok_and(|host| host.contains("apple-darwin"))
        && Path::new("/usr/bin/ar").exists()
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

/// The unversioned absl / protobuf shared libraries in `lib_dir` (e.g.
/// `libabsl_base.dylib` but not `libabsl_base.2508.0.0.dylib`).
fn shim_native_dependencies(lib_dir: &Path) -> Vec<String> {
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

/// `lib64/` on RHEL-family layouts (e.g. the official AlmaLinux archives),
/// `lib/` everywhere else.
fn library_directory(root: &Path) -> PathBuf {
    let lib64 = root.join("lib64");
    if lib64.exists() {
        lib64
    } else {
        root.join("lib")
    }
}

fn shared_library_name() -> &'static str {
    match env::var("CARGO_CFG_TARGET_OS").as_deref() {
        Ok("macos") => "libortools.dylib",
        Ok("windows") => "ortools.lib",
        _ => "libortools.so",
    }
}

fn die_with_help(cause: &str) -> ! {
    // Only suggest download-prebuilt when it is not already enabled.
    let prebuilt_hint = if cfg!(feature = "download-prebuilt") {
        ""
    } else {
        "Or enable oxidor-sys's `download-prebuilt` feature to fetch a static\n\
         bundle built by this project's CI (no local setup).\n\
         \n"
    };
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
         {prebuilt_hint}\
         If you only build models (no solving), disable the `solve` feature\n\
         instead: oxidor = {{ version = \"...\", default-features = false }}\n"
    );
}

/// Downloading, verifying, caching, and unpacking the static bundle.
#[cfg(feature = "download-prebuilt")]
mod prebuilt {
    use std::path::PathBuf;

    /// The release tag on this repository holding the bundles.
    const RELEASE_TAG: &str = "ortools-v9.15";
    /// The OR-Tools version baked into the bundle file names.
    const ORTOOLS_VERSION: &str = "v9.15";
    /// Per-target SHA-256 of the bundle tarballs, one `<target> <hex>` pair
    /// per line (kept in a data file so CI updates don't touch code).
    const CHECKSUMS: &str = include_str!("prebuilt-checksums.txt");

    pub(super) fn obtain() -> PathBuf {
        println!("cargo::rerun-if-env-changed=OXIDOR_CACHE_DIR");
        let target = std::env::var("TARGET").expect("cargo sets TARGET");
        let Some(expected_hash) = checksum_for(&target) else {
            super::die_with_help(&format!(
                "no prebuilt static bundle is published for target {target}"
            ));
        };

        let cache_root = cache_directory();
        // The expected hash is part of the cache key, so republished bundles
        // invalidate stale caches automatically.
        let bundle_root = cache_root.join(format!(
            "{RELEASE_TAG}-{target}-{}",
            &expected_hash[..16.min(expected_hash.len())]
        ));
        let is_complete = |root: &std::path::Path| root.join("lib").join("libortools.a").exists();
        if is_complete(&bundle_root) {
            return bundle_root;
        }

        let url = format!(
            "https://github.com/coldsmirk/oxidor/releases/download/{RELEASE_TAG}/oxidor-ortools-{ORTOOLS_VERSION}-{target}.tar.gz"
        );
        println!("cargo::warning=oxidor-sys: downloading prebuilt OR-Tools bundle ({url})");
        let compressed = download(&url);
        verify_sha256(&compressed, expected_hash, &target);

        // Unpack into a process-unique staging directory, then rename into
        // place: a half-unpacked cache is never mistaken for a valid one, and
        // concurrent builds cannot clobber each other's staging area.
        let staging = bundle_root.with_extension(format!("partial-{}", std::process::id()));
        std::fs::create_dir_all(&staging).expect("create cache directory");
        let decoder = flate2::read::GzDecoder::new(compressed.as_slice());
        tar::Archive::new(decoder)
            .unpack(&staging)
            .expect("unpack the prebuilt bundle");
        match std::fs::rename(&staging, &bundle_root) {
            Ok(()) => {}
            // A concurrent build won the rename with content of the same
            // verified hash; discard our copy and use theirs.
            Err(_) if is_complete(&bundle_root) => {
                let _ = std::fs::remove_dir_all(&staging);
            }
            Err(error) => panic!(
                "could not activate the unpacked bundle at {}: {error}",
                bundle_root.display()
            ),
        }
        bundle_root
    }

    fn checksum_for(target: &str) -> Option<&'static str> {
        CHECKSUMS.lines().find_map(|line| {
            let mut parts = line.split_whitespace();
            (parts.next() == Some(target))
                .then(|| parts.next())
                .flatten()
        })
    }

    fn cache_directory() -> PathBuf {
        if let Some(dir) = std::env::var_os("OXIDOR_CACHE_DIR") {
            return PathBuf::from(dir);
        }
        let home = std::env::var_os("HOME")
            .or_else(|| std::env::var_os("USERPROFILE"))
            .expect("HOME is set");
        PathBuf::from(home).join(".cache").join("oxidor")
    }

    fn download(url: &str) -> Vec<u8> {
        let response = ureq::get(url)
            .timeout(std::time::Duration::from_secs(600))
            .call()
            .unwrap_or_else(|error| panic!("downloading {url} failed: {error}"));
        let mut bytes = Vec::new();
        std::io::Read::read_to_end(&mut response.into_reader(), &mut bytes)
            .expect("read the bundle body");
        bytes
    }

    fn verify_sha256(bytes: &[u8], expected: &str, target: &str) {
        use sha2::Digest;
        let mut hasher = sha2::Sha256::new();
        hasher.update(bytes);
        let actual: String = hasher
            .finalize()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        assert_eq!(
            actual, expected,
            "SHA-256 mismatch for the {target} prebuilt bundle — refusing to use it",
        );
    }
}
