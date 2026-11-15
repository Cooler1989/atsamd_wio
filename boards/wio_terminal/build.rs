use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    if env::var_os("CARGO_FEATURE_RT").is_some() {
        // `bootloader` feature: link behind the UF2 bootloader (production).
        // Otherwise link at 0x0 for SWD development.
        let (name, bytes): (&str, &[u8]) = if env::var_os("CARGO_FEATURE_BOOTLOADER").is_some() {
            ("memory-bootloader.x", include_bytes!("memory-bootloader.x"))
        } else {
            ("memory-nobootloader.x", include_bytes!("memory-nobootloader.x"))
        };
        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(bytes)
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
        println!("cargo:rerun-if-changed={}", name);
    }
    println!("cargo:rerun-if-changed=memory-bootloader.x");
    println!("cargo:rerun-if-changed=memory-nobootloader.x");
    println!("cargo:rerun-if-changed=build.rs");
}
