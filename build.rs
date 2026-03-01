// build.rs
//
// Indique à Cargo de ré-exécuter ce script si memory.x change,
// et ajoute le répertoire courant au chemin de recherche du linker
// pour que `-Tlink.x` (fourni par cortex-m-rt) trouve memory.x.

use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copie memory.x dans OUT_DIR afin que le linker le trouve
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // Déclenche une ré-exécution si memory.x est modifié
    println!("cargo:rerun-if-changed=memory.x");
}
