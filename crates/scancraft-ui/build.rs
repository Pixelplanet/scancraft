//! Embeds the SVG icon masters in `assets/icons/{model,flat}` as `(name, svg)` pairs.
use std::{env, fs, path::Path};

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/icons");
    println!("cargo:rerun-if-changed={}", root.display());
    let mut entries = Vec::new();
    for dir in ["model", "flat"] {
        let mut files: Vec<_> = fs::read_dir(root.join(dir)).expect("icon dir").filter_map(|e| e.ok()).map(|e| e.path()).collect();
        files.sort();
        for path in files {
            if path.extension().and_then(|e| e.to_str()) != Some("svg") {
                continue;
            }
            let name = path.file_stem().unwrap().to_string_lossy().to_string();
            entries.push(format!("    ({:?}, {:?}, include_str!({:?})),\n", name, dir == "flat", path.display().to_string().replace(std::path::MAIN_SEPARATOR, "/")));
        }
    }
    let out = Path::new(&env::var("OUT_DIR").unwrap()).join("icon_data.rs");
    fs::write(out, format!("/// (name, flat line art, svg source)\npub const ICONS: &[(&str, bool, &str)] = &[\n{}];\n", entries.concat())).unwrap();
}
