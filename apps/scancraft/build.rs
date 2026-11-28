//! Embeds the git commit the app is built from, so every capture record names the code that made
//! it (`src/capture.rs`). `SCANCRAFT_GIT_COMMIT` overrides it (builds outside a checkout).

use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn main() {
    println!("cargo:rerun-if-env-changed=SCANCRAFT_GIT_COMMIT");
    let (commit, dirty) = match std::env::var("SCANCRAFT_GIT_COMMIT") {
        Ok(c) if !c.trim().is_empty() => (c.trim().to_string(), false),
        _ => {
            let commit = git(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".into());
            // Tracked files changed (untracked ones do not change the build).
            let dirty = git(&["status", "--porcelain", "--untracked-files=no"]).is_some_and(|s| !s.is_empty());
            (commit, dirty)
        }
    };
    // Rebuild when the commit changes (a worktree's HEAD lives in its own git dir).
    if let Some(head) = git(&["rev-parse", "--git-path", "HEAD"]) {
        println!("cargo:rerun-if-changed={head}");
    }
    if let Some(refs) = git(&["rev-parse", "--symbolic-full-name", "HEAD"]).and_then(|r| git(&["rev-parse", "--git-path", &r])) {
        println!("cargo:rerun-if-changed={refs}");
    }
    if let Some(index) = git(&["rev-parse", "--git-path", "index"]) {
        println!("cargo:rerun-if-changed={index}");
    }
    println!("cargo:rustc-env=SCANCRAFT_GIT_COMMIT={commit}");
    println!("cargo:rustc-env=SCANCRAFT_GIT_DIRTY={}", if dirty { "1" } else { "0" });
    println!("cargo:rustc-env=SCANCRAFT_BUILD_TARGET={}", std::env::var("TARGET").unwrap_or_default());
    println!("cargo:rustc-env=SCANCRAFT_BUILD_PROFILE={}", std::env::var("PROFILE").unwrap_or_default());
}
