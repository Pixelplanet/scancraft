//! Embeds the git commit the app is built from, so every capture record names the code that made
//! it (`src/capture.rs`). `SCANCRAFT_GIT_COMMIT` overrides the commit (builds outside a checkout;
//! `SCANCRAFT_GIT_DIRTY=1` then says the sources were changed, when git cannot tell).

use std::process::Command;

include!("src/build_info.rs");

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn main() {
    println!("cargo:rerun-if-env-changed=SCANCRAFT_GIT_COMMIT");
    println!("cargo:rerun-if-env-changed=SCANCRAFT_GIT_DIRTY");
    let head = git(&["rev-parse", "HEAD"]);
    // Tracked files changed (untracked ones do not change the build).
    let status = git(&["status", "--porcelain", "--untracked-files=no"]);
    let (commit, dirty) = commit_and_dirty(std::env::var("SCANCRAFT_GIT_COMMIT").ok().as_deref(), std::env::var("SCANCRAFT_GIT_DIRTY").ok().as_deref(), head.as_deref(), status.as_deref());
    // Look again when the commit changes (a worktree's HEAD lives in its own git dir), when the
    // index changes, and when any source changes (an unstaged edit touches no git file).
    let mut git_files = Vec::new();
    git_files.extend(git(&["rev-parse", "--git-path", "HEAD"]));
    git_files.extend(git(&["rev-parse", "--symbolic-full-name", "HEAD"]).and_then(|r| git(&["rev-parse", "--git-path", &r])));
    git_files.extend(git(&["rev-parse", "--git-path", "index"]));
    let top = git(&["rev-parse", "--show-toplevel"]).map(std::path::PathBuf::from).unwrap_or_else(|| std::path::PathBuf::from("../.."));
    for path in rerun_paths(&top, &git_files) {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    println!("cargo:rustc-env=SCANCRAFT_GIT_COMMIT={commit}");
    println!("cargo:rustc-env=SCANCRAFT_GIT_DIRTY={}", if dirty { "1" } else { "0" });
    println!("cargo:rustc-env=SCANCRAFT_BUILD_TARGET={}", std::env::var("TARGET").unwrap_or_default());
    println!("cargo:rustc-env=SCANCRAFT_BUILD_PROFILE={}", std::env::var("PROFILE").unwrap_or_default());
}
