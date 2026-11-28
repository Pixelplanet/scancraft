// Shared by `build.rs` (included there) and the app's tests: what the build script embeds about
// the code an app was built from, and when it must look again.

/// The files whose change must make the build script look at the checkout again, given the
/// top-level folder of the checkout. Cargo reruns a build script that names any file only when
/// one of those changes, not on edits of the package's sources, so the sources are named too
/// (folders: Cargo scans them for any modified file): an edit that is not committed and not
/// staged changes none of the git files, and the app would claim a clean commit.
pub fn rerun_paths(top: &std::path::Path, git_files: &[String]) -> Vec<std::path::PathBuf> {
    let mut paths: Vec<std::path::PathBuf> = git_files.iter().map(std::path::PathBuf::from).collect();
    for p in ["apps", "crates", "Cargo.toml", "Cargo.lock"] {
        let p = top.join(p);
        if p.exists() {
            paths.push(p);
        }
    }
    paths
}

/// Commit and dirty flag to embed. `commit_override` / `dirty_override`: the
/// `SCANCRAFT_GIT_COMMIT` / `SCANCRAFT_GIT_DIRTY` variables (a build outside a checkout says
/// itself whether its sources were changed: `1` or `0`); `head` and `status`: what git says
/// (`status` is the porcelain list of changed tracked files). The dirty flag comes from git
/// whenever git can tell, also with a commit override.
pub fn commit_and_dirty(commit_override: Option<&str>, dirty_override: Option<&str>, head: Option<&str>, status: Option<&str>) -> (String, bool) {
    let commit = match commit_override.map(str::trim).filter(|c| !c.is_empty()) {
        Some(c) => c.to_string(),
        None => head.map(str::trim).filter(|h| !h.is_empty()).unwrap_or("unknown").to_string(),
    };
    let dirty = match (status, dirty_override.map(str::trim)) {
        (Some(s), _) => !s.trim().is_empty(),
        (None, Some(d)) => d == "1" || d.eq_ignore_ascii_case("true"),
        (None, None) => false,
    };
    (commit, dirty)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unstaged_edit_of_any_source_makes_the_build_script_look_again() {
        let top = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let paths = rerun_paths(&top, &["/repo/.git/HEAD".into(), "/repo/.git/index".into()]);
        for wanted in ["apps", "crates", "Cargo.toml", "Cargo.lock"] {
            assert!(paths.iter().any(|p| p.ends_with(wanted)), "{wanted} is not watched: {paths:?}");
        }
        assert!(paths.iter().any(|p| p.ends_with("index")), "the git files stay watched");
    }

    #[test]
    fn the_dirty_flag_comes_from_git_also_with_a_commit_override() {
        let changed = Some(" M crates/scancraft-driver/src/recording.rs");
        assert_eq!(commit_and_dirty(None, None, Some("abc123"), changed), ("abc123".into(), true));
        assert_eq!(commit_and_dirty(None, None, Some("abc123"), Some("")), ("abc123".into(), false));
        assert_eq!(commit_and_dirty(Some("release-1"), None, Some("abc123"), changed), ("release-1".into(), true), "an override does not hide changes");
        // Outside a checkout the build says it itself.
        assert_eq!(commit_and_dirty(Some("release-1"), Some("1"), None, None), ("release-1".into(), true));
        assert_eq!(commit_and_dirty(Some("release-1"), None, None, None), ("release-1".into(), false));
        assert_eq!(commit_and_dirty(None, None, None, None).0, "unknown");
    }
}
