//! The command line.
//!
//! ```text
//! scancraft [<project>] [--mock | --connect <ip> | --replay <recording.bin> <camparam.yaml>]
//!           [--new-instance] [--auto <steps>] [--mock-modes pop4] [--open <project>]
//! scancraft --demo-project <dir>
//! ```
//!
//! `<project>` is a project file (`.scancraft`), a bundle (`.scancraftz`) or a project folder; it
//! opens in a tab. Explorer passes it on a double-click (the file association of
//! `packaging/windows/install.ps1`). `--open <project>` is the older spelling. Without
//! `--new-instance` (or `--auto`, a development run), a start while ScanCraft is already running
//! hands the project to that instance and ends (`crate::single_instance`).
//!
//! Such a second start passes on **only its project**. The options that set up a new app
//! (`--mock`, `--connect`, `--replay`, `--mock-modes`) are not applied to the running one, which
//! may be connected or capturing already: they are named in its status bar instead
//! ([`Cli::new_app_options`]), so the user knows to connect from the Connect stage. `--new-instance`
//! starts a second app that does use them (the driver's scanner lock still allows only one
//! stream).

use std::path::PathBuf;

use crate::app::StartConnection;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Cli {
    /// The project to open (absolute).
    pub project: Option<PathBuf>,
    pub start: StartConnection,
    /// Development: a scripted run (`dev::Script`).
    pub auto: Option<String>,
    /// Development: write a demo project and end.
    pub demo_project: Option<String>,
    /// Development: the simulator reports this scanner's mode table (`pop4`).
    pub mock_modes: Option<String>,
    /// Start a second app even when one is running.
    pub new_instance: bool,
    /// The options that set up a new app, as given (`--mock`, `--connect <ip>`, `--replay <a>
    /// <b>`, `--mock-modes <m>`): what a start that hands over to a running app does not apply.
    pub new_app_options: Vec<String>,
    /// Arguments that were not understood (reported, then ignored).
    pub unknown: Vec<String>,
}

impl Cli {
    /// Whether this start may hand its project to a running ScanCraft and end instead.
    pub fn single_instance(&self) -> bool {
        !self.new_instance && self.auto.is_none() && self.demo_project.is_none()
    }
}

/// Parses the arguments after the program name.
pub fn parse(args: &[String]) -> Cli {
    let mut cli = Cli { project: None, start: StartConnection::None, auto: None, demo_project: None, mock_modes: None, new_instance: false, new_app_options: Vec::new(), unknown: Vec::new() };
    let (mut replay, mut mock, mut connect) = (None, false, None);
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        let mut value = || it.next().cloned();
        match arg.as_str() {
            "--demo-project" => cli.demo_project = value(),
            "--open" => cli.project = value().map(PathBuf::from).or(cli.project.take()),
            "--auto" => cli.auto = value(),
            "--mock-modes" => {
                cli.mock_modes = value();
                cli.new_app_options.push(format!("--mock-modes {}", cli.mock_modes.as_deref().unwrap_or_default()));
            }
            "--connect" => {
                connect = value();
                cli.new_app_options.push(format!("--connect {}", connect.as_deref().unwrap_or_default()));
            }
            "--replay" => {
                let recording = value().unwrap_or_default();
                let camparam = it.next().cloned().unwrap_or_default();
                cli.new_app_options.push(format!("--replay {recording} {camparam}"));
                replay = Some((recording, camparam));
            }
            "--mock" => {
                mock = true;
                cli.new_app_options.push("--mock".into());
            }
            "--new-instance" => cli.new_instance = true,
            a if a.starts_with("--") => cli.unknown.push(a.to_string()),
            a if cli.project.is_none() && !a.is_empty() => cli.project = Some(PathBuf::from(a)),
            a => cli.unknown.push(a.to_string()),
        }
    }
    // The same precedence as before: a replay, then the simulator, then an address.
    cli.start = match (replay, mock, connect) {
        (Some((recording, camparam)), _, _) => StartConnection::Replay(recording.into(), camparam.into()),
        (None, true, _) => StartConnection::Mock,
        (None, false, Some(ip)) => StartConnection::Ip(ip),
        (None, false, None) => StartConnection::None,
    };
    cli.project = cli.project.map(|p| std::path::absolute(&p).unwrap_or(p));
    cli
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn a_project_path_opens_and_the_flags_keep_working() {
        let cli = parse(&args(&[r"C:\Scans\Bracket\Bracket.scancraft"]));
        assert_eq!(cli.project, Some(PathBuf::from(r"C:\Scans\Bracket\Bracket.scancraft")));
        assert_eq!(cli.start, StartConnection::None);
        assert!(cli.single_instance());

        let cli = parse(&args(&["--mock", "Part.scancraftz", "--auto", "wait:2,shot:a.png"]));
        assert_eq!(cli.start, StartConnection::Mock);
        assert_eq!(cli.project, Some(std::path::absolute("Part.scancraftz").unwrap()), "relative paths are made absolute");
        assert_eq!(cli.auto.as_deref(), Some("wait:2,shot:a.png"));
        assert!(!cli.single_instance(), "scripted runs never hand over");

        let cli = parse(&args(&["--connect", "192.168.1.20", "--new-instance", "--mock-modes", "pop4"]));
        assert_eq!(cli.start, StartConnection::Ip("192.168.1.20".into()));
        assert_eq!(cli.mock_modes.as_deref(), Some("pop4"));
        assert!(cli.project.is_none() && !cli.single_instance());

        let cli = parse(&args(&["--replay", "rec.bin", "cam.yaml", "--mock"]));
        assert_eq!(cli.start, StartConnection::Replay("rec.bin".into(), "cam.yaml".into()), "a replay wins over the simulator");

        let cli = parse(&args(&["--open", r"D:\old project", "--frobnicate", "extra"]));
        assert_eq!(cli.project, Some(PathBuf::from(r"D:\old project")));
        assert_eq!(cli.unknown, vec!["--frobnicate".to_string(), "extra".to_string()]);

        // What a second start that hands over does not apply, as given.
        let cli = parse(&args(&["Part.scancraftz", "--mock", "--connect", "10.0.0.7", "--mock-modes", "pop4"]));
        assert_eq!(cli.new_app_options, vec!["--mock".to_string(), "--connect 10.0.0.7".into(), "--mock-modes pop4".into()]);
        assert!(cli.single_instance(), "options alone still hand over (and are reported)");
        assert!(parse(&args(&["Part.scancraft"])).new_app_options.is_empty());

        let cli = parse(&args(&["--demo-project", "E:/demo"]));
        assert_eq!(cli.demo_project.as_deref(), Some("E:/demo"));
        assert!(!cli.single_instance());
        assert_eq!(parse(&[]).project, None);
    }
}
