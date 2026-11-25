//! Native file and folder dialogs (Windows Explorer dialogs through `rfd`).
//!
//! A dialog runs on its own thread so the window keeps drawing while it is open; the answer comes
//! back through a channel that the app polls once per frame. One dialog at a time.

use std::path::{Path, PathBuf};
use std::sync::mpsc;

/// What a dialog was opened for, and so what happens with its answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickPurpose {
    /// One or more PLY / OBJ / STL files, each added as a new pass.
    ImportScans,
    /// One or more ScanCraft project folders whose passes are appended.
    AddProjects,
    /// A project folder that replaces what is open.
    OpenProject,
    /// The folder Save Project writes into.
    SaveFolder,
    /// The file an export writes.
    ExportFile,
}

/// A dialog's answer: the purpose and the chosen paths (empty when cancelled).
pub struct Picked {
    pub purpose: PickPurpose,
    pub paths: Vec<PathBuf>,
}

/// The dialog that is open, if any.
#[derive(Default)]
pub struct FilePicker {
    pending: Option<mpsc::Receiver<Picked>>,
}

/// File types offered when exporting, by extension.
fn export_filter(extension: &str) -> (&'static str, &'static [&'static str]) {
    match extension {
        "obj" => ("Wavefront OBJ", &["obj"]),
        "stl" => ("STL", &["stl"]),
        _ => ("PLY", &["ply"]),
    }
}

/// The folder a dialog starts in: the folder of `hint` (or `hint` itself when it is a folder),
/// the nearest one that exists.
fn start_dir(hint: &str) -> Option<PathBuf> {
    let hint = hint.trim().trim_matches('"').trim();
    if hint.is_empty() {
        return None;
    }
    let mut dir = Some(Path::new(hint));
    while let Some(d) = dir {
        if d.is_dir() && d.is_absolute() {
            return Some(d.to_path_buf());
        }
        dir = d.parent();
    }
    None
}

impl FilePicker {
    pub fn is_open(&self) -> bool {
        self.pending.is_some()
    }

    /// Opens a dialog. `hint` is a path whose folder the dialog starts in (a previous choice);
    /// for an export it also gives the proposed file name. Does nothing while a dialog is open.
    pub fn open(&mut self, ctx: &egui::Context, purpose: PickPurpose, hint: &str) {
        if self.is_open() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        let ctx = ctx.clone();
        let dir = start_dir(hint);
        let name = Path::new(hint.trim().trim_matches('"')).file_name().map(|n| n.to_string_lossy().into_owned());
        let extension = Path::new(hint).extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
        std::thread::spawn(move || {
            let mut dialog = rfd::FileDialog::new();
            if let Some(dir) = &dir {
                dialog = dialog.set_directory(dir);
            }
            let paths: Vec<PathBuf> = match purpose {
                PickPurpose::ImportScans => dialog
                    .set_title("Import Scans")
                    .add_filter("Scans (PLY, OBJ, STL)", &["ply", "obj", "stl"])
                    .add_filter("All files", &["*"])
                    .pick_files()
                    .unwrap_or_default(),
                PickPurpose::AddProjects => dialog.set_title("Add ScanCraft Projects").pick_folders().unwrap_or_default(),
                PickPurpose::OpenProject => dialog.set_title("Open ScanCraft Project").pick_folder().into_iter().collect(),
                PickPurpose::SaveFolder => dialog.set_title("Save Project into Folder").pick_folder().into_iter().collect(),
                PickPurpose::ExportFile => {
                    let (label, ext) = export_filter(&extension);
                    let mut dialog = dialog.set_title("Export").add_filter(label, ext);
                    if let Some(name) = &name {
                        dialog = dialog.set_file_name(name);
                    }
                    dialog.save_file().into_iter().collect()
                }
            };
            let _ = tx.send(Picked { purpose, paths });
            ctx.request_repaint();
        });
        self.pending = Some(rx);
    }

    /// The answer of the open dialog once it has closed.
    pub fn poll(&mut self) -> Option<Picked> {
        let rx = self.pending.as_ref()?;
        match rx.try_recv() {
            Ok(picked) => {
                self.pending = None;
                Some(picked)
            }
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                self.pending = None;
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_dir_walks_up_to_an_existing_folder() {
        let tmp = std::env::temp_dir();
        let missing = tmp.join("scancraft-no-such-dir").join("scan.ply");
        assert_eq!(start_dir(&missing.display().to_string()), Some(tmp.clone()));
        assert_eq!(start_dir(&format!("\"{}\"", tmp.display())), Some(tmp));
        assert_eq!(start_dir(""), None);
        assert_eq!(start_dir("scan.ply"), None);
    }
}
