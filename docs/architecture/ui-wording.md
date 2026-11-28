# UI wording and button placement

Rules for every text and action button in the app (`apps/scancraft`). The shared widgets are in
`crates/scancraft-ui/src/kit`. Check new UI against this page.

## 1. Where buttons go

| Situation | Widget | Layout |
| --- | --- | --- |
| A card or modal that asks for a decision (tool cards, alignment proposals, close / quit) | `kit::dialog_footer` (cards), `kit::confirm_modal` (modals) | Primary action at the bottom right, Cancel directly to its left, a destructive action apart at the left |
| The same decision inside a panel body | `kit::action_row` | Same order as the footer |
| A command on a stage panel that starts something (Start Capture, Align All, Export) | `kit::wide_button` | Full width, one per row; primary style for the next step |
| Destructive command (Delete Pass, Discard Changes) | `Button::Danger` | Red ink and border; last in its group, apart from the others |
| Context menu | `kit::menu_item` | Destructive entries last, after a separator; a disabled entry says why on hover |

All of them use the same button height (`form_button_height`), 6 pt between neighbouring
buttons and 12 pt to the card edge.

A disabled button, ribbon button or menu entry always says why in its tooltip.

## 2. Capitals

- Commands (ribbon labels, buttons, menu entries, tool titles, app-bar tooltips): Title Case,
  "Reset Cleaning", "Delete Pass". An entry that opens a file dialog ends in "…".
- Everything else (notes, tooltips, status messages, row labels): sentence case.

## 3. Glossary

| Word | Meaning | Not |
| --- | --- | --- |
| **Pass** | One item under "Scan passes" in the browser: one continuous capture, or one imported scan file. Tools, alignment and Merge work on passes. | "scan" for the same thing |
| **Scan** | The activity and the Scan stage; the scanner; a scan *file* from outside ("Import Scan… adds a scan file as a new pass"). | |
| **Capture** | Recording frames into a pass: Start Capture, Stop Capture, "Stop the capture first". | "scanning", "recording" |
| **Frame** | One stored keyframe of a pass. | |
| **Merged model** | The point cloud Merge makes from the included passes. | "merge result", "project model" (that is what a pass is aligned onto) |
| **Mesh** | The triangle surface from Reconstruct or Make Solid. | "model", "surface" as a noun for the result |
| **Project** | Everything in one tab; on disk a project folder (`<name>.scancraft` + pass data) or a bundle (one `.scancraftz` file). | "session" |
| **Datum** | A plane, axis or point picked on the model that the project is oriented by ("Onto XY (floor)", "Flip"). | |

| Verb | Use for | Examples |
| --- | --- | --- |
| **Delete** | Removing an item from the project (undoable). | Delete Pass, Delete Merged Model, Delete Mesh |
| **Discard** | Throwing away something that is not kept yet, or leaving a frame out. | Discard Changes (closing a tab), Discard Frame / Restore Frame |
| **Reset** | Going back to the original state of something that stays. | Reset Cleaning, Reset Alignment, Reset (camera preset) |
| **Clear All** | Deleting every pass and result at once. | |
| **Accept / Reject** | Deciding on a proposal the app previews (Align Pass, Align All, Refine All). | not "Discard" |
| **Apply / Cancel** | A tool card that changes points or the mesh. Apply runs the tool with the settings shown. | not "OK" |
| **Connect, Export, Save, Align** | Cards whose action has its own name use it as the primary button. | |
| **Orient** | Turning and moving the whole project frame so the model sits square to the world axes, base on the grid. Not "align" — alignment places passes onto each other. | Auto Orient, Orient by Datums, "orient the project" |
| **Scale** | Giving an imported pass its true size from lengths measured on it. | Scale from Lengths…, "× 1.2016" |
| **Import Scan…** | Adding scan files (PLY, OBJ, STL) as new passes. | |
| **Add Project…** | Appending the passes of another ScanCraft project. | |
| **New Project / Open Project… / Save Project…** | Project files; New and Open use a new tab. The Save card saves as a project folder or a bundle (Save As Project Folder, Save as Bundle… in the notes). **Open Project Folder… / Add Project Folder…** (right-click on the app bar's Open / Add) pick the folder itself. | |
| **Save Anyway / Save As…** | The answers when the project file or bundle was saved by someone else since it was opened here ("Changed by Someone Else"): Save Anyway (destructive) replaces their version, Save As… opens the Save card with a free name and keeps both. | "Overwrite", "Force" |
| **Take Over / Open Read-Only** | The answers when another computer has the project open ("Project in Use"): Take Over (destructive) takes its lock, Open Read-Only opens a copy that is saved only to a new place; the tab is named "… (read-only)". Never offered for a ScanCraft running on this computer. | "Break lock", "Steal" |
| **Close / Quit** | Close a tab; quit the app. | |
| **Include in Merge / Leave Out of Merge** | Whether Merge uses a pass. | |
| **Shown / hidden** | The eye in the browser. Tools work on shown passes. | "visible" |

Fixed phrases for why something is disabled:

- "Another operation is running"
- "Stop the capture first"
- "Connect a scanner first"

## 4. Notes and tooltips

- A panel or card ends in at most one note of one or two lines: what the tool does, or what to
  do next.
- What a single setting does goes into that setting's tooltip.
- Do not write "undoable" in notes: every edit is.
- Values carry their unit; a size says what is measured ("mm rim" for the length around a hole).
