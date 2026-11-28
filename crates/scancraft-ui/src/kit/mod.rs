//! ScanCraft's widget kit: SolidCraft's `ui_kit` look, re-implemented for ScanCraft's data.
pub mod chrome;
pub mod form;
pub mod nav;
pub mod ribbon;
pub mod tree;

pub use chrome::*;
pub use form::{Actions, ActionsResponse, Button, Choice, action_row, button, checkbox, dialog_footer, dropdown, dropdown_with_reasons, menu_item, note, note_with_tip, row, scope, section, sized_row, section_heading, segmented, slider, slider_row, status_line, value_field, value_text, wide_button};
pub use nav::{Kind as NavKind, NavButton, TriadPick, distance_gauge, nav_bar, triad};
pub use ribbon::{Cards, RIBBON_HEIGHT, RibbonButton, RibbonGroup, ribbon_card, workspace_card};
pub use tree::{Mark, Row, RowResp, TREE_ROW_HEIGHT, tree_row};
