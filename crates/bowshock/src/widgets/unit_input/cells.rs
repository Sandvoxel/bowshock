/// One cell of a formula being edited: a character, a reference chip, or an empty chip waiting for a pick.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum Cell {
    Char(char),
    /// A reference, by its token (`body_tube#3.od`).
    Chip(String),
    /// A chip being re-picked; the reference it replaces comes back on Esc.
    Slot(Option<String>),
}

/// The typographic form of an operator character for display: `*` → `×`, `/` → `÷`, `-` → `−`. Text is stored as ASCII.
pub(super) fn display_char(c: char) -> char {
    match c {
        '*' => '\u{d7}',
        '/' => '\u{f7}',
        '-' => '\u{2212}',
        c => c,
    }
}

/// The ASCII operator for a typed `×`, `÷` or `−` (everything else unchanged), so both spellings are accepted.
pub(super) fn ascii_operator(c: char) -> char {
    match c {
        '\u{d7}' => '*',
        '\u{f7}' => '/',
        '\u{2212}' => '-',
        c => c,
    }
}

/// Turns text with `{token}`s into cells: each complete `{..}` is a chip, an unclosed `{` stays a character.
pub(super) fn cells_from(text: &str) -> Vec<Cell> {
    let mut cells = Vec::new();
    let mut rest = text;
    while let Some(c) = rest.chars().next() {
        match (c, rest.find('}')) {
            ('{', Some(end)) if !rest[1..end].contains('{') => {
                cells.push(Cell::Chip(rest[1..end].trim().to_owned()));
                rest = &rest[end + 1..];
            }
            _ => {
                cells.push(Cell::Char(ascii_operator(c)));
                rest = &rest[c.len_utf8()..];
            }
        }
    }
    cells
}

/// The text form of cells: chips as `{token}`, a slot as the open `{` it stands for.
pub(super) fn cells_text(cells: &[Cell]) -> String {
    cells
        .iter()
        .map(|c| match c {
            Cell::Char(c) => c.to_string(),
            Cell::Chip(t) => format!("{{{t}}}"),
            Cell::Slot(_) => "{".to_owned(),
        })
        .collect()
}

/// Where the pick will land: a slot, else the last `{` character (an open brace is one with no `}` after it).
pub(super) fn open_brace(cells: &[Cell]) -> Option<usize> {
    if let Some(i) = cells.iter().position(|c| matches!(c, Cell::Slot(_))) {
        return Some(i);
    }
    let open = cells.iter().rposition(|c| *c == Cell::Char('{'))?;
    (!cells[open..].contains(&Cell::Char('}'))).then_some(open)
}

/// Chip metrics.
pub(super) const CHIP_FONT: f32 = 11.5;
pub(super) const CHIP_PAD: f32 = 5.0;
pub(super) const CHIP_SQUARE: f32 = 7.0;
pub(super) const CHIP_H: f32 = 18.0;
pub(super) const CHIP_GAP: f32 = 2.0;
