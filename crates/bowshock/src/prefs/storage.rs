//! Preferences > Storage: where Bowshock keeps things on this computer, how much room each takes, and `Open folder` for each. The
//! caches (the motors downloaded from ThrustCurve.org, the GPU kernel cache of Luma) also have `Clear`, behind a confirmation.
//! Sizes are measured on a worker thread (a kernel cache can hold thousands of files), so the page opens at once and the numbers
//! fill in.

use std::ffi::OsString;
use std::path::{Path, PathBuf};

use iced::widget::{column, container, row, text, Space};
use iced::{Element, Fill, Task, Widget};

use super::Message as PrefsMessage;
use crate::app::{self, worker, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::widgets::button::{button, Weight};

/// One place Bowshock keeps data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// `settings.json`: these preferences.
    Settings,
    /// The motors downloaded from ThrustCurve.org (the user motors directory, `<id>.motor` files).
    Motors,
    /// Luma's GPU kernel cache: compiled programs per graphics card and driver.
    Gpu,
}

impl Kind {
    const ALL: [Kind; 3] = [Kind::Settings, Kind::Motors, Kind::Gpu];

    fn title(self) -> &'static str {
        match self {
            Kind::Settings => "Settings",
            Kind::Motors => "Downloaded motors",
            Kind::Gpu => "GPU kernel cache",
        }
    }

    fn note(self) -> &'static str {
        match self {
            Kind::Settings => "These preferences: the unit set, theme and where solves run.",
            Kind::Motors => "Motors fetched from ThrustCurve.org and kept for offline use. A rocket that uses one needs it fetched again after clearing.",
            Kind::Gpu => "Compiled GPU programs for your graphics card. They are rebuilt on the next solve, which then starts a few seconds slower.",
        }
    }

    /// Whether the entry is a cache that can be cleared.
    fn clearable(self) -> bool {
        self != Kind::Settings
    }
}

/// What is known about the size of one entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Size {
    /// Being measured.
    Measuring,
    /// Nothing there (yet).
    Missing,
    /// Bytes on disk.
    Bytes(u64),
}

/// One row: where it is and how big.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    kind: Kind,
    path: Option<PathBuf>,
    size: Size,
}

/// The messages of the Storage page.
#[derive(Debug, Clone)]
pub enum Message {
    /// A size was measured (`None`: nothing at the path).
    Sized(Kind, Option<u64>),
    /// `Open folder`.
    OpenFolder(Kind),
    /// `Clear…`: ask first.
    Ask(Kind),
    /// The confirmation was declined.
    Cancel,
    /// The confirmation was accepted.
    Clear(Kind),
    /// The clearing finished.
    Cleared(Kind, Result<(), String>),
}

/// The page's state.
#[derive(Debug, Default)]
pub struct State {
    entries: Vec<Entry>,
    /// The entry whose `Clear` awaits its answer.
    confirm: Option<Kind>,
}

impl State {
    /// Reads where everything is from `ctx` and starts measuring each place.
    pub fn refresh(&mut self, ctx: &Ctx) -> Task<app::Message> {
        self.confirm = None;
        self.entries = Kind::ALL.iter().map(|&kind| Entry { kind, path: path_of(kind, ctx), size: Size::Measuring }).collect();
        Task::batch(self.entries.iter().map(|e| measure(e.kind, e.path.clone())))
    }

    /// Drops an unanswered confirmation (the page was left).
    pub fn leave(&mut self) {
        self.confirm = None;
    }
    /// The entry whose `Clear` awaits its answer (tests).
    #[cfg(test)]
    pub(super) fn confirm(&self) -> Option<Kind> {
        self.confirm
    }

    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Sized(kind, bytes) => {
                if let Some(e) = self.entries.iter_mut().find(|e| e.kind == kind) {
                    e.size = bytes.map_or(Size::Missing, Size::Bytes);
                }
            }
            Message::OpenFolder(kind) => {
                if let Some(folder) = self.entries.iter().find(|e| e.kind == kind).and_then(folder_of) {
                    return worker::perform(move || open_folder(&folder), |_| app::Message::Chrome(crate::chrome::Message::Nothing));
                }
            }
            Message::Ask(kind) => self.confirm = Some(kind),
            Message::Cancel => self.confirm = None,
            Message::Clear(kind) => {
                self.confirm = None;
                let Some(path) = self.entries.iter().find(|e| e.kind == kind).and_then(|e| e.path.clone()) else { return Task::none() };
                return worker::perform(
                    move || clear(kind, &path),
                    move |done| app::Message::Prefs(PrefsMessage::Storage(Message::Cleared(kind, done.unwrap_or_else(|| Err("the work stopped".into()))))),
                );
            }
            Message::Cleared(kind, result) => match result {
                Ok(()) => {
                    ctx.notify(format!("Cleared {}", kind.title().to_lowercase()));
                    let path = self.entries.iter().find(|e| e.kind == kind).and_then(|e| e.path.clone());
                    return measure(kind, path);
                }
                Err(e) => ctx.notify(format!("Could not clear {}: {e}", kind.title().to_lowercase())),
            },
        }
        Task::none()
    }

    /// The page body.
    pub fn view(&self) -> Element<'_, PrefsMessage> {
        let mut page = column![].spacing(SPACE_3);
        for (i, entry) in self.entries.iter().enumerate() {
            if i > 0 {
                page = page.push(container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed());
            }
            page = page.push(self.row(entry));
        }
        page.push(text("Cleared caches never touch your rockets or settings.").size(12.0).style(theme::faint).boxed()).boxed()
    }

    fn row<'a>(&'a self, entry: &'a Entry) -> Element<'a, PrefsMessage> {
        let kind = entry.kind;
        let path: Element<'a, Message> = match (&entry.path, kind) {
            (Some(p), _) => text(p.display().to_string()).font(MONO).size(TEXT_LABEL + 0.5).style(theme::faint).boxed(),
            (None, Kind::Settings) => text("Not kept in this session").size(TEXT_LABEL + 0.5).style(theme::faint).boxed(),
            (None, _) => text("Not available on this computer").size(TEXT_LABEL + 0.5).style(theme::faint).boxed(),
        };
        let size = match entry.size {
            Size::Measuring => "…".to_string(),
            Size::Missing => "empty".to_string(),
            Size::Bytes(b) => format_size(b),
        };
        let head =
            row![text(kind.title()).size(TEXT_BODY).style(theme::ink), Space::new().width(Fill), text(size).font(MONO).size(TEXT_BODY).style(theme::muted)]
                .align_y(iced::Center);
        let open: Element<'a, Message> = match folder_of(entry) {
            Some(_) => button("Open folder", Weight::Secondary).small().on_press(Message::OpenFolder(kind)).into(),
            None => button("Open folder", Weight::Secondary).small().disabled("There is no folder yet").into(),
        };
        let mut actions = row![open].spacing(SPACE_2).align_y(iced::Center);
        if kind.clearable() {
            if self.confirm == Some(kind) {
                actions = actions.push(text(format!("Clear {}?", kind.title().to_lowercase())).size(12.0).style(theme::muted).boxed());
                actions = actions.push(Element::from(button("Cancel", Weight::Ghost).small().on_press(Message::Cancel)));
                actions = actions.push(Element::from(button("Clear", Weight::Destructive).small().on_press(Message::Clear(kind))));
            } else if matches!(entry.size, Size::Bytes(b) if b > 0) {
                actions = actions.push(Element::from(button("Clear…", Weight::Secondary).small().on_press(Message::Ask(kind))));
            } else {
                actions = actions.push(Element::from(button("Clear…", Weight::Secondary).small().disabled("Nothing to clear")));
            }
        }
        let note = text(kind.note()).size(12.0).style(theme::muted);
        let body: Element<'a, Message> = column![head, path, note, actions].spacing(SPACE_1 + 2.0).boxed();
        Element::map(body, PrefsMessage::Storage).boxed()
    }
}

fn measure(kind: Kind, path: Option<PathBuf>) -> Task<app::Message> {
    let Some(path) = path else { return Task::done(app::Message::Prefs(PrefsMessage::Storage(Message::Sized(kind, None)))) };
    worker::perform(move || size_of(&path), move |bytes| app::Message::Prefs(PrefsMessage::Storage(Message::Sized(kind, bytes.flatten()))))
}

/// Where `kind` is kept for this session.
fn path_of(kind: Kind, ctx: &Ctx) -> Option<PathBuf> {
    match kind {
        Kind::Settings => ctx.settings_file.clone(),
        Kind::Motors => ctx.env.motors_dir.clone(),
        Kind::Gpu => luma_cache_dir(),
    }
}

/// The folder `Open folder` shows: the directory itself, or the one holding the file; `None` when it does not exist yet.
fn folder_of(entry: &Entry) -> Option<PathBuf> {
    let path = entry.path.as_deref()?;
    let folder = if entry.kind == Kind::Settings { path.parent()? } else { path };
    folder.is_dir().then(|| folder.to_path_buf())
}

/// Luma's cache directory, resolved as Luma does (`LUMA_CACHE_DIR`, `$XDG_CACHE_HOME/luma`, `%LOCALAPPDATA%\luma`, `~/.cache/luma`);
/// `None` when `LUMA_CACHE=off` or no home is known.
pub fn luma_cache_dir() -> Option<PathBuf> {
    resolve_luma(|name| std::env::var_os(name))
}

fn resolve_luma(var: impl Fn(&str) -> Option<OsString>) -> Option<PathBuf> {
    if var("LUMA_CACHE").is_some_and(|v| ["0", "off", "false"].contains(&v.to_string_lossy().to_ascii_lowercase().as_str())) {
        return None;
    }
    let set = |name: &str| var(name).filter(|v| !v.is_empty());
    if let Some(dir) = set("LUMA_CACHE_DIR") {
        return Some(PathBuf::from(dir));
    }
    if let Some(base) = set("XDG_CACHE_HOME").map(PathBuf::from).filter(|b| b.is_absolute()) {
        return Some(base.join("luma"));
    }
    if cfg!(windows) {
        if let Some(base) = set("LOCALAPPDATA") {
            return Some(PathBuf::from(base).join("luma"));
        }
    }
    set("HOME").or_else(|| set("USERPROFILE")).map(|home| PathBuf::from(home).join(".cache").join("luma"))
}

/// Bytes under `path` (a file or a directory tree); `None` when there is nothing there.
pub fn size_of(path: &Path) -> Option<u64> {
    let meta = std::fs::symlink_metadata(path).ok()?;
    if !meta.is_dir() {
        return Some(meta.len());
    }
    let mut total = 0;
    for entry in std::fs::read_dir(path).ok()?.flatten() {
        total += size_of(&entry.path()).unwrap_or(0);
    }
    Some(total)
}

/// Removes what a cache holds: the `.motor` files of the motors directory (and nothing else in it), or everything inside the GPU
/// cache directory. The directory itself stays.
fn clear(kind: Kind, dir: &Path) -> Result<(), String> {
    let only_motors = kind == Kind::Motors;
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(e.to_string()),
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if only_motors && path.extension().is_none_or(|e| e != bowshock_motors::CACHE_EXTENSION) {
            continue;
        }
        let removed = if path.is_dir() { std::fs::remove_dir_all(&path) } else { std::fs::remove_file(&path) };
        removed.map_err(|e| format!("{}: {e}", path.display()))?;
    }
    Ok(())
}

/// Shows `folder` in the system file manager.
fn open_folder(folder: &Path) {
    let program = if cfg!(target_os = "macos") {
        "open"
    } else if cfg!(windows) {
        "explorer"
    } else {
        "xdg-open"
    };
    let _ = std::process::Command::new(program).arg(folder).status();
}

/// `1.4 MB`, `812 kB`, `37 B`.
pub fn format_size(bytes: u64) -> String {
    let b = bytes as f64;
    if bytes < 1_000 {
        format!("{bytes} B")
    } else if bytes < 1_000_000 {
        format!("{:.0} kB", b / 1e3)
    } else if bytes < 1_000_000_000 {
        format!("{:.1} MB", b / 1e6)
    } else {
        format!("{:.2} GB", b / 1e9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("bowshock_storage_{}_{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn sizes_are_summed_over_the_tree_and_a_missing_path_has_none() {
        let dir = scratch("size");
        std::fs::write(dir.join("a.bin"), [0u8; 1500]).unwrap();
        std::fs::create_dir(dir.join("sub")).unwrap();
        std::fs::write(dir.join("sub").join("b.bin"), [0u8; 500]).unwrap();
        assert_eq!(size_of(&dir), Some(2000));
        assert_eq!(size_of(&dir.join("a.bin")), Some(1500));
        assert_eq!(size_of(&dir.join("nope")), None);
        assert_eq!(format_size(37), "37 B");
        assert_eq!(format_size(812_000), "812 kB");
        assert_eq!(format_size(9_100_000), "9.1 MB");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn clearing_motors_keeps_other_files_and_clearing_the_gpu_cache_empties_it() {
        let dir = scratch("clear");
        std::fs::write(dir.join("1.motor"), b"x").unwrap();
        std::fs::write(dir.join("2.motor"), b"x").unwrap();
        std::fs::write(dir.join("mine.eng"), b"x").unwrap();
        clear(Kind::Motors, &dir).unwrap();
        let left: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.file_name()).collect();
        assert_eq!(left, [OsString::from("mine.eng")]);
        clear(Kind::Gpu, &dir).unwrap();
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 0);
        assert!(dir.is_dir(), "the directory stays");
        assert!(clear(Kind::Gpu, &dir.join("missing")).is_ok(), "nothing to clear is fine");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_gpu_cache_directory_follows_lumas_rules() {
        let env = |pairs: &'static [(&'static str, &'static str)]| move |name: &str| pairs.iter().find(|(k, _)| *k == name).map(|(_, v)| OsString::from(*v));
        assert_eq!(resolve_luma(env(&[("LUMA_CACHE_DIR", "/x/y"), ("HOME", "/h")])), Some(PathBuf::from("/x/y")));
        assert_eq!(resolve_luma(env(&[("XDG_CACHE_HOME", "/c"), ("HOME", "/h")])), Some(PathBuf::from("/c/luma")));
        assert_eq!(resolve_luma(env(&[("HOME", "/h")])), Some(PathBuf::from("/h/.cache/luma")));
        assert_eq!(resolve_luma(env(&[("HOME", "/h"), ("LUMA_CACHE", "off")])), None);
        assert_eq!(resolve_luma(env(&[])), None);
    }

    #[test]
    fn clear_asks_first_and_the_answer_is_kept_per_entry() {
        let mut ctx = Ctx::new();
        let mut state = State::default();
        let _ = state.refresh(&ctx);
        assert_eq!(state.entries.len(), 3);
        assert_eq!(state.confirm, None);
        let _ = state.update(Message::Ask(Kind::Gpu), &mut ctx);
        assert_eq!(state.confirm, Some(Kind::Gpu));
        let _ = state.update(Message::Cancel, &mut ctx);
        assert_eq!(state.confirm, None);
        let _ = state.update(Message::Ask(Kind::Motors), &mut ctx);
        state.leave();
        assert_eq!(state.confirm, None, "leaving the page drops an unanswered question");
    }
}
