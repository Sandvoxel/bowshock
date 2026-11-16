//! Native file dialogs as tasks: the only place that talks to `rfd`.

use std::path::PathBuf;

use iced::Task;

/// A filter: label and extensions.
type Filter = (&'static str, &'static [&'static str]);

/// Pick one file; `None` when cancelled.
pub fn pick_file<M: Send + 'static>(title: &'static str, filters: &'static [Filter], map: impl FnOnce(Option<PathBuf>) -> M + Send + 'static) -> Task<M> {
    Task::perform(
        async move {
            let mut dialog = rfd::AsyncFileDialog::new().set_title(title);
            for (label, extensions) in filters {
                dialog = dialog.add_filter(*label, extensions);
            }
            dialog.pick_file().await.map(|f| f.path().to_path_buf())
        },
        map,
    )
}

/// Pick any number of files.
pub fn pick_files<M: Send + 'static>(title: &'static str, filter: Filter, map: impl FnOnce(Vec<PathBuf>) -> M + Send + 'static) -> Task<M> {
    Task::perform(
        async move {
            let files = rfd::AsyncFileDialog::new().set_title(title).add_filter(filter.0, filter.1).pick_files().await;
            files.unwrap_or_default().into_iter().map(|f| f.path().to_path_buf()).collect::<Vec<_>>()
        },
        map,
    )
}

/// Pick a save path. A set `override_path` (`BOWSHOCK_EXPORT_FILE`) answers at once with no dialog: headless runs cannot answer one.
pub fn save_file<M: Send + 'static>(
    override_path: Option<PathBuf>,
    title: &'static str,
    default_name: String,
    filter: Filter,
    map: impl FnOnce(Option<PathBuf>) -> M + Send + 'static,
) -> Task<M> {
    if let Some(path) = override_path {
        return Task::done(map(Some(path)));
    }
    Task::perform(
        async move {
            let file = rfd::AsyncFileDialog::new().set_title(title).set_file_name(default_name).add_filter(filter.0, filter.1).save_file().await;
            file.map(|f| f.path().to_path_buf())
        },
        map,
    )
}

/// Pick a file and read its bytes: `None` when cancelled, `Err` when unreadable.
pub fn pick_bytes<M: Send + 'static>(title: &'static str, filter: Filter, map: impl FnOnce(Option<Result<Vec<u8>, String>>) -> M + Send + 'static) -> Task<M> {
    Task::perform(
        async move {
            let file = rfd::AsyncFileDialog::new().set_title(title).add_filter(filter.0, filter.1).pick_file().await?;
            Some(std::fs::read(file.path()).map_err(|e| e.to_string()))
        },
        map,
    )
}

/// Pick a text file and read it: `Ok(Some((file name, text)))`, `Ok(None)` when cancelled, `Err` when unreadable.
pub fn pick_named_text<M: Send + 'static>(
    title: String,
    filter: Filter,
    map: impl FnOnce(Result<Option<(String, String)>, String>) -> M + Send + 'static,
) -> Task<M> {
    Task::perform(
        async move {
            let Some(file) = rfd::AsyncFileDialog::new().set_title(title).add_filter(filter.0, filter.1).pick_file().await else {
                return Ok(None);
            };
            let name = file.file_name();
            std::fs::read_to_string(file.path()).map(|text| Some((name, text))).map_err(|e| e.to_string())
        },
        map,
    )
}
