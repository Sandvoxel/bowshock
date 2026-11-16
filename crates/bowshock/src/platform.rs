//! Per-OS glue around the application icon and the console. Everything here is a no-op on the other platforms.

/// macOS: shows the Bowshock logo in the Dock and the Cmd-Tab switcher of a process that is not in an app bundle (`cargo run`,
/// a bare binary); inside `Bowshock.app` the bundle's `.icns` does that and nothing is set. Call on the main thread once the event
/// loop exists (iced boots the program after winit has created the `NSApplication`).
#[cfg(target_os = "macos")]
pub fn set_dock_icon() {
    use objc2::AnyThread;
    use objc2_app_kit::{NSApplication, NSImage};
    use objc2_foundation::{MainThreadMarker, NSData};

    let in_bundle = std::env::current_exe().is_ok_and(|exe| exe.to_string_lossy().contains(".app/Contents/MacOS/"));
    let Some(mtm) = MainThreadMarker::new() else { return };
    if in_bundle {
        return;
    }
    let data = NSData::with_bytes(&crate::icons::png(1024));
    let Some(image) = NSImage::initWithData(NSImage::alloc(), &data) else { return };
    // SAFETY: on the main thread (`mtm`) with a valid, retained image.
    unsafe { NSApplication::sharedApplication(mtm).setApplicationIconImage(Some(&image)) };
}

/// Not macOS: the window icon (`logo::window_icon`) and the embedded resource (`build.rs`) cover it.
#[cfg(not(target_os = "macos"))]
pub fn set_dock_icon() {}

/// Windows: release builds use the GUI subsystem (no console window), so a command-line run (`drive`, `import`, `--help`) would
/// lose its output. Attach to the console of the parent process, if there is one, and point unset standard handles at it.
#[cfg(windows)]
pub fn attach_console() {
    use std::ffi::c_void;
    use std::os::windows::io::AsRawHandle;

    #[link(name = "kernel32")]
    extern "system" {
        fn AttachConsole(process_id: u32) -> i32;
        fn GetStdHandle(std_handle: u32) -> *mut c_void;
        fn SetStdHandle(std_handle: u32, handle: *mut c_void) -> i32;
    }
    const ATTACH_PARENT_PROCESS: u32 = u32::MAX;
    const STD_OUTPUT_HANDLE: u32 = -11i32 as u32;
    const STD_ERROR_HANDLE: u32 = -12i32 as u32;
    const INVALID_HANDLE_VALUE: isize = -1;

    // SAFETY: plain kernel32 calls; the handle given to `SetStdHandle` is leaked so it stays open for the life of the process.
    unsafe {
        if AttachConsole(ATTACH_PARENT_PROCESS) == 0 {
            return;
        }
        for which in [STD_OUTPUT_HANDLE, STD_ERROR_HANDLE] {
            let current = GetStdHandle(which);
            if current.is_null() || current as isize == INVALID_HANDLE_VALUE {
                if let Ok(console) = std::fs::OpenOptions::new().write(true).open("CONOUT$") {
                    SetStdHandle(which, console.as_raw_handle());
                    std::mem::forget(console);
                }
            }
        }
    }
}

/// Not Windows: there is no GUI-subsystem split.
#[cfg(not(windows))]
pub fn attach_console() {}
