//! Preferences > Compute: where the CFD solves run, this computer or a remote GPU over SSH, and the button that tests the
//! connection (and, on a computer with no worker yet, sets one up). The choice is kept in the user's settings (`Settings.compute`,
//! held by [`Ctx`]); the test runs on its own thread and reports each set-up step.

use std::time::Duration;

use iced::widget::{column, row, text, text_input};
use iced::{Element, Fill, Task, Widget};

use crate::analyze::cfd::remote::{self, Probe};
use crate::app::settings::Compute;
use crate::app::worker::{self, StreamHandle};
use crate::app::Ctx;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, TEXT_BODY};
use crate::widgets::button::{button as push, Weight};
use crate::widgets::segmented::segmented;

/// What the last connection test found.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Test {
    /// Not run (or the target changed since).
    #[default]
    Idle,
    /// Running, at this set-up step.
    Running(String),
    /// Connected: `NVIDIA GeForce RTX 3090 on omarchy`.
    Ok(String),
    /// Failed, with the reason.
    Failed(String),
}

/// A message of the Compute control.
#[derive(Debug, Clone)]
pub enum ComputeMessage {
    /// This computer (`false`) or a remote GPU (`true`) was chosen.
    Remote(bool),
    /// The target field was edited.
    Target(String),
    /// Test connection was pressed.
    Test,
    /// A step or the end of the test with this serial.
    Probe(u64, Probe),
}

/// The control's state.
#[derive(Default)]
pub struct ComputeState {
    /// The setting on show (a copy of `Ctx::compute`, so the view needs no context).
    pub compute: Compute,
    /// The target field's text; kept while the setting is Local so choosing Remote again finds it.
    pub text: String,
    /// The last connection test.
    pub test: Test,
    serial: u64,
    stream: Option<StreamHandle>,
}

impl ComputeState {
    /// Brings the control up to date with the setting (after a settings load).
    pub fn sync(&mut self, ctx: &Ctx) {
        if self.compute != ctx.compute {
            self.compute = ctx.compute.clone();
            if ctx.compute.is_remote() {
                self.text = ctx.compute.text();
            }
            self.test = Test::Idle;
        }
    }

    fn stop(&mut self) {
        if let Some(stream) = self.stream.take() {
            stream.abort();
        }
        self.serial += 1;
        self.test = Test::Idle;
    }

    /// Applies `compute` as the setting and keeps it in the user's settings file.
    fn choose(&mut self, compute: Compute, ctx: &mut Ctx) {
        self.stop();
        ctx.compute = compute.clone();
        ctx.compute_device = None;
        self.compute = compute;
        ctx.save_settings();
    }

    pub fn update(&mut self, msg: ComputeMessage, ctx: &mut Ctx) -> Task<ComputeMessage> {
        match msg {
            ComputeMessage::Remote(true) => self.choose(Compute::remote(&self.text), ctx),
            ComputeMessage::Remote(false) => self.choose(Compute::Local, ctx),
            ComputeMessage::Target(text) => {
                self.text = text;
                if self.compute.is_remote() {
                    self.choose(Compute::remote(&self.text), ctx);
                }
            }
            ComputeMessage::Test => {
                self.stop();
                self.test = Test::Running("Connecting".to_string());
                let compute = self.compute.clone();
                let (task, stream) = worker::spawn_stream(
                    self.serial,
                    None::<(Duration, fn() -> Probe)>,
                    move |_cancel, tx| remote::test(&compute, move |p| drop(tx.unbounded_send(p))),
                    ComputeMessage::Probe,
                );
                self.stream = Some(stream);
                return task;
            }
            ComputeMessage::Probe(serial, probe) if serial == self.serial => match probe {
                Probe::Status(step) => self.test = Test::Running(step),
                Probe::Done(result) => {
                    self.stream.take().inspect(StreamHandle::abort);
                    ctx.compute_device = result.as_ref().ok().cloned();
                    self.test = result.map_or_else(Test::Failed, Test::Ok);
                }
            },
            ComputeMessage::Probe(..) => {}
        }
        Task::none()
    }
}

fn note<'a>(s: String) -> Element<'a, ComputeMessage> {
    text(s).size(12.0).style(theme::muted).boxed()
}

/// The control: the choice, and for a remote GPU the target, the test button and what the test found.
pub fn view<'a>(ui: &'a ComputeState, _ctx: &'a Ctx) -> Element<'a, ComputeMessage> {
    let choice = segmented(&[(false, "This computer"), (true, "Remote GPU")], ui.compute.is_remote(), ComputeMessage::Remote);
    let mut col = column![choice].spacing(SPACE_1);
    if ui.compute.is_remote() {
        let field = text_input("user@host", &ui.text)
            .on_input(ComputeMessage::Target)
            .on_submit(ComputeMessage::Test)
            .font(MONO)
            .size(TEXT_BODY)
            .padding([4.0, SPACE_2])
            .style(theme::input)
            .width(Fill);
        let testing = matches!(ui.test, Test::Running(_));
        let test: Element<'_, ComputeMessage> = if testing {
            push("Testing…", Weight::Secondary).disabled("Connecting to the remote computer").into()
        } else if ui.compute.host().is_none() {
            push("Test connection", Weight::Secondary).disabled("Type the target first (user@host)").into()
        } else {
            push("Test connection", Weight::Secondary).on_press(ComputeMessage::Test).into()
        };
        col = col.push(row![field, test].spacing(SPACE_2).align_y(iced::Center).boxed());
        col = col.push(note("Solves run over SSH on the other computer's GPU; the first connection sets its solver up (minutes).".to_string()));
        match &ui.test {
            Test::Idle => {}
            Test::Running(step) => col = col.push(note(format!("{step}…"))),
            Test::Ok(device) => col = col.push(note(format!("Connected: {device}"))),
            Test::Failed(why) => col = col.push(note(why.clone())),
        }
    }
    col.boxed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;

    #[test]
    fn the_choice_is_kept_in_the_settings_file_and_a_typed_target_waits_for_remote() {
        let file = std::env::temp_dir().join(format!("bowshock_compute_prefs_{}.json", std::process::id()));
        let _ = std::fs::remove_file(&file);
        let mut app = App::with_example();
        app.ctx_mut().settings_file = Some(file.clone());
        let ctx = app.ctx_mut();
        let mut state = ComputeState::default();
        drop(state.update(ComputeMessage::Target("nobody@invalid.invalid".to_string()), ctx));
        assert_eq!(ctx.compute, Compute::Local, "typing a target does not switch to it");
        drop(state.update(ComputeMessage::Remote(true), ctx));
        let remote = Compute::Remote { target: "nobody@invalid.invalid".to_string(), port: None };
        assert_eq!(ctx.compute, remote);
        assert_eq!(crate::app::settings::Settings::load(Some(&file)).compute, remote);
        ctx.compute_device = Some("GPU on box".to_string());
        drop(state.update(ComputeMessage::Target("me@box:2200".to_string()), ctx));
        assert_eq!(ctx.compute, Compute::Remote { target: "me@box".to_string(), port: Some(2200) });
        assert_eq!(ctx.compute_device, None, "a new target is untested");
        drop(state.update(ComputeMessage::Remote(false), ctx));
        assert_eq!(crate::app::settings::Settings::load(Some(&file)).compute, Compute::Local);
        assert_eq!(state.text, "me@box:2200", "the field keeps its text");
        let _ = std::fs::remove_file(&file);
    }
}
