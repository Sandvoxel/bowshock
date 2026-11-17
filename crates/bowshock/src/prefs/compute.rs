//! Preferences > Compute: which computers the CFD solves run on. This computer and every machine added here (a remote GPU over SSH)
//! has a switch; a machine that is off is never contacted. The solves either use one machine (picked from those that are on) or
//! share the work: every machine that is on takes the next Mach number when it is free, so a faster one does more.
//!
//! Each machine has its own `Test connection`, which sets its worker up on first use and reports each step, then `<gpu> on
//! <host>` or why not. The choice is kept in the user's settings (`Settings.compute`, held by [`Ctx`]); a test runs on its own
//! thread.

use std::collections::BTreeMap;
use std::fmt;
use std::time::Duration;

use iced::widget::{column, container, row, text, text_input};
use iced::{Element, Fill, Task, Widget};

use super::{card, group, setting};
use crate::analyze::cfd::remote::{self, Probe};
use crate::app::settings::{Machine, MachineRef, Mode};
use crate::app::worker::{self, StreamHandle};
use crate::app::Ctx;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY};
use crate::widgets::button::{button as push, Weight};
use crate::widgets::field::{checkbox, select};
use crate::widgets::segmented::segmented;

/// What the last connection test of a machine found.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Test {
    /// Not run.
    #[default]
    Idle,
    /// Running, at this set-up step.
    Running(String),
    /// Connected: `NVIDIA GeForce RTX 3090 on omarchy`.
    Ok(String),
    /// Failed, with the reason.
    Failed(String),
}

/// One entry of the "Use one" select.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Choice {
    /// The machine.
    pub which: MachineRef,
    /// Its name.
    pub name: String,
}

impl fmt::Display for Choice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.name)
    }
}

/// A message of the Compute page.
#[derive(Debug, Clone)]
pub enum ComputeMessage {
    /// This computer takes part, or not.
    UseThis(bool),
    /// The machine with this key is switched on or off.
    Toggle(String, bool),
    /// The machine's name was edited (empty: the target names it).
    Label(String, String),
    /// Remove the machine.
    Remove(String),
    /// The Add machine field was edited.
    AddText(String),
    /// Add the machine typed in.
    Add,
    /// Share the work over every machine that is on (`true`), or use one (`false`).
    Share(bool),
    /// Use this machine.
    Pick(Choice),
    /// Test the connection to the machine with this key.
    Test(String),
    /// A step or the end of the test (key, the test's serial, what it reports).
    Probe(String, u64, Probe),
}

/// The page's state: the Add machine field and the tests.
#[derive(Default)]
pub struct ComputeState {
    /// The text of the Add machine field.
    pub add_text: String,
    tests: BTreeMap<String, Test>,
    runs: BTreeMap<String, (u64, StreamHandle)>,
    serial: u64,
}

impl ComputeState {
    /// Brings the page up to date with the setting (after a settings load, or a machine that went): the tests of machines that are
    /// no longer there are dropped.
    pub fn sync(&mut self, ctx: &Ctx) {
        let gone: Vec<String> = self.tests.keys().filter(|k| ctx.compute.machine(k).is_none()).cloned().collect();
        for key in gone {
            self.forget(&key);
        }
    }

    /// The last test of a machine.
    pub fn test_of(&self, key: &str) -> &Test {
        self.tests.get(key).unwrap_or(&Test::Idle)
    }

    fn forget(&mut self, key: &str) {
        if let Some((_, stream)) = self.runs.remove(key) {
            stream.abort();
        }
        self.tests.remove(key);
    }

    /// Keeps the settings consistent and writes them.
    fn changed(ctx: &mut Ctx) {
        ctx.compute.normalise();
        ctx.save_settings();
    }

    pub fn update(&mut self, msg: ComputeMessage, ctx: &mut Ctx) -> Task<ComputeMessage> {
        match msg {
            ComputeMessage::UseThis(on) => {
                ctx.compute.use_this_computer = on;
                Self::changed(ctx);
            }
            ComputeMessage::Toggle(key, on) => {
                if let Some(m) = ctx.compute.machines.iter_mut().find(|m| m.key() == key) {
                    m.enabled = on;
                }
                if !on {
                    // a machine that is off is not contacted: not even by a test still running
                    self.forget(&key);
                }
                Self::changed(ctx);
            }
            ComputeMessage::Label(key, label) => {
                if let Some(m) = ctx.compute.machines.iter_mut().find(|m| m.key() == key) {
                    m.label = (!label.trim().is_empty()).then_some(label);
                }
                ctx.save_settings();
            }
            ComputeMessage::Remove(key) => {
                self.forget(&key);
                ctx.compute.machines.retain(|m| m.key() != key);
                ctx.compute_devices.remove(&key);
                Self::changed(ctx);
            }
            ComputeMessage::AddText(text) => self.add_text = text,
            ComputeMessage::Add => {
                let machine = Machine::parse(&self.add_text);
                if machine.host().is_some() && ctx.compute.machine(&machine.key()).is_none() {
                    ctx.compute.machines.push(machine);
                    self.add_text.clear();
                    Self::changed(ctx);
                }
            }
            ComputeMessage::Share(share) => {
                ctx.compute.mode = if share { Mode::Share } else { Mode::Pick(MachineRef::This) };
                Self::changed(ctx);
            }
            ComputeMessage::Pick(choice) => {
                ctx.compute.mode = Mode::Pick(choice.which);
                Self::changed(ctx);
            }
            ComputeMessage::Test(key) => {
                let Some(machine) = ctx.compute.machine(&key).filter(|m| m.enabled) else { return Task::none() };
                let (Some(host), name) = (machine.host(), machine.name()) else { return Task::none() };
                self.forget(&key);
                ctx.compute_devices.remove(&key);
                self.serial += 1;
                self.tests.insert(key.clone(), Test::Running("Connecting".to_string()));
                let wrap_key = key.clone();
                let (task, stream) = worker::spawn_stream(
                    self.serial,
                    None::<(Duration, fn() -> Probe)>,
                    move |_cancel, tx| remote::test(&host, &name, move |p| drop(tx.unbounded_send(p))),
                    move |serial, probe| ComputeMessage::Probe(wrap_key.clone(), serial, probe),
                );
                self.runs.insert(key, (self.serial, stream));
                return task;
            }
            ComputeMessage::Probe(key, serial, probe) => {
                if self.runs.get(&key).is_none_or(|(s, _)| *s != serial) {
                    return Task::none();
                }
                match probe {
                    Probe::Status(step) => {
                        self.tests.insert(key, Test::Running(step));
                    }
                    Probe::Done(result) => {
                        if let Some((_, stream)) = self.runs.remove(&key) {
                            stream.abort();
                        }
                        match result {
                            Ok(device) => {
                                ctx.compute_devices.insert(key.clone(), device.clone());
                                self.tests.insert(key, Test::Ok(device));
                            }
                            Err(why) => {
                                ctx.compute_devices.remove(&key);
                                self.tests.insert(key, Test::Failed(why));
                            }
                        }
                    }
                }
            }
        }
        Task::none()
    }
}

fn note<'a>(s: String) -> Element<'a, ComputeMessage> {
    text(s).size(12.0).style(theme::muted).boxed()
}

/// One machine's row: its switch, its name, its target, Test connection and Remove, and under them what its test found.
fn machine_row<'a>(m: &'a Machine, test: &'a Test) -> Element<'a, ComputeMessage> {
    let key = m.key();
    let on = checkbox("", m.enabled, {
        let key = key.clone();
        move |on| ComputeMessage::Toggle(key.clone(), on)
    });
    let label = text_input("name (optional)", m.label.as_deref().unwrap_or(""))
        .on_input({
            let key = key.clone();
            move |l| ComputeMessage::Label(key.clone(), l)
        })
        .size(12.0)
        .padding([2.0, SPACE_2])
        .style(theme::input)
        .width(200);
    let target = text(key.clone()).font(MONO).size(TEXT_BODY).style(if m.enabled { theme::muted } else { theme::faint }).width(Fill);
    let testing = matches!(test, Test::Running(_));
    let try_it: Element<'a, ComputeMessage> = if !m.enabled {
        push("Test connection", Weight::Secondary).small().disabled("Switch the machine on to test it").into()
    } else if testing {
        push("Testing…", Weight::Secondary).small().disabled("Connecting to the machine").into()
    } else {
        push("Test connection", Weight::Secondary).small().on_press(ComputeMessage::Test(key.clone())).into()
    };
    let remove: Element<'a, ComputeMessage> = push("Remove", Weight::Ghost).small().on_press(ComputeMessage::Remove(key)).into();
    let mut col = column![
        row![on, target, try_it, remove].spacing(SPACE_2).align_y(iced::Center),
        row![text("").width(28), label].spacing(SPACE_2).align_y(iced::Center)
    ]
    .spacing(SPACE_1);
    let status = match test {
        Test::Idle => None,
        Test::Running(step) => Some(format!("{step}…")),
        Test::Ok(device) => Some(format!("Connected: {device}")),
        Test::Failed(why) => Some(why.clone()),
    };
    if let Some(status) = status {
        col = col.push(row![text("").width(28), note(status)].boxed());
    }
    padded(col.boxed())
}

/// `body` as a row of a card.
fn padded<'a>(body: Element<'a, ComputeMessage>) -> Element<'a, ComputeMessage> {
    container(body).padding(SPACE_3).width(Fill).boxed()
}

/// The page: the machines with their switches, Add machine, and how the solves use them.
pub fn view<'a>(ui: &'a ComputeState, ctx: &'a Ctx) -> Element<'a, ComputeMessage> {
    let compute = &ctx.compute;
    let share = compute.mode == Mode::Share;

    let this = row![
        checkbox("This computer", compute.use_this_computer, ComputeMessage::UseThis),
        text("its GPU, or its CPU (see below)").size(12.0).style(theme::faint)
    ]
    .spacing(SPACE_3)
    .align_y(iced::Center);
    let mut rows = vec![padded(this.boxed())];
    for m in &compute.machines {
        rows.push(machine_row(m, ui.test_of(&m.key())));
    }

    let adding = Machine::parse(&ui.add_text);
    let can_add = adding.host().is_some() && compute.machine(&adding.key()).is_none();
    let field = text_input("user@host (or user@host:port)", &ui.add_text)
        .on_input(ComputeMessage::AddText)
        .on_submit_maybe(can_add.then_some(ComputeMessage::Add))
        .font(MONO)
        .size(TEXT_BODY)
        .padding([4.0, SPACE_2])
        .style(theme::input)
        .width(Fill);
    let add: Element<'a, ComputeMessage> = if can_add {
        push("Add machine", Weight::Secondary).on_press(ComputeMessage::Add).into()
    } else {
        push("Add machine", Weight::Secondary).disabled("Type the machine's user@host first").into()
    };
    rows.push(padded(row![field, add].spacing(SPACE_2).align_y(iced::Center).boxed()));

    let how = segmented(&[(false, "Use one"), (true, "Share across all switched-on machines")], share, ComputeMessage::Share);
    let mut use_rows = vec![setting("Use", None, how)];
    let mut under = column![].spacing(SPACE_1);
    if share {
        let n = compute.places().map_or(0, |p| p.len());
        under = under.push(note(format!(
            "Each Mach number goes to the next machine that is free, so a faster one does more. {n} machine{} on now. A machine that cannot connect is left out; one that fails mid-solve hands its Mach number to the others.",
            if n == 1 { "" } else { "s" }
        )));
    } else {
        let choices: Vec<Choice> = compute.pickable().into_iter().map(|(which, name)| Choice { which, name }).collect();
        let picked = match &compute.mode {
            Mode::Pick(r) => choices.iter().find(|c| &c.which == r).cloned(),
            Mode::Share => None,
        };
        use_rows.push(setting("Machine", None, container(select(choices, picked, ComputeMessage::Pick)).width(220).boxed()));
        if compute.pickable().is_empty() {
            under = under.push(note("Nothing is switched on: switch This computer or a machine on.".to_string()));
        }
    }
    let how_card = column![card(use_rows), under].spacing(SPACE_2);
    column![
        group("WHERE SOLVES RUN", card(rows), Some("Machines are used over SSH with your keys; the first connection sets up their solver (minutes).")),
        how_card
    ]
    .spacing(SPACE_3)
    .boxed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::settings::Settings;
    use crate::app::App;

    fn press(state: &mut ComputeState, ctx: &mut Ctx, m: ComputeMessage) {
        drop(state.update(m, ctx));
    }

    #[test]
    fn machines_are_added_switched_labelled_and_removed_and_every_change_is_kept() {
        let file = std::env::temp_dir().join(format!("bowshock_compute_prefs_{}.json", std::process::id()));
        let _ = std::fs::remove_file(&file);
        let mut app = App::with_example();
        app.ctx_mut().settings_file = Some(file.clone());
        let ctx = app.ctx_mut();
        let mut state = ComputeState::default();
        let kept = || Settings::load(Some(&file)).compute;

        press(&mut state, ctx, ComputeMessage::AddText("nobody@invalid.invalid".into()));
        assert!(ctx.compute.machines.is_empty(), "typing adds nothing");
        press(&mut state, ctx, ComputeMessage::Add);
        press(&mut state, ctx, ComputeMessage::AddText("me@box:2200".into()));
        press(&mut state, ctx, ComputeMessage::Add);
        press(&mut state, ctx, ComputeMessage::AddText("me@box:2200".into()));
        press(&mut state, ctx, ComputeMessage::Add);
        assert_eq!(ctx.compute.machines.iter().map(Machine::key).collect::<Vec<_>>(), ["nobody@invalid.invalid", "me@box:2200"], "no duplicate");
        assert!(state.add_text.is_empty() || state.add_text == "me@box:2200");
        assert_eq!(kept(), ctx.compute);

        // Pick one: the machine; then switch it off and the pick moves to one that can be used.
        press(&mut state, ctx, ComputeMessage::Pick(Choice { which: MachineRef::Machine("me@box:2200".into()), name: "me@box:2200".into() }));
        assert_eq!(kept().mode, Mode::Pick(MachineRef::Machine("me@box:2200".into())));
        press(&mut state, ctx, ComputeMessage::Toggle("me@box:2200".into(), false));
        assert_eq!(ctx.compute.mode, Mode::Pick(MachineRef::This));
        assert!(!kept().machines[1].enabled);

        // Share: the machines that are on.
        press(&mut state, ctx, ComputeMessage::Share(true));
        assert_eq!(ctx.compute.places().unwrap().len(), 2, "this computer and the one that is on");
        press(&mut state, ctx, ComputeMessage::UseThis(false));
        assert_eq!(kept().places().unwrap().len(), 1);
        press(&mut state, ctx, ComputeMessage::Label("nobody@invalid.invalid".into(), "Lab".into()));
        assert_eq!(kept().machines[0].name(), "Lab");
        press(&mut state, ctx, ComputeMessage::Label("nobody@invalid.invalid".into(), " ".into()));
        assert_eq!(kept().machines[0].name(), "nobody@invalid.invalid");

        // A machine that is off is not tested, and one that is removed goes.
        press(&mut state, ctx, ComputeMessage::Test("me@box:2200".into()));
        assert_eq!(state.test_of("me@box:2200"), &Test::Idle);
        press(&mut state, ctx, ComputeMessage::Remove("me@box:2200".into()));
        assert_eq!(kept().machines.len(), 1);
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn a_test_reports_its_steps_and_its_end_per_machine_and_a_stale_one_is_ignored() {
        let mut app = App::with_example();
        let ctx = app.ctx_mut();
        ctx.settings_file = None;
        ctx.compute.machines = vec![Machine::parse("a@x"), Machine::parse("b@y")];
        let mut state = ComputeState::default();
        // `Test` starts a thread; here the messages it would send are played by hand.
        state.runs.insert("a@x".into(), (7, worker::spawn_iter(7, Vec::<()>::new(), |_, ()| ()).1));
        state.tests.insert("a@x".into(), Test::Running("Connecting".into()));
        press(&mut state, ctx, ComputeMessage::Probe("a@x".into(), 6, Probe::Done(Ok("stale".into()))));
        assert_eq!(state.test_of("a@x"), &Test::Running("Connecting".into()), "another serial is ignored");
        press(&mut state, ctx, ComputeMessage::Probe("a@x".into(), 7, Probe::Status("Building the worker".into())));
        assert_eq!(state.test_of("a@x"), &Test::Running("Building the worker".into()));
        press(&mut state, ctx, ComputeMessage::Probe("a@x".into(), 7, Probe::Done(Ok("RTX 3080 on omarchy".into()))));
        assert_eq!(state.test_of("a@x"), &Test::Ok("RTX 3080 on omarchy".into()));
        assert_eq!(ctx.compute_devices.get("a@x").map(String::as_str), Some("RTX 3080 on omarchy"));
        assert_eq!(state.test_of("b@y"), &Test::Idle, "the other machine is untouched");
        // a failure drops what the machine was known to be
        state.runs.insert("a@x".into(), (8, worker::spawn_iter(8, Vec::<()>::new(), |_, ()| ()).1));
        press(&mut state, ctx, ComputeMessage::Probe("a@x".into(), 8, Probe::Done(Err("Remote GPU a@x: could not connect".into()))));
        assert_eq!(state.test_of("a@x"), &Test::Failed("Remote GPU a@x: could not connect".into()));
        assert!(ctx.compute_devices.is_empty());
    }
}
