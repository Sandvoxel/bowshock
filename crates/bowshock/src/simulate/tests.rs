use std::sync::Arc;
use std::time::Duration;

use super::flight::{self, Flight, MotorChoice};
use super::*;
use crate::app::App;

/// Flies flight `id` synchronously the way the worker thread does, and delivers the result.
fn fly(state: &mut State, ctx: &Ctx, id: FlightId) {
    let _task = state.launch(id, ctx);
    let f = state.flight(id).unwrap();
    let (ticket, spec) = (f.ticket, f.spec.clone());
    let (input, delay) = flight::prepare(ctx, &spec).expect("the example flies");
    let started = Instant::now();
    let result = Arc::new(bowshock_sim::simulate(&input));
    let revision = state.stamp(ctx, &spec);
    state.finished(Done { flight: id, ticket, spec, revision, delay, dt: input.dt, outcome: Ok((result, started.elapsed())) });
}

fn named<'a>(state: &'a State, name: &str) -> &'a Flight {
    state.flights.iter().find(|f| f.name == name).unwrap()
}

/// A state holding the flights `app`'s document stores (the app's own sync does this after every message).
fn state_for(app: &App) -> State {
    let mut state = State::new();
    state.sync(app.ctx(), true);
    state
}

#[test]
fn the_example_flies_the_kit_flights() {
    let app = App::with_example();
    let ctx = app.ctx();
    let mut state = state_for(&app);
    assert_eq!(state.flights.len(), 4);
    let (nominal, windy, bigger) = (named(&state, "Nominal").id, named(&state, "Windy").id, named(&state, "Bigger motor").id);
    for id in [nominal, windy, bigger] {
        fly(&mut state, ctx, id);
    }

    let run = named(&state, "Nominal").run.as_ref().unwrap();
    let s = &run.result.summary;
    // A 76 mm rocket on an H128W: a few hundred metres, over 100 m/s, landing within a field-sized distance.
    assert!((200.0..2000.0).contains(&s.apogee), "apogee {}", s.apogee);
    assert!(s.max_velocity > 80.0 && s.flight_time > s.time_to_apogee);
    let t = run.times;
    assert!(t.rail_exit < t.burnout && t.burnout < t.apogee && t.apogee < t.landing, "{t:?}");
    assert!(run.took > Duration::ZERO && run.dt == 0.001);

    // More wind, more drift; a bigger motor, a higher apogee.
    let drift = |name: &str| named(&state, name).run.as_ref().unwrap().result.summary.landing.0;
    assert!(drift("Windy") > drift("Nominal"), "{} vs {}", drift("Windy"), drift("Nominal"));
    let apogee = |name: &str| named(&state, name).run.as_ref().unwrap().result.summary.apogee;
    assert!(apogee("Bigger motor") > apogee("Nominal"));

    // The selected flight is on show alone: one track, the plots built for the unit set.
    assert_eq!(state.shown.ids, vec![nominal]);
    assert!(!state.shown.compare && state.shown.group.is_some());
    assert_eq!(state.shown.ground.tracks.len(), 1);
    assert_eq!(state.plot.full().0, 0.0);
}

#[test]
fn the_ticked_flights_are_shown_in_colour_and_none_ticked_shows_nothing() {
    let app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, windy, ..] = ids_of(&state);
    fly(&mut state, app.ctx(), nominal);
    fly(&mut state, app.ctx(), windy);
    // The first flight starts ticked and is drawn alone, in its slot colour.
    assert_eq!(state.shown.ids, vec![nominal]);
    let colour = |state: &State| state.shown.group.as_ref().unwrap().panels[0].series[0].color;
    assert_eq!(colour(&state), SeriesColor::Series(1));
    // Selecting another flight does not change what is drawn; unticking the last shows nothing.
    state.select_only(windy);
    state.rebuild();
    assert_eq!(state.shown.ids, vec![nominal]);
    state.flights.iter_mut().for_each(|f| f.slot = None);
    state.rebuild();
    assert!(state.shown.ids.is_empty() && state.shown.group.is_none());
}

#[test]
fn ticking_overlays_flights_in_stable_colours_and_refuses_a_fifth() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let ids: Vec<FlightId> = state.flights.iter().map(|f| f.id).collect();
    for id in &ids[..3] {
        fly(&mut state, app.ctx(), *id);
    }
    state.update(Message::AddFlight, app.ctx_mut());
    let fifth = state.selected;
    state.update(Message::Click(ids[0]), app.ctx_mut());

    // Nominal, the first flight, starts ticked; tick Windy and Bigger motor (the fourth default): colours follow the tick order
    // and stay.
    assert_eq!(state.ticked(), 1);
    for id in [ids[1], ids[3]] {
        state.update(Message::Tick(id), app.ctx_mut());
    }
    let slot = |state: &State, id| state.flight(id).unwrap().slot;
    assert_eq!((slot(&state, ids[0]), slot(&state, ids[1]), slot(&state, ids[3])), (Some(1), Some(2), Some(3)));
    assert!(state.shown.compare);
    // Bigger motor has not run, so only two flights are on show.
    assert_eq!(state.shown.ids, vec![ids[0], ids[1]]);

    // Unticking the first does not repaint the others; the freed colour is the next one handed out.
    state.update(Message::Tick(ids[0]), app.ctx_mut());
    assert_eq!((slot(&state, ids[1]), slot(&state, ids[3])), (Some(2), Some(3)));
    state.update(Message::Tick(ids[2]), app.ctx_mut());
    assert_eq!(slot(&state, ids[2]), Some(1));

    // A fifth tick is refused while four are on.
    state.update(Message::Tick(ids[0]), app.ctx_mut());
    assert_eq!(state.ticked(), 4);
    state.update(Message::Tick(fifth), app.ctx_mut());
    assert_eq!(slot(&state, fifth), None);
    assert!(app.ctx().notice.is_some());
}

#[test]
fn a_result_goes_stale_when_conditions_or_the_rocket_change() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = state.selected;
    fly(&mut state, app.ctx(), id);
    let revision = state.revision(app.ctx());
    assert!(!state.flight(id).unwrap().is_stale(revision));

    state.update(Message::Edit(Edit::Wind(5.0)), app.ctx_mut());
    assert!(state.flight(id).unwrap().is_stale(state.revision(app.ctx())));
    state.update(Message::Edit(Edit::Wind(2.2)), app.ctx_mut());
    assert!(!state.flight(id).unwrap().is_stale(state.revision(app.ctx())), "back to the conditions it flew");
    assert_eq!(state.revision(app.ctx()), revision, "editing a flight is not a change to the rocket");

    app.ctx_mut().doc.edit("Rename", |r| r.designer = "someone".to_string());
    assert!(state.flight(id).unwrap().is_stale(state.revision(app.ctx())));
}

#[test]
fn a_superseded_result_is_ignored_and_a_failed_flight_says_why() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = state.selected;
    let _ = state.launch(id, app.ctx());
    let old = state.flight(id).unwrap().ticket;
    let _ = state.launch(id, app.ctx());
    let spec = state.flight(id).unwrap().spec.clone();
    let (input, delay) = flight::prepare(app.ctx(), &spec).unwrap();
    let late =
        Done { flight: id, ticket: old, spec, revision: 0, delay, dt: input.dt, outcome: Ok((Arc::new(bowshock_sim::simulate(&input)), Duration::ZERO)) };
    state.finished(late);
    assert!(state.flight(id).unwrap().run.is_none() && state.flight(id).unwrap().running);

    // A rocket without a motor mount cannot fly the bigger motor: the flight reports it instead of running.
    app.ctx_mut().doc.new_document();
    let bigger = named(&state, "Bigger motor").id;
    assert_eq!(named(&state, "Bigger motor").spec.motor, MotorChoice::Motor(bowshock_model::MotorRef::new("I284W", "AeroTech")));
    let _ = state.launch(bigger, app.ctx());
    assert!(named(&state, "Bigger motor").error.is_some() && !named(&state, "Bigger motor").running);
}

#[test]
fn choosing_a_motor_keeps_a_valid_delay_and_follow_the_configuration_otherwise() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Edit(Edit::Delay(12.0)), app.ctx_mut());
    assert_eq!(state.selected_flight().spec.delay, Some(12.0));
    let j350 = MotorChoice::Motor(bowshock_model::MotorRef::new("I218R", "AeroTech"));
    state.update(Message::Edit(Edit::Motor(j350.clone())), app.ctx_mut());
    let delays = &app.ctx().physics.motors().find("I218R").unwrap().delays;
    let chosen = state.selected_flight().spec.delay.unwrap();
    assert!(delays.contains(&chosen), "{chosen} not in {delays:?}");
    state.update(Message::Edit(Edit::Motor(MotorChoice::Active)), app.ctx_mut());
    assert_eq!(state.selected_flight().spec.delay, None);
}

#[test]
fn adding_and_removing_flights() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Edit(Edit::Wind(4.0)), app.ctx_mut());
    state.update(Message::AddFlight, app.ctx_mut());
    assert_eq!(state.flights.len(), 5);
    assert_eq!(state.selected_flight().spec.wind_speed, 4.0, "a new flight starts from the selected one's conditions");
    state.update(Message::RemoveFlight, app.ctx_mut());
    assert_eq!(state.flights.len(), 4);
    assert_eq!(state.selected, state.flights[0].id);
    for _ in 0..3 {
        state.update(Message::RemoveFlight, app.ctx_mut());
    }
    state.update(Message::RemoveFlight, app.ctx_mut());
    assert_eq!(state.flights.len(), 1, "the last flight stays");
}

#[test]
fn per_configuration_makes_a_flight_for_each_and_ticks_them() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    // One configuration: nothing to compare.
    let _ = state.update(Message::PerConfiguration, app.ctx_mut());
    assert_eq!(state.flights.len(), 4);

    app.ctx_mut().doc.edit("Duplicate", |r| {
        let id = r.active_config_id();
        r.duplicate_configuration(id);
    });
    let _ = state.update(Message::PerConfiguration, app.ctx_mut());
    assert_eq!(state.flights.len(), 6);
    assert_eq!(state.ticked(), 2);
    assert_eq!(state.selected_flight().spec.motor, MotorChoice::Config(app.ctx().doc.rocket().configurations()[0].id));
}

#[test]
fn clicking_a_number_moves_the_crosshair() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Seek(4.2), app.ctx_mut());
    assert_eq!(state.plot.hover_x(), Some(4.2));
    state.update(Message::Ground(GroundMessage::Hover(None)), app.ctx_mut());
    assert_eq!(state.plot.hover_x(), None);
}

#[test]
fn a_dispersion_study_scatters_landings_around_the_run_and_hides_when_the_run_changes() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = state.selected;
    fly(&mut state, app.ctx(), id);

    let _task = state.launch_dispersion(id, app.ctx());
    let flight = state.flight(id).unwrap();
    assert!(flight.dispersing);
    let (ticket, spec) = (flight.dispersion_ticket, flight.spec.clone());
    let (input, _) = flight::prepare(app.ctx(), &spec).unwrap();
    let study = bowshock_sim::monte_carlo(&input, flight::DISPERSION_RUNS, &flight::tolerances(&spec));
    let landings: Vec<(f64, f64)> = study.runs.iter().map(|r| (r.landing[0], r.landing[1])).collect();
    state.dispersed(DispersionDone { flight: id, ticket, spec, revision: state.revision(app.ctx()), outcome: Ok(landings) });

    let flight = state.flight(id).unwrap();
    assert!(!flight.dispersing);
    let d = flight.dispersion_of_run().expect("the study matches the run");
    assert_eq!(d.landings.len(), flight::DISPERSION_RUNS);
    assert!(d.outside(0.0) == d.landings.len() && d.outside(1.0e9) == 0);
    // The track carries the scatter and its 95 % ellipse.
    let track = &state.shown.ground.tracks[0];
    assert_eq!(track.scatter.len(), flight::DISPERSION_RUNS);
    let ellipse = track.ellipse.expect("an ellipse around 100 landings");
    assert!(ellipse.semi_major >= ellipse.semi_minor && ellipse.semi_minor > 0.0);

    // A superseded study is ignored; a re-run with other conditions hides the old one.
    let late = DispersionDone { flight: id, ticket: ticket - 1, spec: Spec::default(), revision: 0, outcome: Ok(Vec::new()) };
    state.dispersed(late);
    assert_eq!(state.flight(id).unwrap().dispersion.as_ref().unwrap().landings.len(), flight::DISPERSION_RUNS);
    state.update(Message::Edit(Edit::Wind(6.0)), app.ctx_mut());
    fly(&mut state, app.ctx(), id);
    assert!(state.flight(id).unwrap().dispersion_of_run().is_none());
    assert!(state.shown.ground.tracks[0].scatter.is_empty());
}

#[test]
fn enter_zooms_to_the_band_from_anywhere_and_otherwise_does_nothing() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = state.selected;
    fly(&mut state, app.ctx(), id);
    let enter = keyboard::Key::Named(keyboard::key::Named::Enter);
    assert!(state.on_key(&enter, keyboard::Modifiers::empty(), app.ctx()).is_none(), "no band, nothing to zoom to");

    state.update(Message::Plot(PlotMessage::Band(Some((1.0, 5.0)))), app.ctx_mut());
    let message = state.on_key(&enter, keyboard::Modifiers::empty(), app.ctx()).expect("a band is showing");
    state.update(message, app.ctx_mut());
    assert_eq!(state.plot.window(), (1.0, 5.0));
    assert!(state.on_key(&enter, keyboard::Modifiers::empty(), app.ctx()).is_none(), "the band cleared when it zoomed");
}

#[test]
fn flights_are_written_to_the_document_and_follow_undo() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let (nominal, windy) = (named(&state, "Nominal").id, named(&state, "Windy").id);
    fly(&mut state, app.ctx(), nominal);
    assert!(!app.ctx().doc.is_dirty());

    // An edit is a document edit: dirty, stored, one undo step; the other flights' results stay current.
    state.update(Message::Click(windy), app.ctx_mut());
    state.update(Message::Edit(Edit::Wind(9.0)), app.ctx_mut());
    let stored = |app: &App| app.ctx().doc.rocket().flights().iter().find(|f| f.id == windy).unwrap().conditions.wind_speed;
    assert_eq!(stored(&app), 9.0);
    assert!(app.ctx().doc.is_dirty() && app.ctx().doc.can_undo());
    assert!(!state.flight(nominal).unwrap().is_stale(state.revision(app.ctx())), "another flight's result is not stale");

    // Scrubbing one field is one step.
    state.update(Message::Edit(Edit::Wind(9.5)), app.ctx_mut());
    state.update(Message::Edit(Edit::Wind(10.0)), app.ctx_mut());
    app.ctx_mut().doc.seal();
    assert_eq!(app.ctx_mut().doc.undo().as_deref(), Some("Edit flight"));
    state.sync(app.ctx(), false);
    assert_eq!(state.flight(windy).unwrap().spec.wind_speed, 7.0, "undo brings the stored flight back, and the screen follows");
    assert_eq!(stored(&app), 7.0);
    assert!(app.ctx_mut().doc.redo().is_some());
    state.sync(app.ctx(), false);
    assert_eq!(state.flight(windy).unwrap().spec.wind_speed, 10.0);

    // Adding and removing are steps too.
    state.update(Message::AddFlight, app.ctx_mut());
    assert_eq!(app.ctx().doc.rocket().flights().len(), 5);
    assert!(app.ctx_mut().doc.undo().is_some());
    state.sync(app.ctx(), false);
    assert_eq!(state.flights.len(), 4);
    assert!(state.flight(state.selected).is_some(), "a selection that went away moves to a flight that exists");
}

#[test]
fn flights_are_saved_with_the_file_and_loaded_back_without_results() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let hot = named(&state, "Hot day").id;
    fly(&mut state, app.ctx(), hot);
    state.update(Message::Click(hot), app.ctx_mut());
    state.update(Message::Edit(Edit::Temperature(300.0)), app.ctx_mut());
    state.update(Message::Edit(Edit::Motor(MotorChoice::Motor(bowshock_model::MotorRef::new("I218R", "AeroTech")))), app.ctx_mut());
    state.update(Message::Edit(Edit::Delay(9.0)), app.ctx_mut());
    state.update(Message::AddFlight, app.ctx_mut());
    state.update(Message::Edit(Edit::Turbulence(bowshock_sim::Turbulence::High)), app.ctx_mut());

    let path = std::env::temp_dir().join(format!("bowshock_flights_{}.bowshock", std::process::id()));
    app.ctx_mut().doc.save_as(&path).expect("saves");
    let bytes = std::fs::read(&path).unwrap();
    assert!(bytes.starts_with(bowshock_model::MAGIC));
    let saved = bowshock_model::Rocket::from_bytes(&bytes).unwrap();
    assert!(saved.flights().iter().any(|f| f.name == "Hot day"));

    let mut fresh = App::new();
    fresh.ctx_mut().doc.load(&path).expect("loads");
    let _ = std::fs::remove_file(&path);
    let mut loaded = State::new();
    loaded.sync(fresh.ctx(), true);
    let names: Vec<&str> = loaded.flights.iter().map(|f| f.name.as_str()).collect();
    assert_eq!(names, ["Nominal", "Windy", "Hot day", "Bigger motor", "Flight 5"]);
    let spec = &named(&loaded, "Hot day").spec;
    assert_eq!(spec.temperature, 300.0);
    assert_eq!(spec.delay, Some(9.0));
    assert_eq!(spec.motor, MotorChoice::Motor(bowshock_model::MotorRef::new("I218R", "AeroTech")));
    assert_eq!(named(&loaded, "Flight 5").spec.turbulence, bowshock_sim::Turbulence::High);
    assert!(loaded.flights.iter().all(|f| f.run.is_none()), "results are not saved: run again");
    assert_eq!(loaded.selected, loaded.flights[0].id);
}

#[test]
fn a_new_document_starts_with_one_nominal_flight() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    assert_eq!(state.flights.len(), 4);
    app.ctx_mut().doc.new_document();
    state.sync(app.ctx(), true);
    assert_eq!(state.flights.len(), 1);
    assert_eq!(state.selected_flight().name, "Nominal");
    assert_eq!(state.selected_flight().spec, Spec::default());
}

#[test]
fn renaming_a_flight_goes_through_the_document_and_escape_cancels() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = named(&state, "Hot day").id;
    let _ = state.update(Message::StartRename(id), app.ctx_mut());
    state.update(Message::RenameInput("Cold day".into()), app.ctx_mut());
    state.update(Message::CancelRename, app.ctx_mut());
    assert_eq!(state.flight(id).unwrap().name, "Hot day");
    assert!(state.renaming.is_none());
    let _ = state.update(Message::StartRename(id), app.ctx_mut());
    state.update(Message::RenameInput("Cold day".into()), app.ctx_mut());
    state.update(Message::CommitRename, app.ctx_mut());
    assert_eq!(state.flight(id).unwrap().name, "Cold day");
    assert!(app.ctx().doc.rocket().flights().iter().any(|f| f.name == "Cold day"));
    assert_eq!(app.ctx_mut().doc.undo().as_deref(), Some("Rename flight"));
    state.sync(app.ctx(), false);
    assert_eq!(state.flight(id).unwrap().name, "Hot day");
}

/// The source keys of every panel of `state`'s layout.
fn sources(state: &State) -> Vec<Vec<&str>> {
    state.layout.panels.iter().map(|p| p.sources.iter().map(String::as_str).collect()).collect()
}

/// A click on the chrome of panel `panel`, as the plot canvas reports it.
fn click(state: &mut State, app: &mut App, panel: usize, action: PanelAction) {
    let chip = Rectangle::new(Point::new(100.0, 20.0), iced::Size::new(80.0, 18.0));
    let _ = state.update(Message::Plot(PlotMessage::Panel(panel, action, chip)), app.ctx_mut());
}

#[test]
fn a_new_screen_plots_the_flight_profile() {
    let app = App::with_example();
    let state = state_for(&app);
    assert_eq!(sources(&state), [["altitude"], ["vertical_velocity"], ["vertical_acceleration"]]);
    assert_eq!(Preset::of(&state.layout), Preset::FlightProfile);
    assert!(state.picker.is_none());
}

#[test]
fn choosing_a_source_in_the_picker_replaces_the_panels_source_as_one_undo_step() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let nominal = named(&state, "Nominal").id;
    fly(&mut state, app.ctx(), nominal);
    assert!(!app.ctx().doc.is_dirty());

    // The title opens the picker under its chip, in "primary" mode, with an empty search.
    click(&mut state, &mut app, 1, PanelAction::Pick);
    let picker = state.picker.as_ref().unwrap();
    assert_eq!((picker.panel, picker.mode, picker.query.as_str()), (1, Mode::Primary, ""));
    assert_eq!(picker.at, Point::new(100.0, 44.0));
    state.update(Message::PickerQuery("mach".into()), app.ctx_mut());
    assert_eq!(state.picker.as_ref().unwrap().query, "mach");

    state.update(Message::Layout(LayoutEdit::SetPrimary(1, "mach")), app.ctx_mut());
    assert!(state.picker.is_none(), "a choice closes the menu");
    assert_eq!(sources(&state), [vec!["altitude"], vec!["mach"], vec!["vertical_acceleration"]]);
    // The plots follow: the second panel is the Mach number of the run, and the document holds the layout.
    let group = state.shown.group.as_ref().unwrap();
    assert_eq!((group.panels[1].title.as_str(), group.panels[1].unit.as_str()), ("Mach number", ""));
    let max = group.panels[1].series[0].points.iter().map(|p| p.1).fold(0.0, f64::max);
    assert!((max - named(&state, "Nominal").run.as_ref().unwrap().result.summary.max_mach).abs() < 0.02, "{max}");
    assert_eq!(app.ctx().doc.rocket().plot_layout(), &state.layout);
    assert!(app.ctx().doc.is_dirty());
    // Changing the layout is not a change of the rocket: the run is still current.
    assert!(!state.flight(nominal).unwrap().is_stale(state.revision(app.ctx())));

    assert_eq!(app.ctx_mut().doc.undo().as_deref(), Some("Change plot source"));
    state.sync(app.ctx(), false);
    assert_eq!(sources(&state)[1], ["vertical_velocity"]);
    assert!(app.ctx_mut().doc.redo().is_some());
    state.sync(app.ctx(), false);
    assert_eq!(sources(&state)[1], ["mach"]);
}

#[test]
fn a_series_joins_only_with_the_same_unit_and_a_refusal_says_why() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let nominal = named(&state, "Nominal").id;
    fly(&mut state, app.ctx(), nominal);
    click(&mut state, &mut app, 0, PanelAction::AddSeries);
    assert_eq!(state.picker.as_ref().unwrap().mode, Mode::Add);

    // Ground distance is a length like altitude: it overlays.
    state.update(Message::Layout(LayoutEdit::ToggleSource(0, "ground_distance")), app.ctx_mut());
    assert_eq!(sources(&state)[0], ["altitude", "ground_distance"]);
    let group = state.shown.group.as_ref().unwrap();
    assert_eq!(group.panels[0].title, "Distance");
    // A speed does not: nothing changes, nothing is written, and the status strip says why.
    let revision = app.ctx().doc.revision();
    state.update(Message::Layout(LayoutEdit::ToggleSource(0, "vertical_velocity")), app.ctx_mut());
    assert_eq!(sources(&state)[0], ["altitude", "ground_distance"]);
    assert_eq!(app.ctx().doc.revision(), revision);
    let notice = app.ctx().notice.as_ref().map(|(text, _)| text.clone()).unwrap_or_default();
    assert!(notice.contains("Vertical velocity") && notice.contains("one y-axis"), "{notice}");
    // Toggling a source that is on the plot takes it off again.
    state.update(Message::Layout(LayoutEdit::ToggleSource(0, "ground_distance")), app.ctx_mut());
    assert_eq!(sources(&state)[0], ["altitude"]);
}

#[test]
fn panels_are_added_moved_and_removed_through_the_document() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Layout(LayoutEdit::AddPanel), app.ctx_mut());
    assert_eq!(sources(&state).len(), 4);
    assert_eq!(app.ctx().doc.rocket().plot_layout().panels.len(), 4);

    // The chrome: the arrows move a panel, the cross removes it.
    click(&mut state, &mut app, 3, PanelAction::MoveUp);
    assert_eq!(sources(&state)[2], ["mach"]);
    click(&mut state, &mut app, 2, PanelAction::MoveDown);
    assert_eq!(sources(&state)[3], ["mach"]);
    click(&mut state, &mut app, 3, PanelAction::Remove);
    assert_eq!(sources(&state).len(), 3);
    assert_eq!(Preset::of(&state.layout), Preset::FlightProfile);
    assert!(state.picker.is_none());

    // The ends refuse to move, and the last panel stays.
    let revision = app.ctx().doc.revision();
    click(&mut state, &mut app, 0, PanelAction::MoveUp);
    assert_eq!(app.ctx().doc.revision(), revision);
    state.update(Message::Layout(LayoutEdit::Preset(Preset::Custom)), app.ctx_mut());
    assert_eq!(sources(&state), [["altitude"]]);
    click(&mut state, &mut app, 0, PanelAction::Remove);
    assert_eq!(sources(&state).len(), 1);
    assert!(app.ctx().notice.as_ref().is_some_and(|(text, _)| text.contains("last plot")));
}

#[test]
fn presets_set_the_layout_and_the_row_names_the_one_that_matches() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    for preset in [Preset::Aerodynamics, Preset::Forces, Preset::Stability, Preset::FlightProfile] {
        state.update(Message::Layout(LayoutEdit::Preset(preset)), app.ctx_mut());
        assert_eq!(state.layout, preset.layout());
        assert_eq!(Preset::of(&state.layout), preset);
    }
    // Editing a preset makes it Custom; choosing Custom while already custom changes nothing.
    state.update(Message::Layout(LayoutEdit::SetPrimary(0, "mach")), app.ctx_mut());
    assert_eq!(Preset::of(&state.layout), Preset::Custom);
    let revision = app.ctx().doc.revision();
    state.update(Message::Layout(LayoutEdit::Preset(Preset::Custom)), app.ctx_mut());
    assert_eq!(app.ctx().doc.revision(), revision);
}

#[test]
fn the_x_axis_is_one_undo_step_kept_by_presets_and_follows_the_units() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = named(&state, "Nominal").id;
    fly(&mut state, app.ctx(), id);
    state.update(Message::Layout(LayoutEdit::X(PlotX::Altitude)), app.ctx_mut());
    assert_eq!(app.ctx().doc.rocket().plot_layout().x, PlotX::Altitude);
    let apogee = named(&state, "Nominal").run.as_ref().unwrap().result.summary.apogee;
    assert!((state.plot.full().1 - apogee).abs() < 1.0, "{:?} vs {apogee}", state.plot.full());
    // A preset changes the panels, not what they are plotted against.
    state.update(Message::Layout(LayoutEdit::Preset(Preset::Forces)), app.ctx_mut());
    assert_eq!((state.layout.x, Preset::of(&state.layout)), (PlotX::Altitude, Preset::Forces));
    // In feet the zoom's full range is in feet too.
    app.ctx_mut().units = UnitSystem::FT_LB_S;
    state.sync(app.ctx(), false);
    assert!((state.plot.full().1 - apogee / 0.3048).abs() < 3.0, "{:?}", state.plot.full());
    let _ = app.update(crate::app::Message::Undo);
    let _ = app.update(crate::app::Message::Undo);
    assert_eq!(app.ctx().doc.rocket().plot_layout().x, PlotX::Time);
}

#[test]
fn the_layout_is_saved_with_the_file_and_loaded_back() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Layout(LayoutEdit::Preset(Preset::Stability)), app.ctx_mut());
    state.update(Message::Layout(LayoutEdit::ToggleSource(1, "stability_margin")), app.ctx_mut());
    state.update(Message::Layout(LayoutEdit::AddPanel), app.ctx_mut());
    let layout = state.layout.clone();

    let path = std::env::temp_dir().join(format!("bowshock_plots_{}.bowshock", std::process::id()));
    app.ctx_mut().doc.save_as(&path).expect("saves");
    let bytes = std::fs::read(&path).unwrap();
    assert!(bytes.starts_with(bowshock_model::MAGIC));
    let saved = bowshock_model::Rocket::from_bytes(&bytes).unwrap();
    assert!(saved.plot_layout().panels.iter().any(|p| p.sources.iter().any(|s| s == "cp_station")));

    let mut fresh = App::new();
    fresh.ctx_mut().doc.load(&path).expect("loads");
    let _ = std::fs::remove_file(&path);
    let mut loaded = State::new();
    loaded.sync(fresh.ctx(), true);
    assert_eq!(loaded.layout, layout);
    assert_eq!(sources(&loaded)[0], ["cg_station", "cp_station"]);

    // A new document goes back to the flight profile.
    app.ctx_mut().doc.new_document();
    state.sync(app.ctx(), true);
    assert_eq!(state.layout, PlotLayout::default());
}

#[test]
fn a_layout_from_a_newer_version_draws_what_it_knows() {
    let mut app = App::with_example();
    app.ctx_mut().doc.edit("Future plots", |r| {
        r.set_plot_layout(PlotLayout {
            panels: vec![
                bowshock_model::PlotPanel::new(&["future_channel", "mach"]),
                bowshock_model::PlotPanel::new(&["only_the_future"]),
                bowshock_model::PlotPanel::new(&["thrust", "mach"]),
            ],
            ..PlotLayout::default()
        })
    });
    let state = state_for(&app);
    assert_eq!(sources(&state), [vec!["mach"], vec!["thrust"]]);
}

#[test]
fn undoing_closes_the_picker_and_a_flight_can_be_overlaid_with_several_sources() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let (nominal, windy) = (named(&state, "Nominal").id, named(&state, "Windy").id);
    fly(&mut state, app.ctx(), nominal);
    fly(&mut state, app.ctx(), windy);
    // Nominal starts ticked; tick Windy too.
    state.update(Message::Tick(windy), app.ctx_mut());
    state.update(Message::Layout(LayoutEdit::ToggleSource(1, "air_speed")), app.ctx_mut());
    assert_eq!(sources(&state)[1], ["vertical_velocity", "air_speed"]);
    // Two flights and two sources: four lines, colour by flight, style by source.
    let group = state.shown.group.as_ref().unwrap();
    assert_eq!(group.panels[1].series.len(), 4);
    let styles: Vec<_> = group.panels[1].series.iter().map(|s| (s.color, s.style)).collect();
    assert_eq!(styles[0].0, styles[1].0);
    assert_ne!(styles[0].0, styles[2].0);
    assert_ne!(styles[0].1, styles[1].1);
    assert_eq!(styles[0].1, styles[2].1);
    // Hover, zoom and the events of the reference are untouched by the layout.
    assert!(!group.events.is_empty() && group.right_margin == 0.0);

    click(&mut state, &mut app, 1, PanelAction::AddSeries);
    assert!(state.picker.is_some());
    assert!(app.ctx_mut().doc.undo().is_some());
    state.sync(app.ctx(), false);
    assert!(state.picker.is_none());
    assert_eq!(sources(&state)[1], ["vertical_velocity"]);
    // Esc closes it too.
    click(&mut state, &mut app, 0, PanelAction::Pick);
    assert!(state.dismiss_picker() && state.picker.is_none() && !state.dismiss_picker());
}

/// The ids of the example flights, in list order: Nominal, Windy, Hot day, Bigger motor.
fn ids_of(state: &State) -> [FlightId; 4] {
    let ids: Vec<FlightId> = state.flights.iter().map(|f| f.id).collect();
    ids.try_into().expect("the example stores four flights")
}

/// A click on flight `id` with `modifiers` held.
fn click_flight(state: &mut State, app: &mut App, id: FlightId, modifiers: keyboard::Modifiers) {
    state.update(Message::Modifiers(modifiers), app.ctx_mut());
    state.update(Message::Click(id), app.ctx_mut());
    state.update(Message::Modifiers(keyboard::Modifiers::empty()), app.ctx_mut());
}

#[test]
fn clicks_select_toggle_and_extend_a_range_while_ticks_stay_apart() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, windy, hot, bigger] = ids_of(&state);
    let (none, command, shift) = (keyboard::Modifiers::empty(), keyboard::Modifiers::COMMAND, keyboard::Modifiers::SHIFT);

    // A plain click selects one flight.
    click_flight(&mut state, &mut app, windy, none);
    assert_eq!((state.selected, state.edit_ids()), (windy, vec![windy]));

    // Cmd/Ctrl-click adds a flight (it becomes the selected one) and takes it out again.
    click_flight(&mut state, &mut app, bigger, command);
    assert_eq!((state.selected, state.edit_ids()), (bigger, vec![windy, bigger]));
    click_flight(&mut state, &mut app, bigger, command);
    assert_eq!((state.selected, state.edit_ids()), (windy, vec![windy]), "the oldest remaining flight takes over");
    click_flight(&mut state, &mut app, windy, command);
    assert_eq!(state.edit_ids(), vec![windy], "a toggle never empties the selection");

    // Shift-click selects the range from the last click, in list order, whichever way it runs.
    click_flight(&mut state, &mut app, bigger, shift);
    assert_eq!((state.selected, state.edit_ids()), (bigger, vec![windy, hot, bigger]));
    click_flight(&mut state, &mut app, nominal, shift);
    assert_eq!(state.edit_ids(), vec![nominal, windy], "a new range replaces the old one, from the same anchor");
    // With Cmd/Ctrl too the range is added to what is selected.
    click_flight(&mut state, &mut app, nominal, none);
    click_flight(&mut state, &mut app, hot, command);
    click_flight(&mut state, &mut app, bigger, command | shift);
    assert_eq!(state.edit_ids(), vec![nominal, hot, bigger]);
    click_flight(&mut state, &mut app, windy, none);
    assert_eq!(state.edit_ids(), vec![windy], "a plain click collapses the selection");

    // Cmd/Ctrl+A selects every flight; the key reaches the screen only when no widget took it.
    let a = keyboard::Key::Character("a".into());
    assert!(matches!(state.on_key(&a, command, app.ctx()), Some(Message::SelectAll)));
    assert!(state.on_key(&a, none, app.ctx()).is_none() && state.on_key(&a, command | shift, app.ctx()).is_none());
    state.update(Message::SelectAll, app.ctx_mut());
    assert_eq!((state.selected, state.edit_ids()), (windy, vec![nominal, windy, hot, bigger]));

    // The compare ticks are independent of the edit selection, both ways (the first flight starts ticked).
    assert_eq!(state.edit_ids().len(), 4);
    click_flight(&mut state, &mut app, hot, none);
    assert_eq!((state.ticked(), state.flight(nominal).unwrap().slot), (1, Some(1)), "selecting does not tick");

    // A selection survives edits to the list that keep its flights and drops the ones that went.
    click_flight(&mut state, &mut app, windy, command);
    state.update(Message::RemoveFlight, app.ctx_mut());
    assert_eq!(state.flights.len(), 2);
    assert!(state.flight(state.selected).is_some() && state.also.is_empty());
}

#[test]
fn conditions_the_flights_share_show_their_value_and_the_others_are_mixed() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, _, hot, _] = ids_of(&state);
    let shared = |state: &State, app: &App| flight::Shared::of(app.ctx(), &state.edit_flights());

    // One flight: everything is its own.
    let one = shared(&state, &app);
    assert_eq!((one.wind_speed, one.temperature, one.motor.clone()), (Some(2.2), Some(288.15), Some(MotorChoice::Active)));

    // Nominal and Hot day differ in the temperature alone.
    click_flight(&mut state, &mut app, nominal, keyboard::Modifiers::empty());
    click_flight(&mut state, &mut app, hot, keyboard::Modifiers::COMMAND);
    let pair = shared(&state, &app);
    assert_eq!((pair.temperature, pair.wind_speed, pair.rail_length), (None, Some(2.2), Some(Spec::default().rail_length)));

    // All four: wind (Windy), temperature (Hot day) and motor (Bigger motor) are mixed, the rest agree.
    state.update(Message::SelectAll, app.ctx_mut());
    let all = shared(&state, &app);
    assert_eq!((all.wind_speed, all.temperature, all.motor), (None, None, None));
    let spec = Spec::default();
    assert_eq!(
        (all.rail_length, all.launch_angle, all.wind_from, all.turbulence, all.site_altitude, all.field_radius),
        (Some(spec.rail_length), Some(spec.launch_angle), Some(spec.wind_from), Some(spec.turbulence), Some(spec.site_altitude), Some(spec.field_radius))
    );
    // The delay is what each flight flies: Bigger motor's own differs from the configuration's.
    let delays: Vec<f64> = state.flights.iter().map(|f| flight::effective_delay(app.ctx(), &f.spec)).collect();
    assert_eq!(all.delay.is_some(), delays.windows(2).all(|w| w[0] == w[1]));

    // Setting the mixed field on every flight makes it shared.
    state.update(Message::Edit(Edit::Wind(4.0)), app.ctx_mut());
    assert_eq!(shared(&state, &app).wind_speed, Some(4.0));
    assert_eq!(shared(&state, &app).temperature, None, "a field that was not touched stays per-flight");
}

#[test]
fn editing_with_several_selected_sets_the_field_on_each_in_one_undo_step() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, windy, hot, bigger] = ids_of(&state);
    click_flight(&mut state, &mut app, nominal, keyboard::Modifiers::empty());
    click_flight(&mut state, &mut app, hot, keyboard::Modifiers::SHIFT);
    assert_eq!(state.edit_ids(), vec![nominal, windy, hot]);
    let rail = |state: &State, id| state.flight(id).unwrap().spec.rail_length;
    let before = rail(&state, nominal);

    // Scrubbing the field is one step, however many ticks it sends, and the label says what and how many.
    for v in [1.5, 2.0, 2.4] {
        state.update(Message::Edit(Edit::RailLength(v)), app.ctx_mut());
    }
    app.ctx_mut().doc.seal();
    for id in [nominal, windy, hot] {
        assert_eq!(rail(&state, id), 2.4);
        assert_eq!(app.ctx().doc.rocket().flights().iter().find(|f| f.id == id).unwrap().conditions.rail_length, 2.4, "stored too");
    }
    assert_eq!(rail(&state, bigger), before, "a flight outside the selection is untouched");
    // The other fields stay per-flight.
    assert_eq!(state.flight(windy).unwrap().spec.wind_speed, 7.0);
    assert!((state.flight(hot).unwrap().spec.temperature - 308.15).abs() < 1e-9);

    assert_eq!(app.ctx().doc.undo_label(), Some("Set rail length on 3 flights"));
    assert_eq!(app.ctx_mut().doc.undo().as_deref(), Some("Set rail length on 3 flights"));
    state.sync(app.ctx(), false);
    assert!([nominal, windy, hot, bigger].iter().all(|id| rail(&state, *id) == before), "one undo puts all three back");
    assert!(app.ctx_mut().doc.redo().is_some());
    state.sync(app.ctx(), false);
    assert_eq!(rail(&state, windy), 2.4);

    // The motor picker sets the same motor on every selected flight (and the delay each keeps is one the motor offers).
    state.update(Message::Edit(Edit::Delay(12.0)), app.ctx_mut());
    let choice = MotorChoice::Motor(bowshock_model::MotorRef::new("I218R", "AeroTech"));
    state.update(Message::Edit(Edit::Motor(choice.clone())), app.ctx_mut());
    let delays = &app.ctx().physics.motors().find("I218R").unwrap().delays;
    for id in [nominal, windy, hot] {
        let spec = &state.flight(id).unwrap().spec;
        assert_eq!(spec.motor, choice);
        assert!(delays.contains(&spec.delay.unwrap()));
    }
    assert_eq!(state.flight(bigger).unwrap().spec.motor, MotorChoice::Motor(bowshock_model::MotorRef::new("I284W", "AeroTech")));
    assert_eq!(app.ctx().doc.undo_label(), Some("Set motor on 3 flights"));

    // One flight keeps the single-flight label.
    click_flight(&mut state, &mut app, bigger, keyboard::Modifiers::empty());
    state.update(Message::Edit(Edit::Wind(1.0)), app.ctx_mut());
    assert_eq!(app.ctx().doc.undo_label(), Some("Edit flight"));
}

#[test]
fn a_launch_site_is_one_step_and_the_location_and_wind_profile_fly() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, ..] = ids_of(&state);
    click_flight(&mut state, &mut app, nominal, keyboard::Modifiers::empty());
    let spec = |state: &State| state.flight(nominal).unwrap().spec.clone();
    let far = sites::SITES[0];
    state.update(Message::Edit(Edit::Site(0)), app.ctx_mut());
    app.ctx_mut().doc.seal();
    let s = spec(&state);
    assert_eq!(s.site_altitude, far.altitude);
    assert_eq!(sites::matching(s.location, s.site_altitude), Some(&sites::SITES[0]));
    assert!((s.temperature - sites::isa_temperature(far.altitude)).abs() < 1e-9, "the standard day at the site");
    assert_eq!(app.ctx().doc.undo_label(), Some("Set launch site"));
    assert_eq!(app.ctx_mut().doc.undo().as_deref(), Some("Set launch site"));
    state.sync(app.ctx(), false);
    assert_eq!(spec(&state).location, None, "one undo takes the altitude, location and temperature back");

    // Typing a latitude makes a location (longitude 0); a blank field clears it.
    state.update(Message::CoordInput(false, "45.5".into()), app.ctx_mut());
    state.update(Message::CoordCommit, app.ctx_mut());
    assert_eq!(spec(&state).location, Some(bowshock_model::Location { latitude: 45.5, longitude: 0.0 }));
    assert_eq!(app.ctx().doc.undo_label(), Some("Set launch location"));
    let launch = flight::launch_of(&spec(&state));
    assert!((launch.latitude.unwrap() - 45.5f64.to_radians()).abs() < 1e-12);
    state.update(Message::CoordInput(false, " ".into()), app.ctx_mut());
    state.update(Message::CoordCommit, app.ctx_mut());
    assert_eq!(spec(&state).location, None);

    // A wind profile is stored sorted and reaches the flight.
    let level = |altitude: f64, speed: f64| bowshock_model::WindLevel { altitude, speed, from: 0.0 };
    state.update(Message::Edit(Edit::WindLevels(vec![level(1000.0, 20.0), level(300.0, 8.0)])), app.ctx_mut());
    assert_eq!(app.ctx().doc.undo_label(), Some("Set wind profile"));
    let stored = &app.ctx().doc.rocket().flights().iter().find(|f| f.id == nominal).unwrap().conditions.wind_levels;
    assert_eq!(stored.iter().map(|l| l.altitude).collect::<Vec<_>>(), [300.0, 1000.0]);
    assert_eq!(flight::launch_of(&spec(&state)).wind_levels.len(), 2);
}

#[test]
fn a_forecast_sets_the_wind_and_temperature_in_one_step() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, ..] = ids_of(&state);
    click_flight(&mut state, &mut app, nominal, keyboard::Modifiers::empty());
    state.update(Message::ForecastOpen(true), app.ctx_mut());
    // No location: the fetch says what is missing instead of asking.
    state.update(Message::ForecastFetch, app.ctx_mut());
    assert_eq!(state.forecast.as_ref().unwrap().error.as_deref(), Some("Set the launch latitude and longitude first"));
    let f = forecast::parse(include_str!("../../tests/data/openmeteo.json"), 622.0).unwrap();
    state.update(Message::ForecastDone(Ok(f.clone())), app.ctx_mut());
    assert_eq!(app.ctx().doc.undo_label(), Some("Wind from forecast"));
    let spec = &state.flight(nominal).unwrap().spec;
    assert_eq!((spec.wind_speed, spec.wind_levels.len()), (f.ground.speed, f.levels.len()));
    assert!((spec.temperature - f.temperature_k.unwrap()).abs() < 1e-9);
    assert!(state.forecast.is_none(), "the row closes once applied");
}

#[test]
fn run_flies_the_selected_flights_and_removing_takes_them_all() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let [nominal, windy, hot, bigger] = ids_of(&state);
    click_flight(&mut state, &mut app, windy, keyboard::Modifiers::empty());
    click_flight(&mut state, &mut app, bigger, keyboard::Modifiers::COMMAND);
    state.update(Message::Run, app.ctx_mut());
    let running = |state: &State, id| state.flight(id).unwrap().running;
    assert_eq!([nominal, windy, hot, bigger].map(|id| running(&state, id)), [false, true, false, true]);

    // One selected flight runs alone, as before.
    click_flight(&mut state, &mut app, nominal, keyboard::Modifiers::empty());
    state.update(Message::Run, app.ctx_mut());
    assert!(running(&state, nominal) && !running(&state, hot));

    // Remove takes every selected flight in one step; it never takes the last ones.
    click_flight(&mut state, &mut app, windy, keyboard::Modifiers::empty());
    click_flight(&mut state, &mut app, hot, keyboard::Modifiers::COMMAND);
    state.update(Message::RemoveFlight, app.ctx_mut());
    assert_eq!(state.flights.iter().map(|f| f.id).collect::<Vec<_>>(), vec![nominal, bigger]);
    assert_eq!(app.ctx().doc.undo_label(), Some("Remove flights"));
    state.update(Message::SelectAll, app.ctx_mut());
    state.update(Message::RemoveFlight, app.ctx_mut());
    assert_eq!(state.flights.len(), 2, "removing every flight is refused");
}

/// A CFD table of the example rocket's whole stack with the given geometry hash.
fn cfd_table(geometry: u64) -> bowshock_model::CfdTable {
    use bowshock_model::{CfdGrid, CfdPoint, CfdQuality, CfdTable};
    let p = |mach: f64| CfdPoint { mach, cd: 0.5, friction: 0.1, pressure: 0.1, base: 0.1, fins: 0.1, other: 0.1, quality: CfdQuality::Converged };
    CfdTable {
        key: geometry ^ 7,
        geometry,
        stage: 1,
        solved_at: 0,
        solve_seconds: 1.0,
        backend: "test".into(),
        fallback: None,
        altitude: 0.0,
        notes: Vec::new(),
        points: vec![p(0.3), p(1.0), p(2.0)],
        grid: CfdGrid::Coarse,
        lift: None,
    }
}

fn current_geometry(app: &App) -> u64 {
    let rocket = app.ctx().doc.rocket();
    let v = bowshock_sim::from_model::vehicle(rocket, app.ctx().physics.motors(), bowshock_sim::from_model::Phase::Full).unwrap();
    crate::analyze::cfd::plan(rocket, &v, app.ctx().units).unwrap().geometry
}

#[test]
fn the_cfd_drag_source_attaches_a_matching_table_and_says_why_it_cannot() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    state.update(Message::Edit(Edit::Drag(bowshock_model::DragSource::CfdTable)), app.ctx_mut());
    let spec = state.selected_flight().spec.clone();

    let (input, _) = flight::prepare(app.ctx(), &spec).unwrap();
    assert!(input.drag_tables.iter().all(Option::is_none));
    assert!(flight::drag_note(app.ctx(), &spec).unwrap().contains("No CFD table"));

    let geometry = current_geometry(&app);
    assert!(app.ctx_mut().doc.store_cfd_table(cfd_table(geometry ^ 1)));
    let (input, _) = flight::prepare(app.ctx(), &spec).unwrap();
    assert!(input.drag_tables.iter().all(Option::is_none), "another geometry is not used");
    assert!(flight::drag_note(app.ctx(), &spec).unwrap().contains("stale"));

    assert!(app.ctx_mut().doc.store_cfd_table(cfd_table(geometry)));
    let (input, _) = flight::prepare(app.ctx(), &spec).unwrap();
    assert_eq!(input.drag_tables[0].as_ref().map(|t| t.points.len()), Some(3));
    assert_eq!(flight::drag_note(app.ctx(), &spec), None);

    let extended = Spec { drag: bowshock_model::DragSource::Extended, ..spec };
    assert!(flight::prepare(app.ctx(), &extended).unwrap().0.drag_tables.is_empty());
}

#[test]
fn changing_the_drag_source_or_the_stored_table_makes_a_result_stale() {
    let mut app = App::with_example();
    let mut state = state_for(&app);
    let id = state.selected;
    fly(&mut state, app.ctx(), id);
    assert!(!state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));
    state.update(Message::Edit(Edit::Drag(bowshock_model::DragSource::CfdTable)), app.ctx_mut());
    assert!(state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));
    state.update(Message::Edit(Edit::Drag(bowshock_model::DragSource::Extended)), app.ctx_mut());
    assert!(!state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));

    // Storing a table is not a document revision, but a flight on the CFD table goes stale (an Extended one does not).
    let geometry = current_geometry(&app);
    assert!(app.ctx_mut().doc.store_cfd_table(cfd_table(geometry)));
    assert!(!state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));
    state.update(Message::Edit(Edit::Drag(bowshock_model::DragSource::CfdTable)), app.ctx_mut());
    fly(&mut state, app.ctx(), id);
    assert!(!state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));
    assert!(app.ctx_mut().doc.store_cfd_table(cfd_table(geometry ^ 3)));
    assert!(state.flight(id).unwrap().is_stale(state.stamp(app.ctx(), &state.flight(id).unwrap().spec)));
}

/// The example's fin group and a flight of it run on a fresh state: the pieces the cant tests share.
fn example_with_fins() -> (App, bowshock_model::PartId) {
    let mut app = App::with_example();
    let fins = app.ctx().doc.rocket().radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
    app.ctx_mut().selection = crate::app::Selection::Part(fins);
    (app, fins)
}

/// The roll of the nominal flight as the summary reports it: `(max roll rate rad/s, summary)`, flown on the document as it stands.
fn nominal_spin(app: &App) -> (f64, bowshock_sim::Summary) {
    let mut state = state_for(app);
    let nominal = named(&state, "Nominal").id;
    fly(&mut state, app.ctx(), nominal);
    let summary = named(&state, "Nominal").run.as_ref().unwrap().result.summary.clone();
    (summary.max_roll_rate, summary)
}

#[test]
fn a_cant_typed_in_the_inspector_is_one_undo_step_and_the_next_flight_spins_clockwise_seen_from_behind() {
    use crate::app::Message as AppMessage;
    use crate::build::inspector::{Field, Message as Inspector, State as InspectorState};
    use bowshock_model::Dim;

    let (mut app, fins) = example_with_fins();
    let cant = |app: &App| app.ctx().doc.rocket().radial(fins).unwrap().cant;
    let mut state = state_for(&app);
    let nominal = named(&state, "Nominal").id;
    fly(&mut state, app.ctx(), nominal);
    assert_eq!(named(&state, "Nominal").run.as_ref().unwrap().result.summary.max_roll_rate, 0.0, "straight fins do not spin it");
    assert!(!named(&state, "Nominal").is_stale(state.stamp(app.ctx(), &named(&state, "Nominal").spec)));

    // Typing 3 and Enter is one commit, one undo step.
    let mut inspector = InspectorState::new();
    let _ = inspector.update(Inspector::Edit(Field::Dim(Dim::Cant), 3.0_f64.to_radians(), false), app.ctx_mut());
    app.sync();
    assert!((cant(&app) - 3.0_f64.to_radians()).abs() < 1e-12);

    // The result that was on show is out of date, and the next flight carries the cant into the simulation as a positive roll.
    assert!(named(&state, "Nominal").is_stale(state.stamp(app.ctx(), &named(&state, "Nominal").spec)));
    let spec = named(&state, "Nominal").spec.clone();
    let (input, _) = flight::prepare(app.ctx(), &spec).expect("flies");
    assert!(input.vehicle.fin_sets.iter().all(|f| (f.cant - 3.0_f64.to_radians()).abs() < 1e-12), "{:?}", input.vehicle.fin_sets);
    let (max, summary) = nominal_spin(&app);
    // A 3 inch rocket on an H128W with 3 degrees of cant turns at about 20 turns a second at burnout: over a thousand rpm,
    // clockwise (positive) seen from behind, and more than the pitch frequency: it passes through it spinning up.
    assert!(max > 0.0 && format::rpm(max) > 500.0 && format::rpm(max) < 3000.0, "{} rpm", format::rpm(max));
    assert!(summary.roll_rate_at_burnout > 0.9 * max && format::spin_sense(max) == "CW");
    assert!(summary.roll_resonance.is_some_and(|r| r.crossed), "{:?}", summary.roll_resonance);

    // One undo puts the fins straight, and the flight stops spinning with them.
    let _ = app.update(AppMessage::Undo);
    assert_eq!(cant(&app), 0.0);
    assert_eq!(nominal_spin(&app).0, 0.0);
    let _ = app.update(AppMessage::Redo);
    assert!((cant(&app) - 3.0_f64.to_radians()).abs() < 1e-12);

    // Scrubbing the field is live edits that merge, then the commit of the value the scrub ended on: still one step.
    let _ = app.update(AppMessage::Undo);
    assert_eq!(cant(&app), 0.0);
    for degrees in [1.0, 2.0, 4.0] {
        let _ = inspector.update(Inspector::Edit(Field::Dim(Dim::Cant), f64::to_radians(degrees), true), app.ctx_mut());
    }
    let _ = inspector.update(Inspector::Edit(Field::Dim(Dim::Cant), 4.0_f64.to_radians(), false), app.ctx_mut());
    app.sync();
    assert!((cant(&app) - 4.0_f64.to_radians()).abs() < 1e-12);
    let _ = app.update(AppMessage::Undo);
    assert_eq!(cant(&app), 0.0, "the whole scrub is one step");
}

#[test]
fn the_other_way_of_cant_spins_the_other_way_and_fin_b_alone_a_third_as_fast() {
    use crate::build::inspector::{Field, Message as Inspector, State as InspectorState};
    use bowshock_model::Dim;

    let (mut app, fins) = example_with_fins();
    let cant_field = Field::Dim(Dim::Cant);
    let mut inspector = InspectorState::new();
    let mut send = |app: &mut App, message: Inspector| {
        let _ = inspector.update(message, app.ctx_mut());
        app.sync();
    };
    send(&mut app, Inspector::Edit(cant_field, 2.0_f64.to_radians(), false));
    let all = nominal_spin(&app).0;
    send(&mut app, Inspector::Edit(cant_field, -2.0_f64.to_radians(), false));
    let back = nominal_spin(&app).0;
    assert!(all > 0.0 && (back + all).abs() < 1e-6 * all, "{all} {back}");

    // Straight fins again, then Unlink the Cant of fin B (the copy the header letter shows) and cant that fin alone: the three
    // fins share one geometry and one roll damping, so a third of the forcing makes a third of the spin.
    send(&mut app, Inspector::Edit(cant_field, 0.0, false));
    app.ctx_mut().symmetry.instance = Some((fins, 1));
    send(&mut app, Inspector::Unlink(cant_field));
    send(&mut app, Inspector::Edit(cant_field, 2.0_f64.to_radians(), false));
    let rocket = app.ctx().doc.rocket();
    let group = rocket.radial(fins).unwrap();
    assert_eq!((group.effective(0).cant, group.effective(1).cant, group.effective(2).cant), (0.0, 2.0_f64.to_radians(), 0.0));
    let one = nominal_spin(&app).0;
    assert!((one / all - 1.0 / 3.0).abs() < 0.01, "{one} vs {all}");
}
