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
    state.finished(Done { flight: id, ticket, spec, revision: state.revision(ctx), delay, dt: input.dt, outcome: Ok((result, started.elapsed())) });
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

    // The selected flight is on show alone: one track, the plots built for both unit families.
    assert_eq!(state.shown.ids, vec![nominal]);
    assert!(!state.shown.compare && state.shown.groups.iter().all(Option::is_some));
    assert_eq!(state.shown.ground.tracks.len(), 1);
    assert_eq!(state.plot.full().0, 0.0);
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
    state.update(Message::Select(ids[0]), app.ctx_mut());

    // Tick Nominal, Windy, Bigger motor (the fourth default): colours follow the tick order and stay.
    for id in [ids[0], ids[1], ids[3]] {
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
    state.update(Message::Select(windy), app.ctx_mut());
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
    state.update(Message::Select(hot), app.ctx_mut());
    state.update(Message::Edit(Edit::Temperature(300.0)), app.ctx_mut());
    state.update(Message::Edit(Edit::Motor(MotorChoice::Motor(bowshock_model::MotorRef::new("I218R", "AeroTech")))), app.ctx_mut());
    state.update(Message::Edit(Edit::Delay(9.0)), app.ctx_mut());
    state.update(Message::AddFlight, app.ctx_mut());
    state.update(Message::Edit(Edit::Turbulence(bowshock_sim::Turbulence::High)), app.ctx_mut());

    let path = std::env::temp_dir().join(format!("bowshock_flights_{}.bowshock", std::process::id()));
    app.ctx_mut().doc.save_as(&path).expect("saves");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"flights\"") && text.contains("\"Hot day\"") && !text.contains("\"result\""));

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
