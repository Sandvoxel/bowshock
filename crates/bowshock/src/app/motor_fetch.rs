//! Fetching the motors a freshly loaded document needs from ThrustCurve.org. After a document is opened, imported or started
//! from the example ([`super::files`] sets [`Ctx::fetch_wanted`]), [`start`] lists the motors it names that the motor database
//! cannot resolve ([`missing`]) and looks them up on one worker thread ([`fetch`]); [`apply`] adds what came back to the
//! database and the on-disk cache (the way the motor picker's download does, [`Physics::add_downloaded`]) and tells the user in
//! the status strip. A reference is only accepted when exactly one search result carries its name ([`pick`], by the rule
//! [`bowshock_motors::same_motor_name`] that [`bowshock_motors::MotorDb::resolve`] looks a saved reference up by); several or
//! none are reported, never guessed. The lookup runs in the windowed app, and in a headless run or a test only when
//! `BOWSHOCK_THRUSTCURVE_API` is set ([`Env::fetch_motors`](super::env::Env::fetch_motors)).

use std::time::Duration;

use bowshock_model::{FlightMotor, MotorRef, Rocket};
use bowshock_motors::thrustcurve::{Client, RemoteMotor, SearchQuery, TcError};
use bowshock_motors::{same_motor_name, Motor, MotorDb};
use bowshock_sim::from_model::find_motor;
use iced::Task;

use super::{worker, Ctx, Message};

/// The longest any one request of the lookup may take: short, so an offline machine finds out at once.
const TIMEOUT: Duration = Duration::from_secs(8);
/// What a search result must be to be accepted, in the words of the status strip.
const NOT_FOUND: &str = "not found on ThrustCurve.org";
/// The start of a network failure's text (see [`TcError`]'s `Display`).
const UNREACHABLE: &str = "ThrustCurve unreachable";

/// One motor's outcome: the reference that was missing and the motor found for it, or why not.
pub type Outcome = (MotorRef, Result<Motor, String>);

/// The distinct motors `rocket` names that `db` cannot resolve: every configuration's motors and every flight's explicit motor.
pub fn missing(rocket: &Rocket, db: &MotorDb) -> Vec<MotorRef> {
    let named =
        rocket.configurations().iter().flat_map(|c| c.motors.iter().filter_map(|slot| slot.motor.as_ref())).chain(rocket.flights().iter().filter_map(|f| {
            match &f.motor {
                FlightMotor::Motor(m) => Some(m),
                _ => None,
            }
        }));
    let mut out: Vec<MotorRef> = Vec::new();
    for reference in named {
        let same = |o: &MotorRef| o.id == reference.id && same_motor_name(&o.designation, &o.manufacturer, &reference.designation, &reference.manufacturer);
        if find_motor(db, reference).is_none() && !out.iter().any(same) {
            out.push(reference.clone());
        }
    }
    out
}

/// The one search result that is `reference`, or why none is: the results whose designation and manufacturer name it, else
/// (ThrustCurve.org often lists a motor with its delay, `K455NW-17`) the ones that do once delay suffixes are dropped.
pub fn pick(found: &[RemoteMotor], reference: &MotorRef) -> Result<RemoteMotor, String> {
    let named = |same: fn(&str, &str, &str, &str) -> bool| -> Vec<&RemoteMotor> {
        found.iter().filter(|m| same(&m.designation, &m.manufacturer, &reference.designation, &reference.manufacturer)).collect()
    };
    let exact = named(same_motor_name);
    let matches = if exact.is_empty() { named(bowshock_motors::same_motor_base_name) } else { exact };
    match matches.as_slice() {
        [] => Err(NOT_FOUND.to_string()),
        [only] => Ok((*only).clone()),
        more => Err(format!("{} matches on ThrustCurve.org, pick it in the motor picker", more.len())),
    }
}

/// Looks `missing` up with `client`: by ThrustCurve id when the reference has one, else by designation (the manufacturer is
/// checked on the results, [`pick`], so its spelling cannot make the API refuse the query); the accepted motors are downloaded in
/// one request. A network failure ends the lookup: the motors not yet tried get the same reason.
pub fn fetch(client: &Client, missing: &[MotorRef]) -> Vec<Outcome> {
    let mut slots: Vec<Option<Result<Motor, String>>> = vec![None; missing.len()];
    let mut accepted: Vec<(usize, RemoteMotor)> = Vec::new();
    let mut stopped: Option<String> = None;
    for (i, reference) in missing.iter().enumerate() {
        if let Some(why) = &stopped {
            slots[i] = Some(Err(why.clone()));
            continue;
        }
        let looked = match reference.id.strip_prefix(bowshock_motors::THRUSTCURVE_ID_PREFIX) {
            Some(motor_id) => client.download_id(motor_id).map(|motor| slots[i] = Some(Ok(motor))),
            None => search(client, reference).map(|found| match pick(&found, reference) {
                Ok(remote) => accepted.push((i, remote)),
                Err(why) => slots[i] = Some(Err(why)),
            }),
        };
        if let Err(e) = looked {
            let why = e.to_string();
            if matches!(e, TcError::Unreachable(_)) {
                stopped = Some(why.clone());
            }
            slots[i] = Some(Err(why));
        }
    }
    if !accepted.is_empty() && stopped.is_none() {
        let remote: Vec<RemoteMotor> = accepted.iter().map(|(_, r)| r.clone()).collect();
        match client.download(&remote) {
            Ok(motors) => {
                for (i, r) in &accepted {
                    slots[*i] = Some(match motors.iter().find(|m| m.id == r.id()) {
                        Some(m) => Ok(m.clone()),
                        None => Err(TcError::NoData(r.designation.clone()).to_string()),
                    });
                }
            }
            Err(e) => accepted.iter().for_each(|(i, _)| slots[*i] = Some(Err(e.to_string()))),
        }
    }
    missing.iter().cloned().zip(slots.into_iter().map(|s| s.unwrap_or_else(|| Err(NOT_FOUND.to_string())))).collect()
}

/// The results for `reference`'s designation (the common name's when the designation matched nothing).
fn search(client: &Client, reference: &MotorRef) -> Result<Vec<RemoteMotor>, TcError> {
    let query = SearchQuery {
        designation: Some(reference.designation.trim().to_ascii_uppercase()),
        availability: Some("all".into()),
        max_results: Some(200),
        ..Default::default()
    };
    let found = client.search(&query)?;
    match query.fallback() {
        Some(fallback) if found.is_empty() => client.search(&fallback),
        _ => Ok(found),
    }
}

/// After a load: when the document names motors the database lacks and fetching is on, shows the start notice and returns the
/// worker task that looks them up and answers [`Message::MotorsFetched`].
pub(super) fn start(ctx: &mut Ctx) -> Task<Message> {
    if !std::mem::take(&mut ctx.fetch_wanted) || !ctx.env.fetch_motors {
        return Task::none();
    }
    let wanted = missing(ctx.doc.rocket(), ctx.physics.motors());
    if wanted.is_empty() {
        return Task::none();
    }
    ctx.notify(format!("Fetching {} from ThrustCurve.org…", count(wanted.len())));
    let ticket = ctx.load_ticket;
    let base = ctx.env.thrustcurve_api.clone();
    let names = wanted.clone();
    worker::perform(
        move || {
            let client = Client::with_timeout(TIMEOUT);
            let client = match base {
                Some(base) => client.with_base(base),
                None => client,
            };
            fetch(&client, &wanted)
        },
        move |result| {
            let results = result.unwrap_or_else(|| names.into_iter().map(|m| (m, Err(format!("{UNREACHABLE} (the lookup stopped)")))).collect());
            Message::MotorsFetched(ticket, results)
        },
    )
}

/// Adds the motors the lookup of load `ticket` found to the database and the cache, and, when that document is still the open
/// one, says what happened. Motors of a document that has been replaced are still added (they are cached, which does no harm).
pub(super) fn apply(ctx: &mut Ctx, ticket: u64, results: Vec<Outcome>) {
    let mut cache_note = None;
    for (_, result) in &results {
        if let Ok(motor) = result {
            let (_, note) = ctx.physics.add_downloaded(motor.clone());
            cache_note = cache_note.or(note);
        }
    }
    if ticket == ctx.load_ticket {
        let mut notice = summary(&results);
        if let Some(note) = cache_note {
            notice = format!("{notice} · {note}");
        }
        ctx.notify(notice);
    }
}

/// `1 motor`, `2 motors`.
fn count(n: usize) -> String {
    if n == 1 {
        "1 motor".to_string()
    } else {
        format!("{n} motors")
    }
}

/// The status strip's account of a finished lookup.
fn summary(results: &[Outcome]) -> String {
    let fetched: Vec<&str> = results.iter().filter_map(|(_, r)| r.as_ref().ok()).map(|m| m.designation.as_str()).collect();
    let first_failure = results.iter().find_map(|(m, r)| r.as_ref().err().map(|why| (m, why)));
    match first_failure {
        None => format!("Fetched {} from ThrustCurve.org", designations(&fetched)),
        Some((_, why)) if fetched.is_empty() && why.starts_with(UNREACHABLE) => {
            let n = results.len();
            format!("Could not reach ThrustCurve.org; {n} {} missing", if n == 1 { "motor is" } else { "motors are" })
        }
        Some((motor, why)) => {
            format!("Fetched {} of {} motors; {}: {why}", fetched.len(), results.len(), motor.designation)
        }
    }
}

/// `A`, `A and B`, `A, B and C`, `A, B, C and 2 more`.
fn designations(names: &[&str]) -> String {
    match names {
        [] => String::new(),
        [one] => (*one).to_string(),
        [init @ .., last] if names.len() <= 3 => format!("{} and {last}", init.join(", ")),
        _ => format!("{} and {} more", names[..3].join(", "), names.len() - 3),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{InternalKind, NoseShape, PartKind};
    use bowshock_motors::thrustcurve::parse_search;

    fn rocket(refs: [(&str, &str); 3]) -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
        let mount = r
            .attach_internal(tube, 0.5, InternalKind::InnerTube { outer_diameter: 0.042, thickness: 0.002, length: 0.3, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let id = r.active_config_id();
        r.set_motor(id, mount, Some(MotorRef::new(refs[0].0, refs[0].1))).unwrap();
        let mut flights = r.flights().to_vec();
        flights[0].motor = FlightMotor::Motor(MotorRef::new(refs[1].0, refs[1].1));
        let mut second = flights[0].clone();
        second.id = flights.iter().map(|f| f.id).max().unwrap() + 1;
        second.motor = FlightMotor::Motor(MotorRef::new(refs[2].0, refs[2].1));
        flights.push(second);
        r.set_flights(flights);
        r
    }

    fn remote_json(rows: &[(&str, &str, &str)]) -> String {
        let results: Vec<String> = rows
            .iter()
            .map(|(id, maker, designation)| {
                format!(r#"{{"motorId":"{id}","manufacturer":"{maker}","manufacturerAbbrev":"{maker}","designation":"{designation}","commonName":"I284","impulseClass":"I","diameter":38,"length":300,"dataFiles":1}}"#)
            })
            .collect();
        format!(r#"{{"criteria":[],"results":[{}]}}"#, results.join(","))
    }

    fn motor(designation: &str, id: &str) -> Motor {
        let mut m = MotorDb::bundled().find("H128W").unwrap().clone();
        m.designation = designation.into();
        m.id = id.into();
        m
    }

    #[test]
    fn missing_lists_each_unknown_motor_once() {
        // `X999Q` and `Y888Q` are in no catalogue; `H128W` is in the bundled one.
        let db = MotorDb::bundled();
        let r = rocket([("H128W", "AeroTech"), ("X999Q", "AeroTech"), ("Y888Q", "AeroTech")]);
        assert_eq!(missing(&r, &db), vec![MotorRef::new("X999Q", "AeroTech"), MotorRef::new("Y888Q", "AeroTech")]);
        // The same unknown motor in a configuration and a flight is one entry.
        let r = rocket([("X999Q", "AeroTech"), ("x999q", "aerotech"), ("H128W", "AeroTech")]);
        assert_eq!(missing(&r, &db), vec![MotorRef::new("X999Q", "AeroTech")]);
        assert!(missing(&rocket([("H128W", "AeroTech"); 3]), &db).is_empty());
    }

    #[test]
    fn pick_accepts_exactly_one_match() {
        let reference = MotorRef::new("I284W", "AeroTech");
        let one = parse_search(&remote_json(&[("a1", "AeroTech", "I284W"), ("a2", "Cesaroni", "I284W"), ("a3", "AeroTech", "I285W")])).unwrap();
        assert_eq!(pick(&one, &reference).unwrap().motor_id, "a1");
        assert_eq!(pick(&one, &MotorRef::new("i284w", "AEROTECH")).unwrap().motor_id, "a1");
        let none = parse_search(&remote_json(&[("a2", "Cesaroni", "I284W")])).unwrap();
        assert_eq!(pick(&none, &reference), Err("not found on ThrustCurve.org".to_string()));
        assert_eq!(pick(&[], &reference), Err("not found on ThrustCurve.org".to_string()));
        let two = parse_search(&remote_json(&[("a1", "AeroTech", "I284W"), ("a4", "AeroTech", "I284W")])).unwrap();
        assert_eq!(pick(&two, &reference), Err("2 matches on ThrustCurve.org, pick it in the motor picker".to_string()));
        // ThrustCurve.org lists the AeroTech K455NW with its delay; another maker's K455 is not it, and an exact name wins.
        let delayed = parse_search(&remote_json(&[("k1", "AMW", "1483K455-P"), ("k2", "AeroTech", "K455NW-17")])).unwrap();
        assert_eq!(pick(&delayed, &MotorRef::new("K455NW", "AeroTech")).unwrap().motor_id, "k2");
        let both = parse_search(&remote_json(&[("k2", "AeroTech", "K455NW-17"), ("k3", "AeroTech", "K455NW")])).unwrap();
        assert_eq!(pick(&both, &MotorRef::new("K455NW", "AeroTech")).unwrap().motor_id, "k3");
    }

    fn loaded_ctx(refs: [(&str, &str); 3]) -> Ctx {
        let mut ctx = Ctx::new();
        ctx.doc.replace(rocket(refs), None);
        ctx.loaded();
        ctx.physics.sync(ctx.doc.rocket(), ctx.doc.revision(), ctx.units);
        ctx
    }

    fn notice(ctx: &Ctx) -> String {
        ctx.notice.as_ref().map(|n| n.0.clone()).unwrap_or_default()
    }

    #[test]
    fn a_fetched_motor_clears_the_unknown_motor_warning() {
        let mut ctx = loaded_ctx([("X999Q", "AeroTech"), ("H128W", "AeroTech"), ("H128W", "AeroTech")]);
        let unknown = |ctx: &Ctx| ctx.physics.warnings().iter().any(|w| w.message.contains("X999Q is not in the motor database"));
        assert!(unknown(&ctx));
        let ticket = ctx.load_ticket;
        apply(&mut ctx, ticket, vec![(MotorRef::new("X999Q", "AeroTech"), Ok(motor("X999Q", "tc:1")))]);
        ctx.physics.sync(ctx.doc.rocket(), ctx.doc.revision(), ctx.units);
        assert!(!unknown(&ctx));
        assert_eq!(notice(&ctx), "Fetched X999Q from ThrustCurve.org");
    }

    #[test]
    fn a_stale_result_adds_the_motor_and_stays_quiet() {
        let mut ctx = loaded_ctx([("X999Q", "AeroTech"), ("H128W", "AeroTech"), ("H128W", "AeroTech")]);
        let stale = ctx.load_ticket;
        ctx.loaded();
        ctx.notice = None;
        apply(&mut ctx, stale, vec![(MotorRef::new("X999Q", "AeroTech"), Ok(motor("X999Q", "tc:1")))]);
        assert!(ctx.physics.motors().find("X999Q").is_some());
        assert!(ctx.notice.is_none());
    }

    #[test]
    fn the_notices_use_the_agreed_words() {
        let r = |d: &str| MotorRef::new(d, "AeroTech");
        let ok = |d: &str| (r(d), Ok(motor(d, &format!("tc:{d}"))));
        let err = |d: &str, why: &str| (r(d), Err(why.to_string()));
        assert_eq!(summary(&[ok("I284W"), ok("J350W")]), "Fetched I284W and J350W from ThrustCurve.org");
        assert_eq!(summary(&[ok("A8"), ok("B6"), ok("C6")]), "Fetched A8, B6 and C6 from ThrustCurve.org");
        assert_eq!(summary(&[ok("A8"), ok("B6"), ok("C6"), ok("D12"), ok("E9")]), "Fetched A8, B6, C6 and 2 more from ThrustCurve.org");
        assert_eq!(summary(&[ok("I284W"), err("K550W", NOT_FOUND)]), "Fetched 1 of 2 motors; K550W: not found on ThrustCurve.org");
        let many = "2 matches on ThrustCurve.org, pick it in the motor picker";
        assert_eq!(summary(&[err("K550W", many), ok("I284W")]), format!("Fetched 1 of 2 motors; K550W: {many}"));
        let offline = TcError::Unreachable("dns".into()).to_string();
        assert_eq!(summary(&[err("I284W", &offline), err("K550W", &offline)]), "Could not reach ThrustCurve.org; 2 motors are missing");
        assert_eq!(summary(&[ok("I284W"), err("K550W", &offline)]), format!("Fetched 1 of 2 motors; K550W: {offline}"));
        assert_eq!(count(1), "1 motor");
        assert_eq!(count(2), "2 motors");
    }

    #[test]
    fn no_fetch_without_the_gate_or_a_load() {
        let mut ctx = loaded_ctx([("X999Q", "AeroTech"), ("Y888Q", "AeroTech"), ("H128W", "AeroTech")]);
        ctx.fetch_wanted = true;
        assert!(!ctx.env.fetch_motors);
        let _ = start(&mut ctx);
        assert!(!ctx.fetch_wanted && ctx.notice.is_none(), "a run without the gate never announces or starts a lookup");
    }
}
