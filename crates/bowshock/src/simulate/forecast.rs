//! The wind forecast of the Simulate wind profile: the hourly forecast of the Open-Meteo API (https://open-meteo.com, no API
//! key; data under CC BY 4.0, credited under the table) at the launch site, for one hour. The ground wind is the 10 m wind; the
//! levels are the winds on the standard pressure levels, at the geopotential height of each level above the site.

use bowshock_model::WindLevel;
use serde_json::Value;

/// The forecast endpoint.
const API: &str = "https://api.open-meteo.com/v1/forecast";
/// The pressure levels asked for (hPa), from the ground up to about 10 km.
const LEVELS_HPA: [u32; 14] = [1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200];
/// A level at most this far above the site (m) is left out: the 10 m wind is the ground's.
const MIN_LEVEL_AGL: f64 = 15.0;

/// A forecast wind profile and temperature for one hour.
#[derive(Debug, Clone, PartialEq)]
pub struct Forecast {
    /// The 10 m wind, as the wind at the ground (altitude 0).
    pub ground: WindLevel,
    /// The 2 m air temperature (K), when the forecast has one.
    pub temperature_k: Option<f64>,
    /// The pressure-level winds above the site, ascending.
    pub levels: Vec<WindLevel>,
}

/// The `hourly` variables of the request.
fn variables() -> String {
    let mut v = vec!["wind_speed_10m".to_string(), "wind_direction_10m".to_string(), "temperature_2m".to_string()];
    for p in LEVELS_HPA {
        v.extend([format!("wind_speed_{p}hPa"), format!("wind_direction_{p}hPa"), format!("geopotential_height_{p}hPa")]);
    }
    v.join(",")
}

/// Whether `hour` reads `YYYY-MM-DDTHH:00`.
fn is_hour(hour: &str) -> bool {
    let b = hour.as_bytes();
    b.len() == 16
        && b.iter().enumerate().all(|(i, c)| match i {
            4 | 7 => *c == b'-',
            10 => *c == b'T',
            13 => *c == b':',
            14 | 15 => *c == b'0',
            _ => c.is_ascii_digit(),
        })
}

/// Fetches the forecast at `lat`/`lon` (degrees) for the site `site_altitude` metres above sea level, for the hour `utc_hour`
/// (`YYYY-MM-DDTHH:00`, UTC). Blocks for up to 15 s: call it off the UI thread.
pub fn fetch(lat: f64, lon: f64, site_altitude: f64, utc_hour: &str) -> Result<Forecast, String> {
    let hour = utc_hour.trim();
    if !is_hour(hour) {
        return Err("Enter the launch time as YYYY-MM-DDTHH:00 (UTC)".to_string());
    }
    let url = format!(
        "{API}?latitude={lat:.5}&longitude={lon:.5}&elevation={site_altitude:.0}&timezone=GMT&wind_speed_unit=ms&start_hour={hour}&end_hour={hour}&hourly={}",
        variables()
    );
    let config = ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_global(Some(std::time::Duration::from_secs(15)))
        .user_agent(concat!("Bowshock/", env!("CARGO_PKG_VERSION"), " (rocket flight simulator)"))
        .build();
    let agent: ureq::Agent = config.into();
    let unavailable = |e: &dyn std::fmt::Display| format!("Forecast unavailable: {e}");
    let mut response = agent.get(&url).call().map_err(|e| unavailable(&e))?;
    let body = response.body_mut().read_to_string().map_err(|e| unavailable(&e))?;
    parse(&body, site_altitude)
}

/// Reads an Open-Meteo answer for one hour (see [`fetch`]) for a site `site_altitude` metres above sea level.
pub fn parse(json: &str, site_altitude: f64) -> Result<Forecast, String> {
    let v: Value = serde_json::from_str(json).map_err(|e| format!("Forecast unavailable: {e}"))?;
    if v.get("error").and_then(Value::as_bool) == Some(true) {
        let reason = v.get("reason").and_then(Value::as_str).unwrap_or("the service refused the request");
        return Err(format!("Forecast unavailable: {reason}"));
    }
    let hourly = v.get("hourly").ok_or("Forecast unavailable: the answer has no hourly data")?;
    let first = |name: &str| hourly.get(name).and_then(|a| a.get(0)).and_then(Value::as_f64);
    let level = |altitude: f64, speed: f64, from_deg: f64| WindLevel { altitude, speed, from: from_deg.to_radians().rem_euclid(std::f64::consts::TAU) };
    let (Some(speed), Some(from)) = (first("wind_speed_10m"), first("wind_direction_10m")) else {
        return Err("Forecast unavailable: no wind for that hour".to_string());
    };
    let mut levels: Vec<WindLevel> = LEVELS_HPA
        .iter()
        .filter_map(|p| {
            let (speed, from, height) =
                (first(&format!("wind_speed_{p}hPa"))?, first(&format!("wind_direction_{p}hPa"))?, first(&format!("geopotential_height_{p}hPa"))?);
            let agl = height - site_altitude;
            (agl > MIN_LEVEL_AGL).then(|| level(agl, speed, from))
        })
        .collect();
    levels.sort_by(|a, b| a.altitude.total_cmp(&b.altitude));
    Ok(Forecast { ground: level(0.0, speed, from), temperature_k: first("temperature_2m").map(|c| c + 273.15), levels })
}

/// The next whole hour from now, UTC, as `YYYY-MM-DDTHH:00`.
pub fn next_hour_utc() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    hour_after(secs)
}

/// The whole hour after `unix_secs`, UTC, as `YYYY-MM-DDTHH:00` (civil date from the day count, Hinnant's algorithm).
fn hour_after(unix_secs: u64) -> String {
    let hours = unix_secs / 3600 + 1;
    let (days, hour) = ((hours / 24) as i64, hours % 24);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}T{hour:02}:00")
}

#[cfg(test)]
mod tests {
    use super::*;

    const RECORDED: &str = include_str!("../../tests/data/openmeteo.json");

    #[test]
    fn a_recorded_answer_reads_as_a_sorted_profile_above_the_site() {
        let f = parse(RECORDED, 622.0).unwrap();
        assert_eq!(f.ground.altitude, 0.0);
        assert!(f.temperature_k.is_some_and(|t| (200.0..330.0).contains(&t)));
        assert!(!f.levels.is_empty());
        assert!(f.levels.windows(2).all(|w| w[0].altitude < w[1].altitude));
        // The 1000, 975 and 950 hPa surfaces lie below a 622 m site: left out.
        assert!(f.levels.iter().all(|l| l.altitude > MIN_LEVEL_AGL));
        assert!(f.levels.len() < LEVELS_HPA.len());
        assert!(f.levels.iter().all(|l| (0.0..std::f64::consts::TAU).contains(&l.from) && l.speed >= 0.0));
    }

    #[test]
    fn an_error_answer_and_a_bad_hour_say_why() {
        let e = parse(r#"{"error":true,"reason":"Parameter 'start_hour' is out of range"}"#, 0.0).unwrap_err();
        assert_eq!(e, "Forecast unavailable: Parameter 'start_hour' is out of range");
        assert!(fetch(35.0, -117.0, 600.0, "tomorrow").unwrap_err().starts_with("Enter the launch time"));
    }

    #[test]
    fn the_next_hour_is_a_civil_utc_hour() {
        // 2024-02-29 23:30 UTC → 2024-03-01T00:00.
        assert_eq!(hour_after(1_709_249_400), "2024-03-01T00:00");
        assert!(is_hour(&next_hour_utc()));
    }
}
