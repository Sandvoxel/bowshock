//! The ThrustCurve.org API (<https://www.thrustcurve.org/info/api.html>): motor search and data download.
//!
//! The JSON types and the conversion of a download into a [`Motor`] are always built, so they can be tested against recorded
//! responses. The blocking HTTP [`Client`] needs the `thrustcurve` feature.
//!
//! Every motor taken from here gets the id `tc:<motorId>` and a [`ThrustCurveInfo`], and keeps the data file's attribution
//! comments. ThrustCurve.org asks clients to stay at about one request per second; the client spaces its requests by
//! [`MIN_REQUEST_INTERVAL`] and backs off when told `429 Too Many Requests`.

use std::fmt;
use std::time::Duration;

use base64::Engine as _;
use serde::{Deserialize, Serialize};

use crate::motor::{common_name, Motor, ThrustCurveInfo, THRUSTCURVE_ID_PREFIX};
use crate::rasp::{normalize_manufacturer, parse_eng};

/// The API root.
pub const API_BASE: &str = "https://www.thrustcurve.org/api/v1";
/// The shortest time between two requests to the API.
pub const MIN_REQUEST_INTERVAL: Duration = Duration::from_millis(1100);
/// The credit line to show wherever ThrustCurve.org data is listed.
pub const CREDIT: &str = "Data: ThrustCurve.org";

/// Why a ThrustCurve.org request failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TcError {
    /// No connection: offline, DNS failure, timeout, TLS failure. The text is the transport's reason.
    Unreachable(String),
    /// The server kept answering `429 Too Many Requests` after backing off.
    RateLimited,
    /// Any other HTTP status.
    Http(u16),
    /// The API refused the request (an invalid criterion), with its message.
    Api(String),
    /// The response is not the JSON the API documents.
    Parse(String),
    /// The motor has no data file this client can read.
    NoData(String),
}

impl fmt::Display for TcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TcError::Unreachable(why) => write!(f, "ThrustCurve unreachable ({why})"),
            TcError::RateLimited => write!(f, "ThrustCurve is rate limiting requests; try again in a moment"),
            TcError::Http(code) => write!(f, "ThrustCurve answered HTTP {code}"),
            TcError::Api(why) => write!(f, "ThrustCurve: {why}"),
            TcError::Parse(why) => write!(f, "unexpected ThrustCurve response: {why}"),
            TcError::NoData(designation) => write!(f, "ThrustCurve has no usable thrust curve for {designation}"),
        }
    }
}

impl std::error::Error for TcError {}

/// Search criteria (all optional, at least one needed). See [`SearchQuery::from_text`] for the free-text form.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SearchQuery {
    /// A motorId.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// Manufacturer name or abbreviation as ThrustCurve spells it (`AeroTech`, `Cesaroni`, `Estes`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    /// Exact designation (case-insensitive), e.g. `H128W`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub designation: Option<String>,
    /// Common name: class letter and average thrust, e.g. `H128`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub common_name: Option<String>,
    /// Impulse class letter.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impulse_class: Option<String>,
    /// Motor diameter, mm (the API's unit).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diameter: Option<f64>,
    /// `regular`, `occasional`, `OOP`, `available` or `all` (the API's default is `available`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub availability: Option<String>,
    /// Most results to return.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_results: Option<u32>,
}

/// Whether `s` is a ThrustCurve common name: `A`..`O` (or `1/8A`, `1/4A`, `1/2A`) followed by digits.
fn is_common_name(s: &str) -> bool {
    let rest = ["1/8A", "1/4A", "1/2A"]
        .iter()
        .find_map(|p| s.get(..p.len()).filter(|h| h.eq_ignore_ascii_case(p)).map(|_| &s[p.len()..]))
        .or_else(|| s.chars().next().filter(|c| ('A'..='O').contains(&c.to_ascii_uppercase())).map(|_| &s[1..]));
    rest.is_some_and(|r| !r.is_empty() && r.chars().all(|c| c.is_ascii_digit()))
}

impl SearchQuery {
    /// Turns what someone typed into criteria, or `None` for blank text. Words are read as an impulse class (`H`), a common
    /// name (`H128`), a designation (`H128W`, `163H133-14A`: anything else with a digit) or a manufacturer (the rest, matched
    /// against `manufacturers` — `(name, abbreviation)` pairs, case-insensitive, by name, abbreviation or a prefix of either).
    /// Every motor of any availability is searched.
    pub fn from_text(text: &str, manufacturers: &[(String, String)]) -> Option<SearchQuery> {
        let mut q = SearchQuery { availability: Some("all".into()), max_results: Some(200), ..Default::default() };
        let mut maker_words = Vec::new();
        for word in text.split_whitespace() {
            if word.len() == 1 && word.chars().all(|c| ('A'..='O').contains(&c.to_ascii_uppercase())) {
                q.impulse_class = Some(word.to_ascii_uppercase());
            } else if is_common_name(word) {
                q.common_name = Some(word.to_ascii_uppercase());
            } else if word.chars().any(|c| c.is_ascii_digit()) {
                q.designation = Some(word.to_ascii_uppercase());
            } else {
                maker_words.push(word);
            }
        }
        if !maker_words.is_empty() {
            let typed = maker_words.join(" ").to_ascii_lowercase();
            let found = manufacturers
                .iter()
                .find(|(n, a)| n.eq_ignore_ascii_case(&typed) || a.eq_ignore_ascii_case(&typed))
                .or_else(|| manufacturers.iter().find(|(n, a)| n.to_ascii_lowercase().starts_with(&typed) || a.to_ascii_lowercase().starts_with(&typed)));
            // An unknown maker still goes to the API, which then says it is invalid.
            q.manufacturer = Some(found.map_or_else(|| maker_words.join(" "), |(_, a)| a.clone()));
        }
        let any = q.manufacturer.is_some() || q.designation.is_some() || q.common_name.is_some() || q.impulse_class.is_some();
        any.then_some(q)
    }

    /// The query to try when a designation matched nothing: the designation's common name (`H128W-14A` → `H128`), when it
    /// has one, in place of the designation.
    pub fn fallback(&self) -> Option<SearchQuery> {
        let designation = self.designation.as_deref()?;
        let base = common_name(designation);
        let digits_end = base.char_indices().skip(1).find(|(_, c)| !c.is_ascii_digit()).map_or(base.len(), |(i, _)| i);
        let name = &base[..digits_end];
        is_common_name(name).then(|| SearchQuery { designation: None, common_name: Some(name.to_ascii_uppercase()), ..self.clone() })
    }
}

/// One motor in a search result, in SI units. A summary only: [`download`](Client::download) fetches its thrust curve.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RemoteMotor {
    /// ThrustCurve.org's motor identifier.
    pub motor_id: String,
    /// Short manufacturer name (`AeroTech`, `Cesaroni`, `Estes`), as [`Motor::manufacturer`].
    pub manufacturer: String,
    /// Full manufacturer name.
    pub manufacturer_name: String,
    /// Designation, e.g. `H128W`.
    pub designation: String,
    /// ThrustCurve's common name, e.g. `H128`.
    pub common_name: String,
    /// Impulse class letter (`A`..`O`, or a fractional `1/2A`).
    pub impulse_class: String,
    /// Certifying organisation.
    pub cert_org: String,
    /// `SU`, `reload` or `hybrid`.
    pub motor_type: String,
    /// Diameter, m.
    pub diameter: f64,
    /// Length, m.
    pub length: f64,
    /// Listed total impulse, N·s.
    pub total_impulse: f64,
    /// Listed average thrust, N.
    pub average_thrust: f64,
    /// Listed peak thrust, N.
    pub max_thrust: f64,
    /// Listed burn time, s.
    pub burn_time: f64,
    /// Loaded mass, kg.
    pub total_mass: f64,
    /// Propellant mass, kg.
    pub propellant_mass: f64,
    /// Available delays as listed, e.g. `4,6,8,10` (`P` for plugged).
    pub delays: String,
    /// Reload case.
    pub case_info: String,
    /// Propellant name.
    pub propellant: String,
    /// `regular`, `occasional` or `OOP`.
    pub availability: String,
    /// Last update of the listing, `YYYY-MM-DD`.
    pub updated_on: String,
    /// How many data files ThrustCurve holds for it (none: it cannot be simulated).
    pub data_files: u32,
}

impl RemoteMotor {
    /// The [`Motor::id`] the motor gets once downloaded.
    pub fn id(&self) -> String {
        format!("{THRUSTCURVE_ID_PREFIX}{}", self.motor_id)
    }

    fn info(&self, file: &RawFile) -> ThrustCurveInfo {
        ThrustCurveInfo {
            motor_id: self.motor_id.clone(),
            simfile_id: file.simfile_id.clone().unwrap_or_default(),
            manufacturer_name: self.manufacturer_name.clone(),
            common_name: self.common_name.clone(),
            motor_type: self.motor_type.clone(),
            cert_org: self.cert_org.clone(),
            availability: self.availability.clone(),
            data_source: file.source.clone().unwrap_or_default(),
            case_info: self.case_info.clone(),
            propellant: self.propellant.clone(),
            listed_impulse: self.total_impulse,
            listed_average_thrust: self.average_thrust,
            listed_burn_time: self.burn_time,
            updated_on: self.updated_on.clone(),
        }
    }
}

#[derive(Deserialize)]
struct RawCriterion {
    error: Option<String>,
}

#[derive(Deserialize)]
struct RawSearch {
    #[serde(default)]
    criteria: Vec<RawCriterion>,
    #[serde(default)]
    results: Vec<RawResult>,
    error: Option<String>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawResult {
    motor_id: String,
    manufacturer: Option<String>,
    manufacturer_abbrev: Option<String>,
    designation: Option<String>,
    common_name: Option<String>,
    impulse_class: Option<String>,
    cert_org: Option<String>,
    diameter: Option<f64>,
    length: Option<f64>,
    #[serde(rename = "type")]
    motor_type: Option<String>,
    avg_thrust_n: Option<f64>,
    max_thrust_n: Option<f64>,
    tot_impulse_ns: Option<f64>,
    burn_time_s: Option<f64>,
    data_files: Option<u32>,
    total_weight_g: Option<f64>,
    prop_weight_g: Option<f64>,
    delays: Option<String>,
    case_info: Option<String>,
    prop_info: Option<String>,
    availability: Option<String>,
    updated_on: Option<String>,
}

impl From<RawResult> for RemoteMotor {
    fn from(r: RawResult) -> Self {
        let abbrev = r.manufacturer_abbrev.or_else(|| r.manufacturer.clone()).unwrap_or_default();
        RemoteMotor {
            motor_id: r.motor_id,
            manufacturer: normalize_manufacturer(&abbrev),
            manufacturer_name: r.manufacturer.unwrap_or_default(),
            designation: r.designation.unwrap_or_default(),
            common_name: r.common_name.unwrap_or_default(),
            impulse_class: r.impulse_class.unwrap_or_default(),
            cert_org: r.cert_org.unwrap_or_default(),
            motor_type: r.motor_type.unwrap_or_default(),
            diameter: r.diameter.unwrap_or(0.0) / 1000.0,
            length: r.length.unwrap_or(0.0) / 1000.0,
            total_impulse: r.tot_impulse_ns.unwrap_or(0.0),
            average_thrust: r.avg_thrust_n.unwrap_or(0.0),
            max_thrust: r.max_thrust_n.unwrap_or(0.0),
            burn_time: r.burn_time_s.unwrap_or(0.0),
            total_mass: r.total_weight_g.unwrap_or(0.0) / 1000.0,
            propellant_mass: r.prop_weight_g.unwrap_or(0.0) / 1000.0,
            delays: r.delays.unwrap_or_default(),
            case_info: r.case_info.unwrap_or_default(),
            propellant: r.prop_info.unwrap_or_default(),
            availability: r.availability.unwrap_or_default(),
            updated_on: r.updated_on.unwrap_or_default(),
            data_files: r.data_files.unwrap_or(0),
        }
    }
}

/// Parses a `search.json` response. A criterion the API rejected comes back as [`TcError::Api`].
pub fn parse_search(json: &str) -> Result<Vec<RemoteMotor>, TcError> {
    let raw: RawSearch = serde_json::from_str(json).map_err(|e| TcError::Parse(e.to_string()))?;
    if let Some(e) = raw.error.filter(|e| !e.is_empty()) {
        return Err(TcError::Api(e));
    }
    if let Some(e) = raw.criteria.iter().find_map(|c| c.error.clone().filter(|e| !e.is_empty())) {
        return Err(TcError::Api(e));
    }
    Ok(raw.results.into_iter().map(RemoteMotor::from).collect())
}

#[derive(Deserialize)]
struct RawDownload {
    #[serde(default)]
    results: Vec<RawFile>,
    error: Option<String>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RawFile {
    motor_id: String,
    simfile_id: Option<String>,
    format: Option<String>,
    source: Option<String>,
    data: Option<String>,
    #[serde(default)]
    samples: Vec<RawSample>,
}

#[derive(Deserialize)]
struct RawSample {
    time: f64,
    thrust: f64,
}

impl RawFile {
    /// Preference among a motor's data files: RASP before others, certification data before manufacturer before user data,
    /// then more samples.
    fn rank(&self) -> (u8, u8, std::cmp::Reverse<usize>) {
        let format = u8::from(!self.format.as_deref().is_some_and(|f| f.eq_ignore_ascii_case("RASP")));
        let source = match self.source.as_deref() {
            Some("cert") => 0,
            Some("mfr") => 1,
            _ => 2,
        };
        (format, source, std::cmp::Reverse(self.samples.len()))
    }

    /// The motor in this file's RASP data, if it is RASP and parses.
    fn rasp_motor(&self) -> Option<Motor> {
        if !self.format.as_deref().is_some_and(|f| f.eq_ignore_ascii_case("RASP")) {
            return None;
        }
        let encoded: String = self.data.as_deref()?.chars().filter(|c| !c.is_whitespace()).collect();
        let bytes = base64::engine::general_purpose::STANDARD.decode(encoded).ok()?;
        parse_eng(&String::from_utf8_lossy(&bytes)).ok()?.into_iter().next()
    }

    /// A motor from the extracted samples and the listing's masses and dimensions (for files that are not RASP).
    fn sampled_motor(&self, meta: &RemoteMotor) -> Option<Motor> {
        let mut points = Vec::with_capacity(self.samples.len());
        for s in &self.samples {
            if !(s.time.is_finite() && s.thrust.is_finite() && s.time >= 0.0 && s.thrust >= 0.0) || points.last().is_some_and(|&(t, _)| s.time < t) {
                return None;
            }
            points.push((s.time, s.thrust));
        }
        let usable = points.len() >= 2 && meta.diameter > 0.0 && meta.length > 0.0 && meta.total_mass >= meta.propellant_mass && meta.propellant_mass > 0.0;
        usable.then(|| Motor {
            id: String::new(),
            thrustcurve: None,
            designation: String::new(),
            manufacturer: String::new(),
            common_name: String::new(),
            diameter: meta.diameter,
            length: meta.length,
            delays: listed_delays(&meta.delays),
            propellant_mass: meta.propellant_mass,
            total_mass: meta.total_mass,
            points,
            approximated: false,
            comments: vec![format!("{} {}", meta.manufacturer, meta.designation), "provided by ThrustCurve.org (www.thrustcurve.org)".into()],
        })
    }
}

/// The delays of a listing such as `4,6,8,10` or `2-4-8-P`, s (`P` for plugged is not a delay).
fn listed_delays(listing: &str) -> Vec<f64> {
    listing.split([',', '-']).filter_map(|d| d.trim().parse::<f64>().ok()).filter(|d| d.is_finite() && *d >= 0.0).collect()
}

/// The motor for `meta` from its best usable data file among `files` (see [`RawFile::rank`]): the file's curve, masses and
/// dimensions, the listing's designation and manufacturer, the id `tc:<motorId>` and the listing as [`ThrustCurveInfo`].
fn motor_from(meta: &RemoteMotor, files: &[&RawFile]) -> Option<Motor> {
    let mut files = files.to_vec();
    files.sort_by_key(|f| f.rank());
    files.into_iter().find_map(|file| {
        let mut motor = file.rasp_motor().or_else(|| file.sampled_motor(meta))?;
        // The listing has every delay the motor ships with; a data file's header carries only the one it was tested with.
        if !meta.delays.trim().is_empty() {
            motor.delays = listed_delays(&meta.delays);
        }
        if !meta.designation.is_empty() {
            motor.designation = meta.designation.clone();
        }
        motor.common_name = common_name(&motor.designation);
        if !meta.manufacturer.is_empty() {
            motor.manufacturer = meta.manufacturer.clone();
        }
        motor.id = meta.id();
        motor.thrustcurve = Some(meta.info(file));
        Some(motor)
    })
}

/// Parses a `download.json` response into one [`Motor`] per entry of `requested` that has a usable data file, in the order
/// of `requested`. Motors without one are left out (compare the ids to find them).
pub fn parse_download(json: &str, requested: &[RemoteMotor]) -> Result<Vec<Motor>, TcError> {
    let raw: RawDownload = serde_json::from_str(json).map_err(|e| TcError::Parse(e.to_string()))?;
    if let Some(e) = raw.error.filter(|e| !e.is_empty()) {
        return Err(TcError::Api(e));
    }
    Ok(requested
        .iter()
        .filter_map(|meta| {
            let files: Vec<&RawFile> = raw.results.iter().filter(|f| f.motor_id == meta.motor_id).collect();
            motor_from(meta, &files)
        })
        .collect())
}

/// One `(name, abbreviation)` per manufacturer in a `metadata.json` response.
pub fn parse_manufacturers(json: &str) -> Result<Vec<(String, String)>, TcError> {
    #[derive(Deserialize)]
    struct NameAndAbbrev {
        name: String,
        abbrev: Option<String>,
    }
    #[derive(Deserialize)]
    struct Metadata {
        #[serde(default)]
        manufacturers: Vec<NameAndAbbrev>,
    }
    let raw: Metadata = serde_json::from_str(json).map_err(|e| TcError::Parse(e.to_string()))?;
    Ok(raw.manufacturers.into_iter().map(|m| (m.name.clone(), m.abbrev.unwrap_or(m.name))).collect())
}

#[cfg(feature = "thrustcurve")]
pub use client::{download, search, Client, DOWNLOAD_BATCH};

#[cfg(feature = "thrustcurve")]
mod client {
    use super::*;
    use std::sync::{LazyLock, Mutex, OnceLock};
    use std::time::Instant;

    /// How often a request is retried after `429 Too Many Requests` before giving up with [`TcError::RateLimited`].
    const RATE_LIMIT_RETRIES: u32 = 3;
    /// The longest a `Retry-After` is honoured for, s (a longer one gives up at once rather than hang the caller).
    const MAX_RETRY_AFTER: u64 = 10;
    /// Most motorIds per download request.
    pub const DOWNLOAD_BATCH: usize = 20;

    /// When the last request was sent, shared by every client in the process so the whole app stays under the API's rate.
    static LAST_REQUEST: Mutex<Option<Instant>> = Mutex::new(None);

    /// A blocking ThrustCurve.org client. Calls block for at most a few seconds (connect and overall timeouts, bounded
    /// back-off): run them off the UI thread.
    #[derive(Debug, Clone)]
    pub struct Client {
        agent: ureq::Agent,
        base: String,
        manufacturers: std::sync::Arc<OnceLock<Vec<(String, String)>>>,
    }

    impl Default for Client {
        fn default() -> Self {
            Client::new()
        }
    }

    impl Client {
        /// A client of [`API_BASE`] that identifies itself as Bowshock.
        pub fn new() -> Client {
            Client::with_timeout(Duration::from_secs(20))
        }

        /// A client whose requests give up after `timeout` (connecting gives up after at most 5 s).
        pub fn with_timeout(timeout: Duration) -> Client {
            let config = ureq::Agent::config_builder()
                .http_status_as_error(false)
                .timeout_connect(Some(timeout.min(Duration::from_secs(5))))
                .timeout_global(Some(timeout))
                .user_agent(concat!("Bowshock/", env!("CARGO_PKG_VERSION"), " (rocket design app; https://github.com/Sandvoxel/bowshock)"))
                .build();
            Client { agent: config.into(), base: API_BASE.into(), manufacturers: Default::default() }
        }

        /// The same client talking to another API root (a mirror, or an unreachable address to try the offline path).
        pub fn with_base(self, base: impl Into<String>) -> Client {
            Client { base: base.into().trim_end_matches('/').to_string(), ..self }
        }

        /// Sends one request (spaced from the last by [`MIN_REQUEST_INTERVAL`], retried after a `429`) and returns the body.
        fn request(&self, endpoint: &str, body: Option<&serde_json::Value>) -> Result<String, TcError> {
            let url = format!("{}/{endpoint}", self.base);
            let mut wait = Duration::from_secs(2);
            for attempt in 0..=RATE_LIMIT_RETRIES {
                pace();
                let sent = match body {
                    Some(b) => self.agent.post(&url).header("Content-Type", "application/json").send(b.to_string()),
                    None => self.agent.get(&url).call(),
                };
                let mut response = sent.map_err(|e| TcError::Unreachable(e.to_string()))?;
                let status = response.status().as_u16();
                if status == 429 {
                    let after = response.headers().get("retry-after").and_then(|v| v.to_str().ok()).and_then(|v| v.trim().parse::<u64>().ok());
                    if attempt == RATE_LIMIT_RETRIES || after.is_some_and(|s| s > MAX_RETRY_AFTER) {
                        return Err(TcError::RateLimited);
                    }
                    std::thread::sleep(after.map_or(wait, Duration::from_secs));
                    wait *= 2;
                    continue;
                }
                if !(200..300).contains(&status) {
                    return Err(TcError::Http(status));
                }
                return response.body_mut().with_config().limit(64 << 20).read_to_string().map_err(|e| TcError::Unreachable(e.to_string()));
            }
            Err(TcError::RateLimited)
        }

        /// Every manufacturer as `(name, abbreviation)`; fetched once per client.
        pub fn manufacturers(&self) -> Result<&[(String, String)], TcError> {
            if let Some(m) = self.manufacturers.get() {
                return Ok(m);
            }
            let list = parse_manufacturers(&self.request("metadata.json", None)?)?;
            Ok(self.manufacturers.get_or_init(|| list))
        }

        /// Motors matching `query`.
        pub fn search(&self, query: &SearchQuery) -> Result<Vec<RemoteMotor>, TcError> {
            let body = serde_json::to_value(query).map_err(|e| TcError::Parse(e.to_string()))?;
            parse_search(&self.request("search.json", Some(&body))?)
        }

        /// Motors matching typed text (see [`SearchQuery::from_text`]); an unmatched designation is retried as its common
        /// name. Blank text finds nothing without a request.
        pub fn search_text(&self, text: &str) -> Result<Vec<RemoteMotor>, TcError> {
            let needs_makers = text.split_whitespace().any(|w| !w.chars().any(|c| c.is_ascii_digit()) && w.len() > 1);
            let makers = if needs_makers { self.manufacturers()? } else { &[] };
            let Some(query) = SearchQuery::from_text(text, makers) else {
                return Ok(Vec::new());
            };
            let found = self.search(&query)?;
            match query.fallback() {
                Some(fallback) if found.is_empty() => self.search(&fallback),
                _ => Ok(found),
            }
        }

        /// The thrust curves of `motors` (in batches of [`DOWNLOAD_BATCH`]); motors without a usable data file are left out.
        pub fn download(&self, motors: &[RemoteMotor]) -> Result<Vec<Motor>, TcError> {
            let mut out = Vec::with_capacity(motors.len());
            for batch in motors.chunks(DOWNLOAD_BATCH) {
                let ids: Vec<&str> = batch.iter().map(|m| m.motor_id.as_str()).collect();
                let body = serde_json::json!({ "motorIds": ids, "data": "both", "maxResults": 50 * batch.len() });
                out.extend(parse_download(&self.request("download.json", Some(&body))?, batch)?);
            }
            Ok(out)
        }

        /// One motor by ThrustCurve motorId: its listing, then its curve.
        pub fn download_id(&self, motor_id: &str) -> Result<Motor, TcError> {
            let query = SearchQuery { id: Some(motor_id.into()), availability: Some("all".into()), ..Default::default() };
            let meta = self.search(&query)?.into_iter().find(|m| m.motor_id == motor_id).ok_or_else(|| TcError::Api(format!("no motor {motor_id}")))?;
            self.download(std::slice::from_ref(&meta))?.into_iter().next().ok_or(TcError::NoData(meta.designation))
        }
    }

    /// Sleeps until [`MIN_REQUEST_INTERVAL`] has passed since the previous request, then marks this one as sent.
    fn pace() {
        let mut last = LAST_REQUEST.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(t) = *last {
            let since = t.elapsed();
            if since < MIN_REQUEST_INTERVAL {
                std::thread::sleep(MIN_REQUEST_INTERVAL - since);
            }
        }
        *last = Some(Instant::now());
    }

    /// The process-wide client behind [`search`] and [`download`].
    static SHARED: LazyLock<Client> = LazyLock::new(Client::new);

    /// Motors matching typed text, with a shared client (see [`Client::search_text`]).
    pub fn search(text: &str) -> Result<Vec<RemoteMotor>, TcError> {
        SHARED.search_text(text)
    }

    /// One motor's thrust curve by ThrustCurve motorId, with a shared client (see [`Client::download_id`]).
    pub fn download(motor_id: &str) -> Result<Motor, TcError> {
        SHARED.download_id(motor_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SEARCH: &str = include_str!("../tests/fixtures/search_h128.json");
    const DOWNLOAD: &str = include_str!("../tests/fixtures/download_h128w.json");
    const METADATA: &str = include_str!("../tests/fixtures/metadata.json");

    #[test]
    fn search_response_parses_into_si() {
        let found = parse_search(SEARCH).unwrap();
        let h = found.iter().find(|m| m.designation == "H128W").unwrap();
        assert_eq!(h.motor_id, "5f4294d20002310000000083");
        assert_eq!(h.manufacturer, "AeroTech");
        assert_eq!(h.common_name, "H128");
        assert!((h.diameter - 0.029).abs() < 1e-12 && (h.length - 0.194).abs() < 1e-12);
        assert!((h.total_impulse - 172.9).abs() < 1e-9);
        assert!((h.total_mass - 0.206).abs() < 1e-12 && (h.propellant_mass - 0.0942).abs() < 1e-12);
        assert_eq!(h.availability, "regular");
        assert!(h.cert_org.starts_with("Tripoli"));
        assert_eq!(h.id(), "tc:5f4294d20002310000000083");
    }

    #[test]
    fn rejected_criteria_are_api_errors() {
        let json = r#"{"criteria":[{"name":"manufacturer","value":"aerotech","error":"Invalid manufacturer value \"aerotech\".","matches":0}],"matches":0,"results":[]}"#;
        assert_eq!(parse_search(json), Err(TcError::Api("Invalid manufacturer value \"aerotech\".".into())));
        assert!(matches!(parse_search("<html>"), Err(TcError::Parse(_))));
    }

    #[test]
    fn download_prefers_certified_rasp_and_keeps_metadata() {
        let meta = parse_search(SEARCH).unwrap().into_iter().find(|m| m.designation == "H128W").unwrap();
        let motors = parse_download(DOWNLOAD, std::slice::from_ref(&meta)).unwrap();
        assert_eq!(motors.len(), 1);
        let m = &motors[0];
        assert_eq!(m.id, "tc:5f4294d20002310000000083");
        assert_eq!(m.designation, "H128W");
        assert_eq!(m.manufacturer, "AeroTech");
        let info = m.thrustcurve.as_ref().unwrap();
        assert_eq!(info.data_source, "cert");
        assert_eq!(info.availability, "regular");
        assert!((m.diameter - 0.029).abs() < 1e-12 && (m.total_mass - 0.2016).abs() < 1e-12);
        // The certified curve integrates to the listed 172.9 N·s within a few per cent.
        assert!((m.total_impulse() - 172.9).abs() / 172.9 < 0.05, "{}", m.total_impulse());
        assert!(m.comments.iter().any(|c| c.contains("ThrustCurve")));
    }

    #[test]
    fn samples_stand_in_when_there_is_no_rasp_file() {
        let meta = RemoteMotor {
            motor_id: "abc".into(),
            designation: "X10".into(),
            manufacturer: "Test".into(),
            diameter: 0.018,
            length: 0.07,
            total_mass: 0.02,
            propellant_mass: 0.01,
            delays: "3,5,P".into(),
            ..Default::default()
        };
        let json = r#"{"results":[{"motorId":"abc","simfileId":"s1","format":"RockSim","source":"mfr","data":"PD94bWw+","samples":[{"time":0.1,"thrust":10},{"time":0.5,"thrust":8},{"time":0.9,"thrust":0}]}]}"#;
        let m = parse_download(json, std::slice::from_ref(&meta)).unwrap().remove(0);
        assert_eq!(m.points, vec![(0.1, 10.0), (0.5, 8.0), (0.9, 0.0)]);
        assert_eq!(m.delays, vec![3.0, 5.0]);
        assert_eq!((m.id.as_str(), m.thrustcurve.unwrap().simfile_id.as_str()), ("tc:abc", "s1"));
        // No usable file at all: the motor is left out.
        let empty = r#"{"results":[{"motorId":"abc","format":"RockSim","samples":[]}]}"#;
        assert!(parse_download(empty, &[meta]).unwrap().is_empty());
    }

    #[test]
    fn typed_text_becomes_criteria() {
        let makers = parse_manufacturers(METADATA).unwrap();
        assert!(makers.iter().any(|(n, a)| n == "Cesaroni Technology" && a == "Cesaroni"));
        let q = |t: &str| SearchQuery::from_text(t, &makers);
        assert_eq!(q("h128").unwrap().common_name.as_deref(), Some("H128"));
        assert_eq!(q("1/2a3").unwrap().common_name.as_deref(), Some("1/2A3"));
        let d = q("H128W").unwrap();
        assert_eq!((d.designation.as_deref(), d.availability.as_deref()), (Some("H128W"), Some("all")));
        assert_eq!(d.fallback().unwrap().common_name.as_deref(), Some("H128"));
        assert_eq!(q("163H133-14A").unwrap().fallback().unwrap().common_name.as_deref(), Some("H133"));
        let both = q("cesaroni  J").unwrap();
        assert_eq!((both.manufacturer.as_deref(), both.impulse_class.as_deref()), (Some("Cesaroni"), Some("J")));
        assert_eq!(q("aero").unwrap().manufacturer.as_deref(), Some("AeroTech"));
        assert!(q("   ").is_none());
        let json = serde_json::to_string(&q("H128").unwrap()).unwrap();
        assert_eq!(json, r#"{"commonName":"H128","availability":"all","maxResults":200}"#);
    }
}
