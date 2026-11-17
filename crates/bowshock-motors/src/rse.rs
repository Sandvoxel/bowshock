//! RockSim engine (`.rse`) parser.
//!
//! The format is XML: `<engine-database><engine-list><engine mfg code dia len initWt propWt delays …><comments>…</comments>
//! <data><eng-data t f m cg/>…</data></engine></engine-list></engine-database>`. Lengths (`dia`, `len`, `cg`) are millimetres,
//! masses (`initWt`, `propWt`, `m`) grams, `t` seconds and `f` newtons; `delays` is a comma list of seconds in which `P` (or a
//! number of [`PLUGGED_DELAY`] seconds and more, the stand-in some tools write) means no ejection charge. OpenRocket 26.xx stores
//! the user-defined thrust curves of a design this way inside the `.ork`.
//!
//! The reader is a small hand-written tag scanner (the crate has no XML dependency): it handles quoted attributes, entities,
//! comments, processing instructions and CDATA, and ignores every element and attribute it has no use for. The per-sample mass
//! column is not kept ([`Motor`] has one propellant mass, burned off in proportion to impulse); `initWt` and `propWt` give the
//! masses as RASP's header does.

use crate::motor::{common_name, Motor};
use crate::rasp::normalize_manufacturer;
use std::fmt;

/// Delays at or above this many seconds mean "plugged" (no ejection charge); some tools write 1000 for it.
pub const PLUGGED_DELAY: f64 = 999.0;

/// Why an `.rse` source could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RseError {
    /// Human-readable description; names the engine code when the fault is in one engine.
    pub message: String,
}

impl fmt::Display for RseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for RseError {}

fn err<T>(message: impl Into<String>) -> Result<T, RseError> {
    Err(RseError { message: message.into() })
}

/// One start or empty-element tag: its (lower-cased) name and its attributes (names lower-cased, entities decoded).
struct Tag {
    name: String,
    attrs: Vec<(String, String)>,
    /// `<x/>`: no content follows.
    empty: bool,
}

impl Tag {
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(n, _)| n == name).map(|(_, v)| v.as_str())
    }
}

/// The text a markup chunk stands for: the five predefined entities and numeric character references decoded.
fn decode(s: &str) -> String {
    if !s.contains('&') {
        return s.to_string();
    }
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(at) = rest.find('&') {
        out.push_str(&rest[..at]);
        rest = &rest[at..];
        let replaced = rest.find(';').filter(|&end| end <= 10).and_then(|end| {
            let ch = match &rest[1..end] {
                "amp" => Some('&'),
                "lt" => Some('<'),
                "gt" => Some('>'),
                "quot" => Some('"'),
                "apos" => Some('\''),
                num => num
                    .strip_prefix('#')
                    .and_then(|n| n.strip_prefix(['x', 'X']).map_or_else(|| n.parse().ok(), |h| u32::from_str_radix(h, 16).ok()))
                    .and_then(char::from_u32),
            }?;
            Some((ch, end))
        });
        match replaced {
            Some((ch, end)) => {
                out.push(ch);
                rest = &rest[end + 1..];
            }
            None => {
                out.push('&');
                rest = &rest[1..];
            }
        }
    }
    out.push_str(rest);
    out
}

/// What the scanner found.
enum Item {
    Open(Tag),
    Close(String),
    Text(String),
}

/// Splits `src` into tags and text; comments, processing instructions and doctypes are skipped, CDATA becomes text.
fn scan(src: &str) -> Result<Vec<Item>, RseError> {
    let mut items = Vec::new();
    let mut rest = src;
    while !rest.is_empty() {
        let Some(lt) = rest.find('<') else {
            items.push(Item::Text(decode(rest)));
            break;
        };
        if lt > 0 {
            items.push(Item::Text(decode(&rest[..lt])));
        }
        rest = &rest[lt..];
        if let Some(body) = rest.strip_prefix("<!--") {
            let Some(end) = body.find("-->") else { return err("unterminated XML comment") };
            rest = &body[end + 3..];
        } else if let Some(body) = rest.strip_prefix("<![CDATA[") {
            let Some(end) = body.find("]]>") else { return err("unterminated CDATA section") };
            items.push(Item::Text(body[..end].to_string()));
            rest = &body[end + 3..];
        } else if rest.starts_with("<?") || rest.starts_with("<!") {
            let Some(end) = rest.find('>') else { return err("unterminated XML declaration") };
            rest = &rest[end + 1..];
        } else {
            let Some((tag, after)) = tag(&rest[1..]) else { return err("malformed XML tag") };
            items.push(tag);
            rest = after;
        }
    }
    Ok(items)
}

/// Parses a tag whose `<` has been consumed; returns it and the text after its `>`.
fn tag(s: &str) -> Option<(Item, &str)> {
    if let Some(body) = s.strip_prefix('/') {
        let end = body.find('>')?;
        return Some((Item::Close(body[..end].trim().to_ascii_lowercase()), &body[end + 1..]));
    }
    let name_end = s.find(|c: char| c.is_whitespace() || c == '>' || c == '/').unwrap_or(s.len());
    let name = s[..name_end].to_ascii_lowercase();
    let mut rest = &s[name_end..];
    let mut attrs = Vec::new();
    loop {
        rest = rest.trim_start();
        if let Some(after) = rest.strip_prefix("/>") {
            return Some((Item::Open(Tag { name, attrs, empty: true }), after));
        }
        if let Some(after) = rest.strip_prefix('>') {
            return Some((Item::Open(Tag { name, attrs, empty: false }), after));
        }
        let eq = rest.find('=')?;
        let key = rest[..eq].trim().to_ascii_lowercase();
        let value = rest[eq + 1..].trim_start();
        let quote = value.chars().next().filter(|q| *q == '"' || *q == '\'')?;
        let close = value[1..].find(quote)?;
        attrs.push((key, decode(&value[1..1 + close])));
        rest = &value[close + 2..];
    }
}

/// An engine being read.
struct Engine {
    motor: Motor,
    init_wt: Option<f64>,
    prop_wt: Option<f64>,
    /// First and last per-sample mass, g (the fallback when a header mass is missing).
    first_mass: Option<f64>,
    last_mass: Option<f64>,
}

/// Parses every `<engine>` of an `.rse` source into a [`Motor`] (SI units; [`Motor::id`] is [`Motor::file_id`]).
///
/// Comments of the engine are kept in [`Motor::comments`], one line each. An engine with no thrust samples, a non-finite or
/// negative value, non-increasing times, a missing or non-positive size, or a propellant mass above its total mass is an error
/// naming its `code`; so is a source with no `<engine>`.
pub fn parse_rse(src: &str) -> Result<Vec<Motor>, RseError> {
    let items = scan(src)?;
    let mut motors = Vec::new();
    let mut current: Option<Engine> = None;
    let mut in_comments = false;
    let mut comment_text = String::new();
    for item in items {
        match item {
            Item::Open(t) => match t.name.as_str() {
                "engine" => {
                    if let Some(done) = current.take() {
                        motors.push(finish(done)?);
                    }
                    let engine = open_engine(&t)?;
                    if t.empty {
                        motors.push(finish(engine)?);
                    } else {
                        current = Some(engine);
                    }
                }
                "comments" if current.is_some() && !t.empty => {
                    in_comments = true;
                    comment_text.clear();
                }
                "eng-data" => {
                    if let Some(e) = &mut current {
                        sample(e, &t)?;
                    }
                }
                _ => {}
            },
            Item::Close(name) => match name.as_str() {
                "engine" => {
                    if let Some(done) = current.take() {
                        motors.push(finish(done)?);
                    }
                }
                "comments" if in_comments => {
                    in_comments = false;
                    if let Some(e) = &mut current {
                        e.motor.comments = comment_text.lines().map(str::trim).filter(|l| !l.is_empty()).map(str::to_string).collect();
                    }
                }
                _ => {}
            },
            Item::Text(text) => {
                if in_comments {
                    comment_text.push_str(&text);
                }
            }
        }
    }
    if let Some(done) = current.take() {
        motors.push(finish(done)?);
    }
    if motors.is_empty() {
        return err("no <engine> found");
    }
    Ok(motors)
}

fn open_engine(t: &Tag) -> Result<Engine, RseError> {
    let code = t.attr("code").map(str::trim).unwrap_or("");
    if code.is_empty() {
        return err("an <engine> has no code");
    }
    let number = |name: &str| -> Result<Option<f64>, RseError> {
        match t.attr(name).map(str::trim) {
            None | Some("") => Ok(None),
            Some(s) => match s.parse::<f64>() {
                Ok(v) if v.is_finite() && v >= 0.0 => Ok(Some(v)),
                _ => err(format!("engine {code}: invalid {name} `{s}`")),
            },
        }
    };
    let size = |name: &str, what: &str| -> Result<f64, RseError> {
        match number(name)? {
            Some(v) if v > 0.0 => Ok(v / 1000.0),
            _ => err(format!("engine {code}: {what} is missing or not positive")),
        }
    };
    let diameter = size("dia", "diameter")?;
    let length = size("len", "length")?;
    let init_wt = number("initwt")?;
    let prop_wt = number("propwt")?;
    let delays = parse_delays(code, t.attr("delays").unwrap_or(""))?;
    let motor = Motor {
        id: String::new(),
        thrustcurve: None,
        common_name: common_name(code),
        designation: code.to_string(),
        manufacturer: normalize_manufacturer(t.attr("mfg").unwrap_or("")),
        diameter,
        length,
        delays,
        propellant_mass: 0.0,
        total_mass: 0.0,
        points: Vec::new(),
        approximated: false,
        comments: Vec::new(),
    };
    Ok(Engine { motor, init_wt, prop_wt, first_mass: None, last_mass: None })
}

/// `7,9,11` seconds; `P` and delays of [`PLUGGED_DELAY`] or more add none (an empty list is a plugged motor).
fn parse_delays(code: &str, field: &str) -> Result<Vec<f64>, RseError> {
    let mut out = Vec::new();
    for s in field.split([',', ';', ' ']).map(str::trim).filter(|s| !s.is_empty()) {
        if s.eq_ignore_ascii_case("p") {
            continue;
        }
        match s.parse::<f64>() {
            Ok(v) if v.is_finite() && v >= 0.0 => {
                if v < PLUGGED_DELAY {
                    out.push(v);
                }
            }
            _ => return err(format!("engine {code}: invalid delay `{s}` in `{field}`")),
        }
    }
    Ok(out)
}

fn sample(e: &mut Engine, t: &Tag) -> Result<(), RseError> {
    let code = e.motor.designation.clone();
    let read = |name: &str| -> Result<f64, RseError> {
        match t.attr(name).map(str::trim).map(str::parse::<f64>) {
            Some(Ok(v)) if v.is_finite() && v >= 0.0 => Ok(v),
            Some(_) => err(format!("engine {code}: invalid thrust sample (`{name}` must be finite and >= 0)")),
            None => err(format!("engine {code}: thrust sample without `{name}`")),
        }
    };
    let (time, force) = (read("t")?, read("f")?);
    if let Some(&(prev, _)) = e.motor.points.last() {
        if time <= prev {
            return err(format!("engine {code}: time {time} is not later than the previous sample at {prev}"));
        }
    }
    if t.attr("m").is_some() {
        let mass = read("m")?;
        e.first_mass.get_or_insert(mass);
        e.last_mass = Some(mass);
    }
    e.motor.points.push((time, force));
    Ok(())
}

fn finish(e: Engine) -> Result<Motor, RseError> {
    let Engine { mut motor, init_wt, prop_wt, first_mass, last_mass } = e;
    let code = motor.designation.clone();
    if motor.points.is_empty() {
        return err(format!("engine {code} has no thrust samples"));
    }
    let total_g = init_wt.or(first_mass);
    let prop_g = prop_wt.or_else(|| Some(first_mass? - last_mass?));
    let (Some(total_g), Some(prop_g)) = (total_g, prop_g) else {
        return err(format!("engine {code}: no initWt/propWt (and no sample masses to take them from)"));
    };
    if prop_g > total_g {
        return err(format!("engine {code}: propellant mass {prop_g} g is more than the total mass {total_g} g"));
    }
    motor.total_mass = total_g / 1000.0;
    motor.propellant_mass = prop_g / 1000.0;
    motor.approximated = motor.comments.iter().any(|c| c.to_ascii_uppercase().contains("APPROXIMATED"));
    motor.id = motor.file_id();
    Ok(motor)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<engine-database>
 <engine-list>
  <engine mfg="AeroTech" code="I32N" Type="unknown" dia="38" len="356" initWt="647" propWt="380.5" delays="1000" auto-calc-mass="0">
   <comments>From a post &amp; a photo
second line</comments>
   <data>
    <eng-data t="0" f="0" m="647" cg="178"/>
    <eng-data t="0.5" f="100" m="600" cg="178"/>
    <eng-data t="2" f="40" m="400" cg="178"/>
    <eng-data t="3" f="0" m="266.5" cg="178"/>
   </data>
  </engine>
  <!-- a second engine -->
  <engine mfg="AT" code="H100W" dia="29" len="124.5" initWt="200" propWt="100" delays="P,7,10,1000">
   <data><eng-data t="0.1" f="10" m="200"/><eng-data t="1" f="5" m="100"/></data>
  </engine>
 </engine-list>
</engine-database>"#;

    #[test]
    fn converts_units_delays_and_comments() {
        let motors = parse_rse(SAMPLE).unwrap();
        assert_eq!(motors.len(), 2);
        let i = &motors[0];
        assert_eq!((i.designation.as_str(), i.manufacturer.as_str()), ("I32N", "AeroTech"));
        assert!((i.diameter - 0.038).abs() < 1e-12 && (i.length - 0.356).abs() < 1e-12);
        assert!((i.total_mass - 0.647).abs() < 1e-12 && (i.propellant_mass - 0.3805).abs() < 1e-12);
        assert!(i.delays.is_empty(), "a delay of 1000 s is a plugged motor");
        assert_eq!(i.points, vec![(0.0, 0.0), (0.5, 100.0), (2.0, 40.0), (3.0, 0.0)]);
        assert_eq!(i.comments, vec!["From a post & a photo", "second line"]);
        assert!(i.id.starts_with("eng:"));
        // 0.5*0.5*100 + 0.5*(100+40)*1.5 + 0.5*40*1
        assert!((i.total_impulse() - (25.0 + 105.0 + 20.0)).abs() < 1e-9);
        let h = &motors[1];
        assert_eq!(h.manufacturer, "AeroTech");
        assert_eq!(h.delays, vec![7.0, 10.0]);
        assert!((h.length - 0.1245).abs() < 1e-12);
        assert_eq!(h.common_name, "H100W");
    }

    #[test]
    fn plugged_spellings() {
        for delays in ["P", "p", "1000", "", "P,1000"] {
            let src = format!("<engine mfg=\"X\" code=\"A1\" dia=\"18\" len=\"70\" initWt=\"10\" propWt=\"5\" delays=\"{delays}\"><data><eng-data t=\"1\" f=\"1\"/></data></engine>");
            assert!(parse_rse(&src).unwrap()[0].delays.is_empty(), "{delays:?}");
        }
    }

    fn one(attrs: &str, data: &str) -> Result<Vec<Motor>, RseError> {
        // Attributes in `attrs` come first, so they override the defaults.
        parse_rse(&format!("<engine {attrs} code=\"Z9\" mfg=\"X\" dia=\"29\" len=\"100\" initWt=\"50\" propWt=\"20\"><data>{data}</data></engine>"))
    }

    #[test]
    fn rejections_name_the_engine() {
        let ok = "<eng-data t=\"0.1\" f=\"5\"/><eng-data t=\"0.2\" f=\"0\"/>";
        assert!(one("", ok).is_ok());
        for (attrs, data, why) in [
            ("", "", "no thrust samples"),
            ("", "<eng-data t=\"0.1\" f=\"5\"/><eng-data t=\"0.1\" f=\"4\"/>", "not later"),
            ("", "<eng-data t=\"0.2\" f=\"5\"/><eng-data t=\"0.1\" f=\"4\"/>", "not later"),
            ("", "<eng-data t=\"0.1\" f=\"-5\"/>", "invalid thrust sample"),
            ("", "<eng-data t=\"NaN\" f=\"5\"/>", "invalid thrust sample"),
            ("", "<eng-data t=\"0.1\" f=\"inf\"/>", "invalid thrust sample"),
            ("", "<eng-data t=\"0.1\"/>", "without `f`"),
            ("dia=\"-3\"", ok, "invalid dia"),
            ("len=\"0\"", ok, "length"),
            ("propWt=\"90\"", ok, "more than the total"),
            ("delays=\"x\"", ok, "invalid delay"),
            ("initWt=\"abc\"", ok, "invalid initwt"),
        ] {
            let e = one(attrs, data).unwrap_err().to_string();
            assert!(e.contains("Z9") && e.contains(why), "{attrs} {data}: {e}");
        }
        assert!(parse_rse("<engine-database/>").unwrap_err().to_string().contains("no <engine>"));
        assert!(parse_rse("<engine dia=\"1\">").unwrap_err().to_string().contains("no code"));
        assert!(parse_rse("<engine code=").is_err());
    }

    #[test]
    fn missing_header_masses_come_from_the_samples() {
        let m = parse_rse(
            "<engine code=\"A\" dia=\"18\" len=\"70\"><data><eng-data t=\"0\" f=\"0\" m=\"30\"/><eng-data t=\"1\" f=\"4\" m=\"12\"/></data></engine>",
        )
        .unwrap();
        assert!((m[0].total_mass - 0.030).abs() < 1e-12 && (m[0].propellant_mass - 0.018).abs() < 1e-12);
        assert!(parse_rse("<engine code=\"A\" dia=\"18\" len=\"70\"><data><eng-data t=\"1\" f=\"4\"/></data></engine>").is_err());
    }
}
