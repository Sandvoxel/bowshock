//! RASP `.eng` parser.
//!
//! Format: `;` starts a comment. Each motor is a header line
//! `name diameter_mm length_mm delays propellant_kg total_kg manufacturer` followed by `time_s thrust_N` pairs. Delays are
//! `-` (or `,`) separated seconds such as `4-7-10`, or `P` for a plugged motor. A file may hold several motors back to back.

use crate::motor::{common_name, Motor};
use std::fmt;

/// A syntax or validity error in an `.eng` file, with the 1-based line it was found on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    /// 1-based line number in the source text.
    pub line: usize,
    /// Human-readable description.
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for ParseError {}

fn err<T>(line: usize, message: impl Into<String>) -> Result<T, ParseError> {
    Err(ParseError { line, message: message.into() })
}

/// Parses every motor in an `.eng` source string.
///
/// Comment lines preceding a motor are kept in [`Motor::comments`]; a comment containing `APPROXIMATED` sets
/// [`Motor::approximated`]. The file must contain at least one motor and every motor at least one thrust point with
/// non-negative, non-decreasing time and non-negative thrust.
pub fn parse_eng(src: &str) -> Result<Vec<Motor>, ParseError> {
    let mut motors = Vec::new();
    let mut current: Option<(usize, Motor)> = None;
    let mut pending_comments: Vec<String> = Vec::new();

    for (idx, raw) in src.lines().enumerate() {
        let line_no = idx + 1;
        let trimmed = raw.trim();
        if let Some(comment) = trimmed.strip_prefix(';') {
            let comment = comment.trim();
            if !comment.is_empty() {
                match &mut current {
                    Some((_, m)) if m.points.is_empty() => m.comments.push(comment.to_string()),
                    _ => pending_comments.push(comment.to_string()),
                }
            }
            continue;
        }
        let content = trimmed.split(';').next().unwrap_or("").trim();
        if content.is_empty() {
            continue;
        }
        let tokens: Vec<&str> = content.split_whitespace().collect();

        if tokens.len() == 2 {
            if let (Ok(t), Ok(f)) = (tokens[0].parse::<f64>(), tokens[1].parse::<f64>()) {
                let Some((_, m)) = &mut current else {
                    return err(line_no, "thrust data before any motor header");
                };
                if !t.is_finite() || !f.is_finite() || t < 0.0 || f < 0.0 {
                    return err(line_no, format!("invalid thrust point `{content}` (time and thrust must be finite and >= 0)"));
                }
                if let Some(&(prev, _)) = m.points.last() {
                    if t < prev {
                        return err(line_no, format!("time {t} is earlier than the previous point at {prev}"));
                    }
                }
                m.points.push((t, f));
                continue;
            }
        }

        // Anything else starts a new motor.
        if let Some((header_line, done)) = current.take() {
            motors.push(finish(header_line, done)?);
        }
        let mut motor = parse_header(line_no, &tokens)?;
        motor.comments = std::mem::take(&mut pending_comments);
        current = Some((line_no, motor));
    }

    if let Some((header_line, done)) = current.take() {
        motors.push(finish(header_line, done)?);
    }
    if motors.is_empty() {
        return err(1, "no motor header found");
    }
    Ok(motors)
}

fn finish(header_line: usize, mut motor: Motor) -> Result<Motor, ParseError> {
    if motor.points.is_empty() {
        return err(header_line, format!("motor `{}` has no thrust points", motor.designation));
    }
    motor.approximated = motor.comments.iter().any(|c| c.to_ascii_uppercase().contains("APPROXIMATED"));
    Ok(motor)
}

fn parse_header(line: usize, tokens: &[&str]) -> Result<Motor, ParseError> {
    if tokens.len() < 6 {
        return err(line, format!("motor header needs `name diameter length delays propellant_mass total_mass [manufacturer]`, got {} fields", tokens.len()));
    }
    let number = |what: &str, s: &str| -> Result<f64, ParseError> {
        match s.parse::<f64>() {
            Ok(v) if v.is_finite() && v >= 0.0 => Ok(v),
            _ => err(line, format!("invalid {what} `{s}`")),
        }
    };
    let diameter_mm = number("diameter", tokens[1])?;
    let length_mm = number("length", tokens[2])?;
    let propellant_mass = number("propellant mass", tokens[4])?;
    let total_mass = number("total mass", tokens[5])?;
    if diameter_mm <= 0.0 || length_mm <= 0.0 {
        return err(line, "diameter and length must be positive");
    }
    if total_mass < propellant_mass {
        return err(line, format!("total mass {total_mass} kg is less than propellant mass {propellant_mass} kg"));
    }
    let delays = parse_delays(line, tokens[3])?;
    let designation = tokens[0].to_string();
    Ok(Motor {
        common_name: common_name(&designation),
        designation,
        manufacturer: normalize_manufacturer(&tokens[6.min(tokens.len())..].join(" ")),
        diameter: diameter_mm / 1000.0,
        length: length_mm / 1000.0,
        delays,
        propellant_mass,
        total_mass,
        points: Vec::new(),
        approximated: false,
        comments: Vec::new(),
    })
}

fn parse_delays(line: usize, field: &str) -> Result<Vec<f64>, ParseError> {
    if field.eq_ignore_ascii_case("p") {
        return Ok(Vec::new());
    }
    field
        .split(['-', ','])
        .map(|s| match s.parse::<f64>() {
            Ok(v) if v.is_finite() && v >= 0.0 => Ok(v),
            _ => err(line, format!("invalid delay `{s}` in `{field}` (expected seconds like `4-7-10`, or `P`)")),
        })
        .collect()
}

/// Expands the manufacturer abbreviations that RASP files commonly use (`AT`, `RCS/Aerotech` → `AeroTech`, `CTI` → `Cesaroni`,
/// `E` → `Estes`).
fn normalize_manufacturer(raw: &str) -> String {
    let raw = raw.trim();
    if raw == "AT" || raw.to_ascii_lowercase().contains("aerotech") {
        return "AeroTech".to_string();
    }
    match raw {
        "CTI" => "Cesaroni".to_string(),
        "E" => "Estes".to_string(),
        other => other.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
; Test file with two motors
; second line of the comment
X10 29 100 4-7-10 0.020 0.050 TestCo
   0.0 0
   0.1 20 ; peak
   1.0 10

   2.0 0
; comment between motors
Y5-P 18 70.5 P .005 .015
0.5 5
1.5 5
1.6 0.0000
";

    #[test]
    fn parses_two_motors_with_comments() {
        let motors = parse_eng(SAMPLE).unwrap();
        assert_eq!(motors.len(), 2);
        let x = &motors[0];
        assert_eq!(x.designation, "X10");
        assert_eq!(x.manufacturer, "TestCo");
        assert!((x.diameter - 0.029).abs() < 1e-12 && (x.length - 0.1).abs() < 1e-12);
        assert_eq!(x.delays, vec![4.0, 7.0, 10.0]);
        assert_eq!((x.propellant_mass, x.total_mass), (0.020, 0.050));
        assert_eq!(x.points, vec![(0.0, 0.0), (0.1, 20.0), (1.0, 10.0), (2.0, 0.0)]);
        assert_eq!(x.comments, vec!["Test file with two motors", "second line of the comment"]);
        // 0.5*0.1*20 + 0.5*(20+10)*0.9 + 0.5*10*1.0
        assert!((x.total_impulse() - (1.0 + 13.5 + 5.0)).abs() < 1e-12);

        let y = &motors[1];
        assert_eq!(y.designation, "Y5-P");
        assert!(y.delays.is_empty());
        assert_eq!(y.comments, vec!["comment between motors"]);
        assert!((y.length - 0.0705).abs() < 1e-12);
        assert_eq!(y.manufacturer, "");
        assert!(!y.approximated);
    }

    #[test]
    fn approximated_flag_and_abbreviations() {
        let m = parse_eng("; APPROXIMATED from published totals\nA1 18 70 3,5 .001 .01 AT\n0.1 2\n0.2 0\n").unwrap();
        assert!(m[0].approximated);
        assert_eq!(m[0].manufacturer, "AeroTech");
        assert_eq!(m[0].delays, vec![3.0, 5.0]);
    }

    #[test]
    fn errors_carry_line_numbers() {
        let e = parse_eng("; c\n\nA1 18 70 3 .001 .01 E\n0.1 2\n0.05 1\n").unwrap_err();
        assert_eq!(e.line, 5);
        assert!(e.to_string().starts_with("line 5:"));
        assert_eq!(parse_eng("0.1 2\n").unwrap_err().line, 1);
        assert_eq!(parse_eng("A1 18 70 x .001 .01 E\n0.1 2\n").unwrap_err().line, 1);
        assert_eq!(parse_eng("\n\nA1 18 70\n").unwrap_err().line, 3);
        assert_eq!(parse_eng("A1 18 70 3 .001 .01 E\n").unwrap_err().line, 1);
        assert_eq!(parse_eng("A1 18 70 3 .02 .01 E\n0.1 2\n").unwrap_err().line, 1);
        assert_eq!(parse_eng("; only a comment\n").unwrap_err().line, 1);
        assert_eq!(parse_eng("A1 18 70 3 .001 .01 E\n0.1 -2\n").unwrap_err().line, 2);
    }
}
