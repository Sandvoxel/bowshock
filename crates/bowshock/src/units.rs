//! Display units, number formatting and quantity-expression parsing.
//!
//! The whole app stores and computes in SI. This module is the only place where values are converted to and from a
//! display unit, rendered as text, or parsed back from what the user typed into a numeric field.
//!
//! Typed fields accept small arithmetic expressions: `2*38`, `(120-50)/2`, `3in`, `1' 2"`, `0.5*{body.od}+2`.
//! Numbers with a unit suffix are converted into the field's display unit while parsing; plain numbers are already in
//! the field's display unit; `{id}` tokens are references to other fields, resolved at evaluation time.

use std::fmt;

/// Physical quantities the UI edits and displays.
///
/// SI bases: length m, mass kg, angle rad, velocity m/s, acceleration m/s², force N, impulse N·s, density kg/m³,
/// area m², time s, temperature K, roughness m, altitude m, percentage as a ratio (1.0 = 100 %), calibers as a ratio.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Quantity {
    /// Lengths (m).
    Length,
    /// Masses (kg).
    Mass,
    /// Plane angles (rad).
    Angle,
    /// Speeds (m/s).
    Velocity,
    /// Accelerations (m/s²).
    Acceleration,
    /// Forces (N).
    Force,
    /// Impulses (N·s).
    Impulse,
    /// Mass densities (kg/m³).
    Density,
    /// Areas (m²).
    Area,
    /// Durations (s).
    Time,
    /// Absolute temperatures (K).
    Temperature,
    /// Surface roughness heights (m).
    Roughness,
    /// Altitudes above the launch site or sea level (m).
    Altitude,
    /// Ratios shown as percentages (1.0 = 100 %).
    Percentage,
    /// Lengths in units of body diameter (dimensionless ratio).
    Calibers,
    /// A plain number with no physical unit (drag coefficient, counts, exponents).
    Scalar,
}

/// A display unit of a quantity: `si = value * to_si + offset`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Unit {
    /// Text shown next to the number.
    pub symbol: &'static str,
    /// Size of one display unit in SI.
    pub to_si: f64,
    /// SI value of display zero (non-zero only for temperatures).
    pub offset: f64,
    /// Decimals shown when formatting.
    pub decimals: usize,
}

impl Unit {
    /// Converts an SI value to this unit.
    ///
    /// ```ignore
    /// assert_eq!(Quantity::Length.units()[0].from_si(1.0), 1000.0); // mm
    /// ```
    #[allow(clippy::wrong_self_convention)]
    pub fn from_si(&self, si: f64) -> f64 {
        (si - self.offset) / self.to_si
    }

    /// Converts a value shown in this unit to SI.
    #[allow(clippy::wrong_self_convention)]
    pub fn to_si(&self, shown: f64) -> f64 {
        shown * self.to_si + self.offset
    }

    /// One scrub step in display units: `10^-decimals`.
    pub fn step(&self) -> f64 {
        10f64.powi(-(self.decimals as i32))
    }
}

/// Unit systems of the top-bar selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum UnitSystem {
    /// Millimetres, grams, seconds.
    #[default]
    MmGS,
    /// Centimetres, grams, seconds.
    CmGS,
    /// Metres, kilograms, seconds.
    MKgS,
    /// Inches, ounces, seconds.
    InOzS,
    /// Feet, pounds, seconds.
    FtLbS,
}

impl UnitSystem {
    /// Every system, in selector order.
    pub const ALL: [UnitSystem; 5] = [UnitSystem::MmGS, UnitSystem::CmGS, UnitSystem::MKgS, UnitSystem::InOzS, UnitSystem::FtLbS];

    /// Selector label, e.g. `"mm · g · s"`.
    pub fn label(self) -> &'static str {
        match self {
            UnitSystem::MmGS => "mm · g · s",
            UnitSystem::CmGS => "cm · g · s",
            UnitSystem::MKgS => "m · kg · s",
            UnitSystem::InOzS => "in · oz · s",
            UnitSystem::FtLbS => "ft · lb · s",
        }
    }

    fn is_imperial(self) -> bool {
        matches!(self, UnitSystem::InOzS | UnitSystem::FtLbS)
    }
}

// Exact definitions: 1 in = 25.4 mm, 1 ft = 0.3048 m, 1 lb = 0.45359237 kg, 1 oz = lb/16 = 0.028349523125 kg,
// g0 = 9.80665 m/s², 1 lbf = lb * g0 = 4.4482216152605 N, 1 mil = 0.001 in = 25.4 µm, K = (F + 459.67) * 5/9.
const INCH: f64 = 0.0254;
const FOOT: f64 = 0.3048;
const POUND: f64 = 0.45359237;
const OUNCE: f64 = 0.028349523125;
const G0: f64 = 9.80665;
const LBF: f64 = 4.4482216152605;

const fn unit(symbol: &'static str, to_si: f64, decimals: usize) -> Unit {
    Unit { symbol, to_si, offset: 0.0, decimals }
}

static LENGTH: [Unit; 5] = [unit("mm", 0.001, 2), unit("cm", 0.01, 3), unit("m", 1.0, 4), unit("in", INCH, 3), unit("ft", FOOT, 4)];
static MASS: [Unit; 4] = [unit("g", 0.001, 1), unit("kg", 1.0, 3), unit("oz", OUNCE, 2), unit("lb", POUND, 3)];
static ANGLE: [Unit; 2] = [unit("deg", std::f64::consts::PI / 180.0, 1), unit("rad", 1.0, 3)];
static VELOCITY: [Unit; 4] = [unit("m/s", 1.0, 1), unit("km/h", 1.0 / 3.6, 1), unit("ft/s", FOOT, 1), unit("mph", 1609.344 / 3600.0, 1)];
static ACCELERATION: [Unit; 2] = [unit("g", G0, 2), unit("m/s²", 1.0, 2)];
static FORCE: [Unit; 2] = [unit("N", 1.0, 2), unit("lbf", LBF, 2)];
static IMPULSE: [Unit; 2] = [unit("N·s", 1.0, 2), unit("lbf·s", LBF, 2)];
static DENSITY: [Unit; 3] = [unit("kg/m³", 1.0, 0), unit("g/cm³", 1000.0, 3), unit("lb/ft³", POUND / (FOOT * FOOT * FOOT), 1)];
static AREA: [Unit; 4] = [unit("mm²", 1e-6, 1), unit("cm²", 1e-4, 2), unit("m²", 1.0, 4), unit("in²", INCH * INCH, 3)];
static TIME: [Unit; 3] = [unit("s", 1.0, 2), unit("ms", 0.001, 1), unit("min", 60.0, 2)];
static TEMPERATURE: [Unit; 3] = [
    Unit { symbol: "°C", to_si: 1.0, offset: 273.15, decimals: 1 },
    Unit { symbol: "K", to_si: 1.0, offset: 0.0, decimals: 1 },
    Unit { symbol: "°F", to_si: 5.0 / 9.0, offset: 459.67 * 5.0 / 9.0, decimals: 1 },
];
static ROUGHNESS: [Unit; 2] = [unit("µm", 1e-6, 1), unit("mil", 25.4e-6, 2)];
static ALTITUDE: [Unit; 3] = [unit("m", 1.0, 0), unit("ft", FOOT, 0), unit("km", 1000.0, 3)];
static PERCENTAGE: [Unit; 1] = [unit("%", 0.01, 1)];
static CALIBERS: [Unit; 1] = [unit("cal", 1.0, 2)];
static SCALAR: [Unit; 1] = [unit("·", 1.0, 2)];

impl Quantity {
    /// All selectable units, in click-cycle order; the first is the most common.
    pub fn units(self) -> &'static [Unit] {
        match self {
            Quantity::Length => &LENGTH,
            Quantity::Mass => &MASS,
            Quantity::Angle => &ANGLE,
            Quantity::Velocity => &VELOCITY,
            Quantity::Acceleration => &ACCELERATION,
            Quantity::Force => &FORCE,
            Quantity::Impulse => &IMPULSE,
            Quantity::Density => &DENSITY,
            Quantity::Area => &AREA,
            Quantity::Time => &TIME,
            Quantity::Temperature => &TEMPERATURE,
            Quantity::Roughness => &ROUGHNESS,
            Quantity::Altitude => &ALTITUDE,
            Quantity::Percentage => &PERCENTAGE,
            Quantity::Calibers => &CALIBERS,
            Quantity::Scalar => &SCALAR,
        }
    }

    /// The unit this quantity uses in `system`.
    ///
    /// Angle is always degrees, time always seconds; percentages and calibers have a single unit.
    pub fn unit_for(self, system: UnitSystem) -> &'static Unit {
        use UnitSystem::*;
        let imperial = system.is_imperial();
        let index = match self {
            Quantity::Length => match system {
                MmGS => 0,
                CmGS => 1,
                MKgS => 2,
                InOzS => 3,
                FtLbS => 4,
            },
            Quantity::Mass => match system {
                MmGS | CmGS => 0,
                MKgS => 1,
                InOzS => 2,
                FtLbS => 3,
            },
            Quantity::Velocity => {
                if imperial {
                    2
                } else {
                    0
                }
            }
            Quantity::Acceleration => usize::from(system == MKgS),
            Quantity::Force | Quantity::Impulse | Quantity::Roughness | Quantity::Altitude => usize::from(imperial),
            Quantity::Temperature => {
                if imperial {
                    2
                } else {
                    0
                }
            }
            Quantity::Density => match system {
                MmGS | CmGS => 1,
                MKgS => 0,
                InOzS | FtLbS => 2,
            },
            Quantity::Area => match system {
                MmGS => 0,
                CmGS => 1,
                MKgS => 2,
                InOzS | FtLbS => 3,
            },
            Quantity::Angle | Quantity::Time | Quantity::Percentage | Quantity::Calibers | Quantity::Scalar => 0,
        };
        let units = self.units();
        &units[index.min(units.len() - 1)]
    }

    /// Position of `unit` in [`Quantity::units`] (matched by symbol), or 0 if it is not one of this quantity's units.
    pub fn unit_index(self, unit: &Unit) -> usize {
        self.units().iter().position(|u| u.symbol == unit.symbol).unwrap_or(0)
    }

    /// The next unit in the click cycle, wrapping around after the last.
    pub fn next_unit(self, current: &Unit) -> &'static Unit {
        let units = self.units();
        &units[(self.unit_index(current) + 1) % units.len()]
    }

    /// Extra spellings accepted as unit suffixes, as `(text, index into units())`. Unit symbols are always accepted.
    fn aliases(self) -> &'static [(&'static str, usize)] {
        match self {
            Quantity::Length => &[("\"", 3), ("″", 3), ("inch", 3), ("inches", 3), ("'", 4), ("′", 4), ("foot", 4), ("feet", 4)],
            Quantity::Mass => &[("lbs", 3), ("gram", 0), ("grams", 0)],
            Quantity::Angle => &[("°", 0), ("degrees", 0), ("radian", 1), ("radians", 1)],
            Quantity::Velocity => &[("kph", 1), ("kmh", 1), ("fps", 2)],
            Quantity::Acceleration => &[("m/s2", 1)],
            Quantity::Force => &[],
            Quantity::Impulse => &[("Ns", 0), ("lbfs", 1)],
            Quantity::Density => &[("kg/m3", 0), ("g/cm3", 1), ("lb/ft3", 2), ("lbs/ft3", 2)],
            Quantity::Area => &[("mm2", 0), ("cm2", 1), ("m2", 2), ("in2", 3)],
            Quantity::Time => &[("sec", 0), ("secs", 0)],
            Quantity::Temperature => &[("C", 0), ("F", 2), ("degC", 0), ("degF", 2)],
            Quantity::Roughness => &[("um", 0), ("μm", 0), ("mils", 1)],
            Quantity::Altitude => &[("feet", 1), ("foot", 1)],
            Quantity::Percentage => &[("pct", 0)],
            Quantity::Calibers => &[("cals", 0), ("caliber", 0), ("calibers", 0)],
            Quantity::Scalar => &[],
        }
    }
}

/// Formats an SI value in the unit the quantity uses in `system`: `("152.40", "mm")`.
///
/// Values that round to zero print without a sign (no `-0.00`). Non-finite values print as `"—"`.
///
/// ```ignore
/// assert_eq!(format(0.1524, Quantity::Length, &UnitSystem::MmGS), ("152.40".to_string(), "mm"));
/// ```
pub fn format(value_si: f64, quantity: Quantity, system: &UnitSystem) -> (String, &'static str) {
    let unit = quantity.unit_for(*system);
    (format_in(value_si, unit), unit.symbol)
}

/// Formats an SI value in a specific unit: number only, `unit.decimals` decimals, no digit grouping.
pub fn format_in(value_si: f64, unit: &Unit) -> String {
    if !value_si.is_finite() {
        return "—".to_string();
    }
    let shown = unit.from_si(value_si);
    let text = format!("{:.*}", unit.decimals, shown);
    match text.strip_prefix('-') {
        Some(rest) if rest.chars().all(|c| c == '0' || c == '.') => rest.to_string(),
        _ => text,
    }
}

/// Expression AST. Numbers are in the display unit of the field being edited: the parser has already converted any
/// unit suffix into that unit.
#[derive(Debug, Clone, PartialEq)]
pub enum Expr {
    /// A literal number.
    Num(f64),
    /// A reference to another value, from a `{id}` token. The id is opaque to this module.
    Ref(String),
    /// Unary minus.
    Neg(Box<Expr>),
    /// A binary operation.
    BinOp(Box<Expr>, Op, Box<Expr>),
}

/// Binary operators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    /// Addition.
    Add,
    /// Subtraction.
    Sub,
    /// Multiplication.
    Mul,
    /// Division.
    Div,
}

/// A syntax error with the byte offset in the input where it was detected.
#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    /// User-presentable description.
    pub message: String,
    /// Byte offset into the parsed text.
    pub position: usize,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ParseError {}

/// Why an expression could not be evaluated.
#[derive(Debug, Clone, PartialEq)]
pub enum EvalError {
    /// A `{id}` reference that the resolver does not know.
    UnknownRef(String),
    /// Division by zero.
    DivideByZero,
    /// The result overflowed or was otherwise not a finite number.
    NotFinite,
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvalError::UnknownRef(id) => write!(f, "Unknown reference '{id}'"),
            EvalError::DivideByZero => f.write_str("Division by zero"),
            EvalError::NotFinite => f.write_str("Result is not a finite number"),
        }
    }
}

impl std::error::Error for EvalError {}

/// Re-expresses `expr`, a value in unit `from`, in unit `to`. Literals fold into a single number.
fn convert(expr: Expr, from: &Unit, to: &Unit) -> Expr {
    if from.to_si == to.to_si && from.offset == to.offset {
        return expr;
    }
    if let Expr::Num(v) = expr {
        return Expr::Num(to.from_si(from.to_si(v)));
    }
    let scaled = Expr::BinOp(Box::new(expr), Op::Mul, Box::new(Expr::Num(from.to_si / to.to_si)));
    let shift = (from.offset - to.offset) / to.to_si;
    if shift == 0.0 {
        scaled
    } else {
        Expr::BinOp(Box::new(scaled), Op::Add, Box::new(Expr::Num(shift)))
    }
}

/// True for characters that continue a unit word (so `3 meters` is not read as `3 m` followed by `eters`).
fn is_unit_letter(c: char) -> bool {
    c.is_alphabetic() || c == '°'
}

struct Parser<'a> {
    src: &'a str,
    pos: usize,
    quantity: Quantity,
    unit: &'a Unit,
}

impl Parser<'_> {
    fn err<T>(&self, message: String, position: usize) -> Result<T, ParseError> {
        Err(ParseError { message, position })
    }

    fn rest(&self) -> &str {
        &self.src[self.pos..]
    }

    fn peek(&self) -> Option<char> {
        self.rest().chars().next()
    }

    fn bump(&mut self) {
        if let Some(c) = self.peek() {
            self.pos += c.len_utf8();
        }
    }

    fn skip_ws(&mut self) {
        while self.peek().is_some_and(char::is_whitespace) {
            self.bump();
        }
    }

    fn unexpected<T>(&self) -> Result<T, ParseError> {
        match self.peek() {
            Some(c) => self.err(format!("Unexpected '{c}' at position {}", self.pos), self.pos),
            None => self.err("Unexpected end of expression".to_string(), self.src.len()),
        }
    }

    fn expr(&mut self) -> Result<Expr, ParseError> {
        let mut lhs = self.term()?;
        loop {
            self.skip_ws();
            let op = match self.peek() {
                Some('+') => Op::Add,
                Some('-' | '−') => Op::Sub,
                _ => return Ok(lhs),
            };
            self.bump();
            let rhs = self.term()?;
            lhs = Expr::BinOp(Box::new(lhs), op, Box::new(rhs));
        }
    }

    fn term(&mut self) -> Result<Expr, ParseError> {
        let mut lhs = self.unary()?;
        loop {
            self.skip_ws();
            let op = match self.peek() {
                Some('*' | '×') => Op::Mul,
                Some('/' | '÷') => Op::Div,
                _ => return Ok(lhs),
            };
            self.bump();
            let rhs = self.unary()?;
            lhs = Expr::BinOp(Box::new(lhs), op, Box::new(rhs));
        }
    }

    fn unary(&mut self) -> Result<Expr, ParseError> {
        self.skip_ws();
        match self.peek() {
            Some('-' | '−') => {
                self.bump();
                Ok(Expr::Neg(Box::new(self.unary()?)))
            }
            Some('+') => {
                self.bump();
                self.unary()
            }
            _ => self.primary(),
        }
    }

    fn primary(&mut self) -> Result<Expr, ParseError> {
        self.skip_ws();
        match self.peek() {
            Some(c) if c.is_ascii_digit() || c == '.' => self.number_with_units(),
            Some('(') => {
                let open = self.pos;
                self.bump();
                let inner = self.expr()?;
                self.skip_ws();
                if self.peek() != Some(')') {
                    return self.err("Missing ')'".to_string(), self.src.len().max(open));
                }
                self.bump();
                match self.suffix()? {
                    Some(index) => Ok(convert(inner, &self.quantity.units()[index], self.unit)),
                    None => Ok(inner),
                }
            }
            Some('{') => {
                let open = self.pos;
                let Some(len) = self.rest().find('}') else {
                    return self.err("Unclosed '{'".to_string(), open);
                };
                let id = self.rest()[1..len].trim().to_string();
                if id.is_empty() {
                    return self.err(format!("Empty reference at position {open}"), open);
                }
                self.pos += len + 1;
                Ok(Expr::Ref(id))
            }
            _ => self.unexpected(),
        }
    }

    /// A number, optionally with a unit suffix, optionally followed by more suffixed numbers that add (`5ft 3in`).
    fn number_with_units(&mut self) -> Result<Expr, ParseError> {
        let first = self.number()?;
        let Some(index) = self.suffix()? else {
            return Ok(Expr::Num(first));
        };
        let units = self.quantity.units();
        let mut total = convert(Expr::Num(first), &units[index], self.unit);
        loop {
            let save = self.pos;
            self.skip_ws();
            if !self.peek().is_some_and(|c| c.is_ascii_digit() || c == '.') {
                self.pos = save;
                return Ok(total);
            }
            let start = self.pos;
            let value = self.number()?;
            let Some(index) = self.suffix()? else {
                return self.err(format!("Missing unit for '{}' at position {start}", &self.src[start..self.pos]), start);
            };
            let part = convert(Expr::Num(value), &units[index], self.unit);
            total = Expr::BinOp(Box::new(total), Op::Add, Box::new(part));
        }
    }

    fn number(&mut self) -> Result<f64, ParseError> {
        let start = self.pos;
        let bytes = self.src.as_bytes();
        let mut end = start;
        let mut digits = 0;
        while end < bytes.len() && bytes[end].is_ascii_digit() {
            end += 1;
            digits += 1;
        }
        if end < bytes.len() && bytes[end] == b'.' {
            end += 1;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
                digits += 1;
            }
        }
        if digits == 0 {
            return self.unexpected();
        }
        if end < bytes.len() && (bytes[end] == b'e' || bytes[end] == b'E') {
            let mut e = end + 1;
            if e < bytes.len() && (bytes[e] == b'+' || bytes[e] == b'-') {
                e += 1;
            }
            if e < bytes.len() && bytes[e].is_ascii_digit() {
                while e < bytes.len() && bytes[e].is_ascii_digit() {
                    e += 1;
                }
                end = e;
            }
        }
        let text = &self.src[start..end];
        match text.parse::<f64>() {
            Ok(v) if v.is_finite() => {
                self.pos = end;
                Ok(v)
            }
            _ => self.err(format!("Number '{text}' out of range at position {start}"), start),
        }
    }

    /// Parses an optional unit suffix (after optional whitespace) and returns its index in `quantity.units()`.
    /// Consumes nothing when there is no suffix. Letters that are not a unit of this quantity are an error.
    fn suffix(&mut self) -> Result<Option<usize>, ParseError> {
        let save = self.pos;
        self.skip_ws();
        let start = self.pos;
        let rest = &self.src[start..];
        let Some(first) = rest.chars().next() else {
            self.pos = save;
            return Ok(None);
        };
        let mut found = self.match_unit(rest, false).or_else(|| self.match_unit(rest, true));
        if let Some((len, _)) = found {
            // Whole-word match: `3 meters` must not read as `3 m` + `eters`.
            if rest[..len].chars().next_back().is_some_and(is_unit_letter) && rest[len..].chars().next().is_some_and(is_unit_letter) {
                found = None;
            }
        }
        if let Some((len, index)) = found {
            self.pos = start + len;
            return Ok(Some(index));
        }
        if is_unit_letter(first) || first == 'µ' {
            let len = rest.find(|c: char| !(is_unit_letter(c) || c == 'µ')).unwrap_or(rest.len());
            return self.err(format!("Unknown unit '{}' for {:?}", &rest[..len], self.quantity), start);
        }
        if matches!(first, '"' | '\'' | '%' | '″' | '′') {
            return self.err(format!("Unknown unit '{first}' for {:?}", self.quantity), start);
        }
        self.pos = save;
        Ok(None)
    }

    /// Longest unit symbol or alias that `rest` starts with, as `(byte length, unit index)`.
    fn match_unit(&self, rest: &str, ignore_case: bool) -> Option<(usize, usize)> {
        let starts = |candidate: &str| -> bool {
            if ignore_case {
                let mut chars = rest.chars();
                candidate.chars().all(|c| chars.next().is_some_and(|r| r.to_lowercase().eq(c.to_lowercase())))
            } else {
                rest.starts_with(candidate)
            }
        };
        let byte_len = |candidate: &str| -> usize {
            if ignore_case {
                rest.chars().take(candidate.chars().count()).map(char::len_utf8).sum()
            } else {
                candidate.len()
            }
        };
        let symbols = self.quantity.units().iter().enumerate().map(|(i, u)| (u.symbol, i));
        let aliases = self.quantity.aliases().iter().copied();
        symbols.chain(aliases).filter(|(text, _)| starts(text)).map(|(text, i)| (byte_len(text), i)).max_by_key(|&(len, _)| len)
    }
}

/// Parses `text` into an [`Expr`] for a field that shows `quantity` in `unit`.
///
/// Supports `+ - * /` (also `× ÷ −`), parentheses, unary minus, decimals (`.5`, `1e3`), `{ref}` tokens, and unit
/// suffixes of the quantity's units (plus aliases such as `"` and `'` for inches and feet, `lbs`, `°`). A suffixed number
/// is converted into `unit` (`3in` with `unit` = mm becomes `Num(76.2)`); juxtaposed suffixed numbers add
/// (`1' 2"` = 14 in); a suffix after a parenthesised group converts the whole group (`(2+3)in`). Unsuffixed numbers
/// are already in `unit`. There is no exponentiation operator.
///
/// ```ignore
/// let mm = &Quantity::Length.units()[0];
/// assert_eq!(parse_expr("2+3*4", Quantity::Length, mm).map(|e| eval(&e, &|_| None)), Ok(Ok(14.0)));
/// ```
pub fn parse_expr(text: &str, quantity: Quantity, unit: &Unit) -> Result<Expr, ParseError> {
    if text.trim().is_empty() {
        return Err(ParseError { message: "Empty expression".to_string(), position: 0 });
    }
    let mut parser = Parser { src: text, pos: 0, quantity, unit };
    let expr = parser.expr()?;
    parser.skip_ws();
    if parser.pos < text.len() {
        return parser.unexpected();
    }
    Ok(expr)
}

/// Evaluates `expr`. References are resolved by `resolve`, which returns the referenced value already in the display unit
/// of the field being edited.
///
/// Fails on an unresolved reference, division by zero, or a non-finite result.
pub fn eval(expr: &Expr, resolve: &dyn Fn(&str) -> Option<f64>) -> Result<f64, EvalError> {
    let value = match expr {
        Expr::Num(v) => *v,
        Expr::Ref(id) => resolve(id).ok_or_else(|| EvalError::UnknownRef(id.clone()))?,
        Expr::Neg(inner) => -eval(inner, resolve)?,
        Expr::BinOp(lhs, op, rhs) => {
            let a = eval(lhs, resolve)?;
            let b = eval(rhs, resolve)?;
            match op {
                Op::Add => a + b,
                Op::Sub => a - b,
                Op::Mul => a * b,
                Op::Div if b == 0.0 => return Err(EvalError::DivideByZero),
                Op::Div => a / b,
            }
        }
    };
    if value.is_finite() {
        Ok(value)
    } else {
        Err(EvalError::NotFinite)
    }
}

/// All references in `expr`, in order of appearance, without duplicates (for dependency tracking and cycle detection).
pub fn refs(expr: &Expr) -> Vec<String> {
    fn walk(expr: &Expr, out: &mut Vec<String>) {
        match expr {
            Expr::Num(_) => {}
            Expr::Ref(id) => {
                if !out.contains(id) {
                    out.push(id.clone());
                }
            }
            Expr::Neg(inner) => walk(inner, out),
            Expr::BinOp(lhs, _, rhs) => {
                walk(lhs, out);
                walk(rhs, out);
            }
        }
    }
    let mut out = Vec::new();
    walk(expr, &mut out);
    out
}

/// One-shot: parses and evaluates `text` for a field showing `unit`, returning the SI value.
///
/// `resolve_si` supplies referenced values in SI; they are converted into `unit` before the arithmetic. Errors are
/// user-presentable strings (the `Display` of [`ParseError`] or [`EvalError`]).
///
/// ```ignore
/// let mm = &Quantity::Length.units()[0];
/// assert_eq!(parse_quantity("76.2", Quantity::Length, mm, &|_| None), Ok(0.0762));
/// ```
pub fn parse_quantity(text: &str, quantity: Quantity, unit: &Unit, resolve_si: &dyn Fn(&str) -> Option<f64>) -> Result<f64, String> {
    let expr = parse_expr(text, quantity, unit).map_err(|e| e.to_string())?;
    let shown = eval(&expr, &|id| resolve_si(id).map(|si| unit.from_si(si))).map_err(|e| e.to_string())?;
    Ok(unit.to_si(shown))
}

/// The kind of value a formula can compute for a field showing `quantity` (`None` for the quantities no dimension has).
pub fn formula_kind(quantity: Quantity) -> Option<bowshock_model::Kind> {
    match quantity {
        Quantity::Length => Some(bowshock_model::Kind::Length),
        Quantity::Angle => Some(bowshock_model::Kind::Angle),
        Quantity::Scalar => Some(bowshock_model::Kind::Scalar),
        _ => None,
    }
}

/// Parses `text` typed in a field showing `quantity` in `unit` as a document formula: references `{name#id.key}` bind to part
/// ids and the numbers are converted to SI (a factor stays a factor, a length added to a reference becomes metres).
/// Errors are user-presentable.
pub fn parse_formula(text: &str, quantity: Quantity, unit: &Unit) -> Result<bowshock_model::Expr, String> {
    use bowshock_model::{Expr as Model, FieldRef, Op as ModelOp};
    fn convert(e: &Expr) -> Result<Model, String> {
        Ok(match e {
            Expr::Num(v) => Model::Num(*v),
            Expr::Ref(token) => Model::Ref(FieldRef::from_token(token).ok_or_else(|| format!("Unknown reference '{token}'"))?.0),
            Expr::Neg(inner) => Model::Neg(Box::new(convert(inner)?)),
            Expr::BinOp(l, op, r) => {
                let op = match op {
                    Op::Add => ModelOp::Add,
                    Op::Sub => ModelOp::Sub,
                    Op::Mul => ModelOp::Mul,
                    Op::Div => ModelOp::Div,
                };
                Model::Bin(Box::new(convert(l)?), op, Box::new(convert(r)?))
            }
        })
    }
    let kind = formula_kind(quantity).ok_or_else(|| "This value cannot be a formula".to_string())?;
    let parsed = parse_expr(text, quantity, unit).map_err(|e| e.to_string())?;
    convert(&parsed)?.scaled(kind, unit.to_si).map_err(|e| e.to_string())
}

/// The text of a stored formula as it is edited in a field showing `quantity` in `unit`: references as tokens with the name
/// `name(part)` gives, numbers in `unit`. `None` if the formula does not fit the quantity.
pub fn formula_text(expr: &bowshock_model::Expr, quantity: Quantity, unit: &Unit, name: &dyn Fn(bowshock_model::PartId) -> Option<String>) -> Option<String> {
    let kind = formula_kind(quantity)?;
    Some(expr.scaled(kind, 1.0 / unit.to_si).ok()?.to_text(name))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_QUANTITIES: [Quantity; 15] = [
        Quantity::Length,
        Quantity::Mass,
        Quantity::Angle,
        Quantity::Velocity,
        Quantity::Acceleration,
        Quantity::Force,
        Quantity::Impulse,
        Quantity::Density,
        Quantity::Area,
        Quantity::Time,
        Quantity::Temperature,
        Quantity::Roughness,
        Quantity::Altitude,
        Quantity::Percentage,
        Quantity::Calibers,
    ];

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * b.abs().max(1.0)
    }

    fn u(q: Quantity, symbol: &str) -> &'static Unit {
        q.units().iter().find(|u| u.symbol == symbol).expect("unit exists")
    }

    fn none(_: &str) -> Option<f64> {
        None
    }

    fn pq(text: &str, q: Quantity, symbol: &str) -> Result<f64, String> {
        parse_quantity(text, q, u(q, symbol), &none)
    }

    fn assert_pq(text: &str, q: Quantity, symbol: &str, expected: f64) {
        let got = pq(text, q, symbol).unwrap_or_else(|e| panic!("{text:?} failed: {e}"));
        assert!(close(got, expected), "{text:?}: got {got}, expected {expected}");
    }

    fn err_of(text: &str, q: Quantity, symbol: &str) -> ParseError {
        parse_expr(text, q, u(q, symbol)).expect_err("should fail to parse")
    }

    #[test]
    fn round_trip_every_unit() {
        for q in ALL_QUANTITIES {
            for unit in q.units() {
                for shown in [0.0, 1.0, -3.5, 123.456, 1e4] {
                    let back = unit.from_si(unit.to_si(shown));
                    assert!(close(back, shown), "{:?} {}: {shown} -> {back}", q, unit.symbol);
                }
            }
        }
    }

    #[test]
    fn unit_constants() {
        // 1 in = 25.4 mm exactly; 1 ft = 12 in.
        assert!(close(u(Quantity::Length, "in").to_si(1.0), 0.0254));
        assert!(close(u(Quantity::Length, "ft").to_si(1.0), 12.0 * 0.0254));
        // 1 lb = 0.45359237 kg; 1 oz = 1/16 lb.
        assert!(close(u(Quantity::Mass, "lb").to_si(1.0), 0.45359237));
        assert!(close(u(Quantity::Mass, "oz").to_si(16.0), 0.45359237));
        // g0 = 9.80665 m/s² (standard gravity).
        assert_eq!(u(Quantity::Acceleration, "g").to_si(1.0), 9.80665);
        // 1 lbf = 0.45359237 kg * 9.80665 m/s².
        assert!(close(u(Quantity::Force, "lbf").to_si(1.0), 0.45359237 * 9.80665));
        assert!(close(u(Quantity::Force, "lbf").to_si(1.0), 4.4482216152605));
        // 1 mil = 25.4 µm.
        assert!(close(u(Quantity::Roughness, "mil").to_si(1.0), 25.4e-6));
        // 1 mph = 0.44704 m/s; 1 km/h = 1/3.6 m/s.
        assert!(close(u(Quantity::Velocity, "mph").to_si(1.0), 0.44704));
        assert!(close(u(Quantity::Velocity, "km/h").to_si(36.0), 10.0));
        // 1 lb/ft³ = 16.018463... kg/m³.
        assert!(close(u(Quantity::Density, "lb/ft³").to_si(1.0), 16.018463373960138));
        // 1 g/cm³ = 1000 kg/m³.
        assert!(close(u(Quantity::Density, "g/cm³").to_si(1.0), 1000.0));
        assert!(close(u(Quantity::Area, "in²").to_si(1.0), 0.00064516));
        assert!(close(u(Quantity::Angle, "deg").to_si(180.0), std::f64::consts::PI));
        assert!(close(u(Quantity::Percentage, "%").to_si(50.0), 0.5));
    }

    #[test]
    fn temperature_conversions() {
        let c = u(Quantity::Temperature, "°C");
        let k = u(Quantity::Temperature, "K");
        let f = u(Quantity::Temperature, "°F");
        assert!(close(c.to_si(0.0), 273.15));
        assert!(close(f.to_si(212.0), 373.15));
        assert!(close(f.to_si(32.0), 273.15));
        // K = (F + 459.67) * 5/9
        assert!(close(f.to_si(-459.67), 0.0));
        assert!(close(k.to_si(300.0), 300.0));
        assert!(close(c.from_si(373.15), 100.0));
        assert!(close(f.from_si(273.15), 32.0));
        assert_eq!(format_in(273.15, c), "0.0");
        assert_eq!(format_in(373.15, f), "212.0");
        // A Fahrenheit suffix typed in a Celsius field: 100 °F = 37.8 °C.
        let v = parse_quantity("100°F", Quantity::Temperature, c, &none).expect("parses");
        assert_eq!(format_in(v, c), "37.8");
        assert_pq("20°C", Quantity::Temperature, "K", 293.15);
        assert_pq("300 K", Quantity::Temperature, "°C", 300.0);
    }

    #[test]
    fn format_examples() {
        let s = UnitSystem::MmGS;
        assert_eq!(format(0.1524, Quantity::Length, &s), ("152.40".to_string(), "mm"));
        assert_eq!(format_in(0.0762, u(Quantity::Length, "in")), "3.000");
        assert_eq!(format_in(1.284, u(Quantity::Mass, "kg")), "1.284");
        assert_eq!(format_in(std::f64::consts::FRAC_PI_6, u(Quantity::Angle, "deg")), "30.0");
        assert_eq!(format(1.92, Quantity::Calibers, &s), ("1.92".to_string(), "cal"));
        assert_eq!(format_in(0.7, u(Quantity::Percentage, "%")), "70.0");
        assert_eq!(format_in(1000.0, u(Quantity::Altitude, "ft")), "3281");
        assert_eq!(format_in(1000.0, u(Quantity::Altitude, "km")), "1.000");
        assert_eq!(format_in(2700.0, u(Quantity::Density, "kg/m³")), "2700");
    }

    #[test]
    fn format_edge_cases() {
        let mm = u(Quantity::Length, "mm");
        assert_eq!(format_in(-0.0, mm), "0.00");
        assert_eq!(format_in(-1e-9, mm), "0.00");
        assert_eq!(format_in(-0.0015, mm), "-1.50");
        assert_eq!(format_in(f64::NAN, mm), "—");
        assert_eq!(format_in(f64::INFINITY, mm), "—");
        assert_eq!(format(f64::NAN, Quantity::Mass, &UnitSystem::MKgS), ("—".to_string(), "kg"));
        assert_eq!(format_in(1234.5678, u(Quantity::Length, "m")), "1234.5678");
    }

    #[test]
    fn step_is_power_of_ten() {
        assert!(close(u(Quantity::Length, "mm").step(), 0.01));
        assert!(close(u(Quantity::Length, "in").step(), 0.001));
        assert!(close(u(Quantity::Angle, "deg").step(), 0.1));
        assert_eq!(u(Quantity::Density, "kg/m³").step(), 1.0);
        assert_eq!(u(Quantity::Altitude, "m").step(), 1.0);
    }

    #[test]
    fn next_unit_cycles_and_wraps() {
        let units = Quantity::Length.units();
        let mut unit = &units[0];
        let mut seen = Vec::new();
        for _ in 0..units.len() {
            seen.push(unit.symbol);
            unit = Quantity::Length.next_unit(unit);
        }
        assert_eq!(seen, ["mm", "cm", "m", "in", "ft"]);
        assert_eq!(unit.symbol, "mm");
        for q in ALL_QUANTITIES {
            let last = q.units().last().expect("non-empty");
            assert_eq!(q.next_unit(last).symbol, q.units()[0].symbol);
            assert_eq!(q.unit_index(last), q.units().len() - 1);
        }
        // Not found: index 0.
        assert_eq!(Quantity::Mass.unit_index(u(Quantity::Length, "ft")), 0);
        assert_eq!(Quantity::Percentage.next_unit(&Quantity::Percentage.units()[0]).symbol, "%");
    }

    #[test]
    fn unit_orders_and_decimals() {
        let syms = |q: Quantity| q.units().iter().map(|u| u.symbol).collect::<Vec<_>>();
        assert_eq!(syms(Quantity::Mass), ["g", "kg", "oz", "lb"]);
        assert_eq!(syms(Quantity::Velocity), ["m/s", "km/h", "ft/s", "mph"]);
        assert_eq!(syms(Quantity::Temperature), ["°C", "K", "°F"]);
        assert_eq!(syms(Quantity::Altitude), ["m", "ft", "km"]);
        let dec = |q: Quantity| q.units().iter().map(|u| u.decimals).collect::<Vec<_>>();
        assert_eq!(dec(Quantity::Length), [2, 3, 4, 3, 4]);
        assert_eq!(dec(Quantity::Mass), [1, 3, 2, 3]);
        assert_eq!(dec(Quantity::Angle), [1, 3]);
        assert_eq!(dec(Quantity::Density), [0, 3, 1]);
        assert_eq!(dec(Quantity::Area), [1, 2, 4, 3]);
        assert_eq!(dec(Quantity::Time), [2, 1, 2]);
        assert_eq!(dec(Quantity::Altitude), [0, 0, 3]);
        assert_eq!(dec(Quantity::Roughness), [1, 2]);
    }

    #[test]
    fn unit_for_each_system() {
        let sym = |q: Quantity, s: UnitSystem| q.unit_for(s).symbol;
        use Quantity::*;
        use UnitSystem::*;
        let expect: [(UnitSystem, [&str; 11]); 5] = [
            (MmGS, ["mm", "g", "m/s", "g", "N", "N·s", "g/cm³", "mm²", "µm", "m", "°C"]),
            (CmGS, ["cm", "g", "m/s", "g", "N", "N·s", "g/cm³", "cm²", "µm", "m", "°C"]),
            (MKgS, ["m", "kg", "m/s", "m/s²", "N", "N·s", "kg/m³", "m²", "µm", "m", "°C"]),
            (InOzS, ["in", "oz", "ft/s", "g", "lbf", "lbf·s", "lb/ft³", "in²", "mil", "ft", "°F"]),
            (FtLbS, ["ft", "lb", "ft/s", "g", "lbf", "lbf·s", "lb/ft³", "in²", "mil", "ft", "°F"]),
        ];
        for (system, want) in expect {
            let got = [
                sym(Length, system),
                sym(Mass, system),
                sym(Velocity, system),
                sym(Acceleration, system),
                sym(Force, system),
                sym(Impulse, system),
                sym(Density, system),
                sym(Area, system),
                sym(Roughness, system),
                sym(Altitude, system),
                sym(Temperature, system),
            ];
            assert_eq!(got, want, "{system:?}");
            assert_eq!(sym(Angle, system), "deg");
            assert_eq!(sym(Time, system), "s");
            assert_eq!(sym(Percentage, system), "%");
            assert_eq!(sym(Calibers, system), "cal");
        }
        // Every system/quantity resolves to a member of units().
        for system in UnitSystem::ALL {
            for q in ALL_QUANTITIES {
                assert!(q.units().contains(q.unit_for(system)));
            }
        }
    }

    #[test]
    fn system_labels() {
        assert_eq!(UnitSystem::default(), UnitSystem::MmGS);
        let labels: Vec<_> = UnitSystem::ALL.iter().map(|s| s.label()).collect();
        assert_eq!(labels, ["mm · g · s", "cm · g · s", "m · kg · s", "in · oz · s", "ft · lb · s"]);
        assert!(labels[0].contains('\u{00B7}'));
    }

    #[test]
    fn parse_with_units() {
        use Quantity::*;
        assert_pq("3in", Length, "mm", 0.0762);
        assert_pq("3 in", Length, "mm", 0.0762);
        assert_pq("76.2mm", Length, "mm", 0.0762);
        assert_pq("76.2", Length, "mm", 0.0762);
        assert_pq("1' 2\"", Length, "mm", 0.3556);
        assert_pq("1'2\"", Length, "mm", 0.3556);
        assert_pq("5ft 3in", Length, "mm", 5.25 * 0.3048);
        assert_pq("5ft3in", Length, "in", 63.0 * 0.0254);
        assert_pq("5.5 cm", Length, "in", 0.055);
        assert_pq("250 g", Mass, "kg", 0.25);
        assert_pq("250g", Mass, "g", 0.25);
        assert_pq("2 lbs", Mass, "g", 2.0 * 0.45359237);
        assert_pq("1lb 4oz", Mass, "kg", 1.25 * 0.45359237);
        assert_pq("30°", Angle, "deg", 30.0 * std::f64::consts::PI / 180.0);
        assert_pq("30 deg", Angle, "rad", 30.0 * std::f64::consts::PI / 180.0);
        assert_pq("1.5rad", Angle, "deg", 1.5);
        assert_pq("100 km/h", Velocity, "m/s", 100.0 / 3.6);
        assert_pq("10 m/s", Velocity, "ft/s", 10.0);
        assert_pq("60mph", Velocity, "m/s", 26.8224);
        assert_pq("2 g", Acceleration, "m/s²", 2.0 * 9.80665);
        assert_pq("10 lbf", Force, "N", 10.0 * 4.4482216152605);
        assert_pq("2.7 g/cm³", Density, "kg/m³", 2700.0);
        assert_pq("2700 kg/m3", Density, "g/cm³", 2700.0);
        assert_pq("1 m²", Area, "mm²", 1.0);
        assert_pq("500 ms", Time, "s", 0.5);
        assert_pq("2min", Time, "s", 120.0);
        assert_pq("5 mil", Roughness, "µm", 127e-6);
        assert_pq("40 µm", Roughness, "mil", 40e-6);
        assert_pq("3 um", Roughness, "µm", 3e-6);
        assert_pq("1000 ft", Altitude, "m", 304.8);
        assert_pq("1 km", Altitude, "ft", 1000.0);
        assert_pq("25%", Percentage, "%", 0.25);
        assert_pq("2.5 cal", Calibers, "cal", 2.5);
    }

    #[test]
    fn parse_case_insensitive_fallback() {
        assert_pq("3 IN", Quantity::Length, "mm", 0.0762);
        assert_pq("3 MM", Quantity::Length, "m", 0.003);
        assert_pq("10 n", Quantity::Force, "N", 10.0);
    }

    #[test]
    fn parse_arithmetic() {
        use Quantity::*;
        assert_pq("(120-50)/2", Length, "mm", 0.035);
        assert_pq("2*38", Length, "mm", 0.076);
        assert_pq("-5", Length, "mm", -0.005);
        assert_pq("2+3*4", Length, "m", 14.0);
        assert_pq("(2+3)*4", Length, "m", 20.0);
        assert_pq("((1+2)*(3+4))", Length, "m", 21.0);
        assert_pq("--3", Length, "m", 3.0);
        assert_pq("-(-3)", Length, "m", 3.0);
        assert_pq("2*-3", Length, "m", -6.0);
        assert_pq("+4", Length, "m", 4.0);
        assert_pq("10-4-3", Length, "m", 3.0);
        assert_pq("100/5/2", Length, "m", 10.0);
        assert_pq(".5+1e3", Length, "m", 1000.5);
        assert_pq("1E-2", Length, "m", 0.01);
        assert!(pq("2e", Length, "m").is_err());
    }

    #[test]
    fn parse_unicode_operators() {
        assert_pq("6×7", Quantity::Length, "m", 42.0);
        assert_pq("84÷2", Quantity::Length, "m", 42.0);
        assert_pq("50−8", Quantity::Length, "m", 42.0);
    }

    #[test]
    fn parse_group_suffix() {
        assert_pq("(2+3)in", Quantity::Length, "mm", 0.127);
        assert_pq("(120-50)/2 mm", Quantity::Length, "mm", 0.035);
        assert_pq("(120-50)/(2 mm)", Quantity::Length, "mm", 0.035);
        assert_pq("2*(1+1)ft", Quantity::Length, "in", 4.0 * 0.3048);
        // Offset units applied to a group: (50+50)°F in °C display.
        assert_pq("(50+50)°F", Quantity::Temperature, "°C", 310.9277777777778);
    }

    #[test]
    fn parse_refs() {
        let resolve = |id: &str| match id {
            "body.od" => Some(0.0762),
            "x y" => Some(0.01),
            _ => None,
        };
        let mm = u(Quantity::Length, "mm");
        let got = parse_quantity("0.5*{body.od}+2", Quantity::Length, mm, &resolve).expect("parses");
        assert!(close(got, 0.5 * 0.0762 + 0.002), "{got}");
        let got = parse_quantity("{ x y }*2", Quantity::Length, mm, &resolve).expect("parses");
        assert!(close(got, 0.02));
        let err = parse_quantity("{body_tube#3.od}", Quantity::Length, mm, &resolve).expect_err("unknown ref");
        assert_eq!(err, "Unknown reference 'body_tube#3.od'");
        // A ref in inches display: the resolver's SI value is converted into the field's unit first.
        let inch = u(Quantity::Length, "in");
        let got = parse_quantity("{body.od}-1in", Quantity::Length, inch, &resolve).expect("parses");
        assert!(close(got, 0.0762 - 0.0254));
    }

    #[test]
    fn parse_errors() {
        use Quantity::*;
        let e = err_of("2*", Length, "mm");
        assert_eq!(e.message, "Unexpected end of expression");
        assert_eq!(e.position, 2);
        let e = err_of("1 + * 3", Length, "mm");
        assert_eq!(e.message, "Unexpected '*' at position 4");
        assert_eq!(e.position, 4);
        let e = err_of("(1+2", Length, "mm");
        assert_eq!(e.message, "Missing ')'");
        let e = err_of("1+2)", Length, "mm");
        assert_eq!(e.message, "Unexpected ')' at position 3");
        let e = err_of("3 foo", Length, "mm");
        assert_eq!(e.message, "Unknown unit 'foo' for Length");
        assert_eq!(e.position, 2);
        let e = err_of("3 meters", Length, "mm");
        assert_eq!(e.message, "Unknown unit 'meters' for Length");
        let e = err_of("3 kg", Length, "mm");
        assert_eq!(e.message, "Unknown unit 'kg' for Length");
        let e = err_of("5\"", Mass, "g");
        assert_eq!(e.message, "Unknown unit '\"' for Mass");
        let e = err_of("{", Length, "mm");
        assert_eq!(e.message, "Unclosed '{'");
        assert_eq!(e.position, 0);
        let e = err_of("1+{abc", Length, "mm");
        assert_eq!(e.message, "Unclosed '{'");
        assert_eq!(e.position, 2);
        let e = err_of("{  }", Length, "mm");
        assert_eq!(e.message, "Empty reference at position 0");
        assert_eq!(err_of("", Length, "mm").message, "Empty expression");
        assert_eq!(err_of("   ", Length, "mm").message, "Empty expression");
        assert_eq!(err_of("3 4", Length, "mm").message, "Unexpected '4' at position 2");
        assert_eq!(err_of("1ft 2", Length, "mm").message, "Missing unit for '2' at position 4");
        assert_eq!(err_of(".", Length, "mm").message, "Unexpected '.' at position 0");
        assert_eq!(err_of("2^3", Length, "mm").message, "Unexpected '^' at position 1");
        assert!(err_of("1e999", Length, "mm").message.starts_with("Number '1e999' out of range"));
        // Positions are byte offsets: '×' is two bytes.
        assert_eq!(err_of("×", Length, "mm").position, 0);
        assert_eq!(err_of("2××", Length, "mm").position, 3);
    }

    #[test]
    fn eval_errors() {
        let mm = u(Quantity::Length, "mm");
        assert_eq!(pq("1/0", Quantity::Length, "mm"), Err("Division by zero".to_string()));
        assert_eq!(pq("1/(2-2)", Quantity::Length, "mm"), Err("Division by zero".to_string()));
        assert_eq!(pq("1e300*1e300", Quantity::Length, "mm"), Err("Result is not a finite number".to_string()));
        let expr = parse_expr("{a}+1", Quantity::Length, mm).expect("parses");
        assert_eq!(eval(&expr, &|_| None), Err(EvalError::UnknownRef("a".to_string())));
        assert_eq!(eval(&expr, &|id| (id == "a").then_some(2.0)), Ok(3.0));
        assert_eq!(EvalError::DivideByZero.to_string(), "Division by zero");
        assert_eq!(pq("(", Quantity::Length, "mm"), Err("Unexpected end of expression".to_string()));
    }

    #[test]
    fn refs_dedupe_and_order() {
        let mm = u(Quantity::Length, "mm");
        let expr = parse_expr("{b}*{a} + {b} - -{c} / ({a}+1)", Quantity::Length, mm).expect("parses");
        assert_eq!(refs(&expr), ["b", "a", "c"]);
        assert!(refs(&parse_expr("1+2", Quantity::Length, mm).expect("parses")).is_empty());
    }

    #[test]
    fn ast_shape() {
        let mm = u(Quantity::Length, "mm");
        match parse_expr("3in", Quantity::Length, mm).expect("parses") {
            Expr::Num(v) => assert!(close(v, 76.2), "{v}"),
            other => panic!("expected Num, got {other:?}"),
        }
        assert_eq!(
            parse_expr("2+3*4", Quantity::Length, mm).expect("parses"),
            Expr::BinOp(Box::new(Expr::Num(2.0)), Op::Add, Box::new(Expr::BinOp(Box::new(Expr::Num(3.0)), Op::Mul, Box::new(Expr::Num(4.0)))))
        );
        assert_eq!(parse_expr("-{x}", Quantity::Length, mm).expect("parses"), Expr::Neg(Box::new(Expr::Ref("x".to_string()))));
        // Same-unit suffix is a no-op.
        assert_eq!(parse_expr("5mm", Quantity::Length, mm).expect("parses"), Expr::Num(5.0));
    }

    #[test]
    fn a_typed_formula_is_stored_in_si_and_reads_back_in_the_field_unit() {
        use bowshock_model::{Expr as Model, FieldRef, Op as ModelOp};
        let mm = u(Quantity::Length, "mm");
        let od = FieldRef::new(3, bowshock_model::Dim::Diameter);
        let model = parse_formula("0.5 * {body_tube#3.od} + 2", Quantity::Length, mm).expect("parses");
        // The factor stays a factor; the 2 mm added to a length is metres.
        let expected =
            Model::Bin(Box::new(Model::Bin(Box::new(Model::Num(0.5)), ModelOp::Mul, Box::new(Model::Ref(od)))), ModelOp::Add, Box::new(Model::Num(0.002)));
        assert_eq!(model, expected);
        let name = |_| Some("Body tube".to_string());
        let text = formula_text(&model, Quantity::Length, mm, &name).expect("fits a length");
        assert_eq!(text, "0.5 * {body_tube#3.od} + 2");
        // The same formula in inches: the 2 mm shows as 0.0787… in, and typing it back gives the same metres.
        let inch = u(Quantity::Length, "in");
        let in_text = formula_text(&model, Quantity::Length, inch, &name).expect("fits a length");
        let back = parse_formula(&in_text, Quantity::Length, inch).expect("parses");
        match back {
            Model::Bin(_, ModelOp::Add, r) => assert!(matches!(*r, Model::Num(v) if (v - 0.002).abs() < 1e-12), "{in_text}"),
            other => panic!("{other:?}"),
        }
        // A unit on a number beside a reference counts: `{od} + 1in` is 25.4 mm more.
        let model = parse_formula("{body_tube#3.od} + 1in", Quantity::Length, mm).expect("parses");
        assert!(matches!(&model, Model::Bin(_, ModelOp::Add, r) if matches!(**r, Model::Num(v) if (v - 0.0254).abs() < 1e-12)), "{model:?}");
    }

    #[test]
    fn formulas_reject_what_does_not_add_up() {
        let mm = u(Quantity::Length, "mm");
        assert!(parse_formula("{x}", Quantity::Length, mm).is_err(), "not a reference token");
        assert!(parse_formula("{body_tube#3.cant} + 2", Quantity::Length, mm).is_err(), "an angle is not a length");
        assert!(parse_formula("{body_tube#3.od} * {body_tube#3.length}", Quantity::Length, mm).is_err());
        assert!(parse_formula("{body_tube#3.od} / {body_tube#3.length}", Quantity::Length, mm).is_err(), "a ratio is not a length");
        let scalar = Quantity::Scalar.unit_for(UnitSystem::MmGS);
        assert!(parse_formula("{body_tube#3.od} / {body_tube#3.length}", Quantity::Scalar, scalar).is_ok());
        assert!(formula_text(&bowshock_model::Expr::Num(1.0), Quantity::Mass, mm, &|_| None).is_none());
    }
}
