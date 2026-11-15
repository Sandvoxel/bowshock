//! Formulas: a dimension of one part computed from dimensions of other parts (design kit, UnitInput: *References*).
//!
//! A field holds either a plain number or a formula ([`Value`]). A formula is an [`Expr`] over numbers and references to other
//! fields, a [`FieldRef`] being a part id and a [`Dim`]. References bind to ids, so renaming a part never breaks one. The
//! document stores formulas as [`Formula`]s keyed by target field; after every edit [`Rocket::apply_formulas`] evaluates them in
//! topological order and writes the results with [`Rocket::set_dim`], so a change to a source part reaches every dependant.
//! A formula that would make a field depend on itself is refused ([`FormulaError::Circular`]); a formula whose referenced part
//! was deleted keeps the last valid value and reports [`FormulaStatus::Broken`].
//!
//! The expression inside the document is in SI (metres, radians; a literal beside a reference is a length in metres). Its text
//! form, used in files and for copy/paste, is `0.5 * {body_tube#3.od} + 0.002`: the part's name as a slug, `#`, its id, `.`,
//! and the dimension key. Only the id and key matter when reading it back.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::{Dim, PartId, Rocket};

/// Most passes [`Rocket::apply_formulas`] makes while a write to one field moves another (a diameter carried to a neighbour).
const MAX_PASSES: usize = 6;

/// What a dimension measures, so a formula cannot add a length to an angle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    /// Metres.
    Length,
    /// Radians.
    Angle,
    /// A plain number (a nose shape parameter).
    Scalar,
}

impl Kind {
    /// The lower-case noun used in messages.
    fn noun(self) -> &'static str {
        match self {
            Kind::Length => "length",
            Kind::Angle => "angle",
            Kind::Scalar => "plain number",
        }
    }
}

/// What `dim` measures.
pub fn dim_kind(dim: Dim) -> Kind {
    match dim {
        Dim::Cant | Dim::Angle => Kind::Angle,
        Dim::Shape => Kind::Scalar,
        _ => Kind::Length,
    }
}

/// The key of a dimension in the text form of a reference.
pub fn dim_key(dim: Dim) -> String {
    match dim {
        Dim::Length => "length".into(),
        Dim::ForeLength => "fore_length".into(),
        Dim::Diameter => "od".into(),
        Dim::ForeDiameter => "fore_od".into(),
        Dim::Wall => "wall".into(),
        Dim::Shape => "shape".into(),
        Dim::Position => "position".into(),
        Dim::Span => "span".into(),
        Dim::RootChord => "root_chord".into(),
        Dim::TipChord => "tip_chord".into(),
        Dim::Sweep => "sweep".into(),
        Dim::Thickness => "thickness".into(),
        Dim::Cant => "cant".into(),
        Dim::Angle => "angle".into(),
        Dim::Gap => "gap".into(),
        Dim::Height => "height".into(),
        Dim::InnerDiameter => "id".into(),
        Dim::Overhang => "overhang".into(),
        Dim::PackedLength => "packed_length".into(),
        Dim::ShoulderLength => "shoulder_length".into(),
        Dim::ShoulderDiameter => "shoulder_od".into(),
        Dim::ShoulderThickness => "shoulder_thickness".into(),
        Dim::ForeShoulderLength => "fore_shoulder_length".into(),
        Dim::ForeShoulderDiameter => "fore_shoulder_od".into(),
        Dim::ForeShoulderThickness => "fore_shoulder_thickness".into(),
        Dim::FilletRadius => "fillet".into(),
        Dim::Vertex(i) => format!("vertex{i}"),
    }
}

/// The dimension a key names, the inverse of [`dim_key`].
pub fn dim_from_key(key: &str) -> Option<Dim> {
    Some(match key {
        "length" => Dim::Length,
        "fore_length" => Dim::ForeLength,
        "od" => Dim::Diameter,
        "fore_od" => Dim::ForeDiameter,
        "wall" => Dim::Wall,
        "shape" => Dim::Shape,
        "position" => Dim::Position,
        "span" => Dim::Span,
        "root_chord" => Dim::RootChord,
        "tip_chord" => Dim::TipChord,
        "sweep" => Dim::Sweep,
        "thickness" => Dim::Thickness,
        "cant" => Dim::Cant,
        "angle" => Dim::Angle,
        "gap" => Dim::Gap,
        "height" => Dim::Height,
        "id" => Dim::InnerDiameter,
        "overhang" => Dim::Overhang,
        "packed_length" => Dim::PackedLength,
        "shoulder_length" => Dim::ShoulderLength,
        "shoulder_od" => Dim::ShoulderDiameter,
        "shoulder_thickness" => Dim::ShoulderThickness,
        "fore_shoulder_length" => Dim::ForeShoulderLength,
        "fore_shoulder_od" => Dim::ForeShoulderDiameter,
        "fore_shoulder_thickness" => Dim::ForeShoulderThickness,
        "fillet" => Dim::FilletRadius,
        other => Dim::Vertex(other.strip_prefix("vertex")?.parse().ok()?),
    })
}

/// A dimension of a part: the unit a formula refers to. A reference to one fin of a linked group is the group's own, as the
/// group is one part in the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FieldRef {
    /// The part.
    pub part: PartId,
    /// Which dimension.
    pub dim: Dim,
}

impl FieldRef {
    /// A reference to `dim` of `part`.
    pub fn new(part: PartId, dim: Dim) -> Self {
        FieldRef { part, dim }
    }

    /// The text of the reference's token without braces, `body_tube#3.od`; `name` is the part's name (slugged).
    pub fn token(self, name: &str) -> String {
        format!("{}#{}.{}", slug(name), self.part, dim_key(self.dim))
    }

    /// Reads a token written by [`token`](Self::token) (with or without the name): returns the reference and the slug.
    pub fn from_token(token: &str) -> Option<(FieldRef, String)> {
        let (slug, rest) = token.trim().split_once('#')?;
        let (id, key) = rest.split_once('.')?;
        Some((FieldRef { part: id.trim().parse().ok()?, dim: dim_from_key(key.trim())? }, slug.trim().to_string()))
    }
}

/// A part name as a token slug: lower case letters and digits, other runs becoming one underscore.
fn slug(name: &str) -> String {
    let mut out = String::new();
    for c in name.chars() {
        if c.is_alphanumeric() {
            out.extend(c.to_lowercase());
        } else if !out.is_empty() && !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_end_matches('_').to_string()
}

/// A binary operator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// `+`
    Add,
    /// `-`
    Sub,
    /// `*`
    Mul,
    /// `/`
    Div,
}

impl Op {
    fn symbol(self) -> char {
        match self {
            Op::Add => '+',
            Op::Sub => '-',
            Op::Mul => '*',
            Op::Div => '/',
        }
    }

    fn precedence(self) -> u8 {
        match self {
            Op::Add | Op::Sub => 1,
            Op::Mul | Op::Div => 2,
        }
    }
}

/// An expression over numbers and references to other fields.
#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    /// A literal.
    Num(f64),
    /// The value of a field of a part.
    Ref(FieldRef),
    /// Unary minus.
    Neg(Box<Expr>),
    /// `lhs op rhs`.
    Bin(Box<Expr>, Op, Box<Expr>),
}

/// Why an expression is wrong.
#[derive(Clone, Debug, PartialEq)]
pub enum ExprError {
    /// The text does not parse (message, byte offset).
    Syntax(String, usize),
    /// The units do not work out ("Can't add a length and an angle.").
    Units(String),
}

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExprError::Syntax(m, _) | ExprError::Units(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for ExprError {}

/// Why an expression has no value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EvalError {
    /// A reference whose part (or dimension) does not exist.
    Broken(FieldRef),
    /// Division by zero.
    DivideByZero,
    /// An overflow.
    NotFinite,
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvalError::Broken(r) => write!(f, "Part {} has no {}", r.part, dim_key(r.dim)),
            EvalError::DivideByZero => f.write_str("Division by zero"),
            EvalError::NotFinite => f.write_str("Result is not a finite number"),
        }
    }
}

impl std::error::Error for EvalError {}

/// What a sub-expression is, for the unit check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ty {
    /// A bare literal: takes the unit its neighbour has.
    Free,
    /// A plain number (a ratio, a factor).
    Scalar,
    /// A quantity of this kind.
    Of(Kind),
}

/// Which unit literals are in where they stand: a plain number, or the quantity of the field with this factor.
#[derive(Clone, Copy, Debug)]
enum Want {
    Scalar,
    Unit(f64),
    /// The unit of a quantity of this kind, which the field is not: a literal here cannot be converted.
    Unknown(Kind),
}

impl Expr {
    /// The references in the expression, in order of appearance, without repeats.
    pub fn refs(&self) -> Vec<FieldRef> {
        fn walk(e: &Expr, out: &mut Vec<FieldRef>) {
            match e {
                Expr::Num(_) => {}
                Expr::Ref(r) => {
                    if !out.contains(r) {
                        out.push(*r);
                    }
                }
                Expr::Neg(inner) => walk(inner, out),
                Expr::Bin(l, _, r) => {
                    walk(l, out);
                    walk(r, out);
                }
            }
        }
        let mut out = Vec::new();
        walk(self, &mut out);
        out
    }

    /// The value, with `get` supplying each referenced field (`None` for one that does not exist).
    pub fn eval(&self, get: &dyn Fn(FieldRef) -> Option<f64>) -> Result<f64, EvalError> {
        let v = match self {
            Expr::Num(v) => *v,
            Expr::Ref(r) => get(*r).ok_or(EvalError::Broken(*r))?,
            Expr::Neg(e) => -e.eval(get)?,
            Expr::Bin(l, op, r) => {
                let (a, b) = (l.eval(get)?, r.eval(get)?);
                match op {
                    Op::Add => a + b,
                    Op::Sub => a - b,
                    Op::Mul => a * b,
                    Op::Div if b == 0.0 => return Err(EvalError::DivideByZero),
                    Op::Div => a / b,
                }
            }
        };
        if v.is_finite() {
            Ok(v)
        } else {
            Err(EvalError::NotFinite)
        }
    }

    fn infer(&self) -> Result<Ty, ExprError> {
        let units = |m: String| Err(ExprError::Units(m));
        Ok(match self {
            Expr::Num(_) => Ty::Free,
            Expr::Ref(r) => Ty::Of(dim_kind(r.dim)),
            Expr::Neg(e) => e.infer()?,
            Expr::Bin(l, op, r) => {
                let (a, b) = (l.infer()?, r.infer()?);
                match (op, a, b) {
                    (Op::Add | Op::Sub, Ty::Free, t) | (Op::Add | Op::Sub, t, Ty::Free) => t,
                    (Op::Add | Op::Sub, Ty::Scalar, Ty::Scalar) => Ty::Scalar,
                    (Op::Add | Op::Sub, Ty::Of(x), Ty::Of(y)) if x == y => Ty::Of(x),
                    (Op::Add | Op::Sub, Ty::Of(x), Ty::Of(y)) => return units(format!("Can't add a {} and a {}.", x.noun(), y.noun())),
                    (Op::Add | Op::Sub, Ty::Of(x), Ty::Scalar) | (Op::Add | Op::Sub, Ty::Scalar, Ty::Of(x)) => {
                        return units(format!("Can't add a plain number to a {}.", x.noun()))
                    }
                    (Op::Mul, Ty::Of(x), Ty::Of(_)) => return units(format!("Can't multiply two values with units ({}).", x.noun())),
                    (Op::Mul, Ty::Of(x), _) | (Op::Mul, _, Ty::Of(x)) => Ty::Of(x),
                    (Op::Mul, Ty::Scalar, Ty::Scalar) => Ty::Scalar,
                    (Op::Mul, _, _) => Ty::Free,
                    (Op::Div, Ty::Of(x), Ty::Of(y)) if x == y => Ty::Scalar,
                    (Op::Div, Ty::Of(x), Ty::Of(y)) => return units(format!("Can't divide a {} by a {}.", x.noun(), y.noun())),
                    (Op::Div, _, Ty::Of(x)) => return units(format!("Can't divide a number by a {}.", x.noun())),
                    (Op::Div, t, _) => t,
                }
            }
        })
    }

    /// The literals of the expression scaled by `factor` where they stand for a quantity of the `target` kind (a bare literal
    /// beside a reference, or the whole expression): the conversion between a field's display unit and SI (`factor` =
    /// `si_per_unit`, or its inverse to go back). Literals that are factors or divisors stay as they are. `Err` when the
    /// units do not work out or the result is not of the `target` kind.
    pub fn scaled(&self, target: Kind, factor: f64) -> Result<Expr, ExprError> {
        let ty = self.infer()?;
        let ok = match ty {
            Ty::Free => true,
            Ty::Scalar => target == Kind::Scalar,
            Ty::Of(k) => k == target,
        };
        if !ok {
            return Err(ExprError::Units(match ty {
                Ty::Scalar => format!("The result is a ratio, not a {}.", target.noun()),
                Ty::Of(k) => format!("The result is a {}, not a {}.", k.noun(), target.noun()),
                Ty::Free => String::new(),
            }));
        }
        let want = if target == Kind::Scalar { Want::Scalar } else { Want::Unit(factor) };
        self.scale(target, factor, want)
    }

    fn scale(&self, target: Kind, factor: f64, want: Want) -> Result<Expr, ExprError> {
        // What a literal beside a part of type `t` stands for: the unit of what it sits next to, `outer` when it has no say.
        let beside = |t: Ty, outer: Want| -> Want {
            match t {
                Ty::Free => outer,
                Ty::Scalar | Ty::Of(Kind::Scalar) => Want::Scalar,
                Ty::Of(k) if k == target => Want::Unit(factor),
                Ty::Of(k) => Want::Unknown(k),
            }
        };
        Ok(match self {
            Expr::Num(v) => Expr::Num(match want {
                Want::Scalar => *v,
                Want::Unit(f) => v * f,
                Want::Unknown(k) => return Err(ExprError::Units(format!("Give the number next to a {} a unit.", k.noun()))),
            }),
            Expr::Ref(r) => Expr::Ref(*r),
            Expr::Neg(e) => Expr::Neg(Box::new(e.scale(target, factor, want)?)),
            Expr::Bin(l, op, r) => {
                let (tl, tr) = (l.infer()?, r.infer()?);
                let (wl, wr) = match op {
                    Op::Add | Op::Sub => {
                        let w = match (tl, tr) {
                            (Ty::Of(_), _) => beside(tl, want),
                            (_, Ty::Of(_)) => beside(tr, want),
                            (Ty::Scalar, _) | (_, Ty::Scalar) => Want::Scalar,
                            _ => want,
                        };
                        (w, w)
                    }
                    Op::Mul => match (tl, tr) {
                        (Ty::Of(_), _) => (beside(tl, want), Want::Scalar),
                        (_, Ty::Of(_)) => (Want::Scalar, beside(tr, want)),
                        (Ty::Scalar, Ty::Scalar) => (Want::Scalar, Want::Scalar),
                        (Ty::Scalar, _) => (Want::Scalar, want),
                        _ => (want, Want::Scalar),
                    },
                    Op::Div => (beside(tl, want), if matches!(tr, Ty::Of(_)) { beside(tr, want) } else { Want::Scalar }),
                };
                Expr::Bin(Box::new(l.scale(target, factor, wl)?), *op, Box::new(r.scale(target, factor, wr)?))
            }
        })
    }

    /// The text form, with each reference token carrying the name `name(part)` gives (a part that is not known gets none).
    pub fn to_text(&self, name: &dyn Fn(PartId) -> Option<String>) -> String {
        let mut out = String::new();
        self.write(&mut out, 0, name);
        out
    }

    fn write(&self, out: &mut String, parent: u8, name: &dyn Fn(PartId) -> Option<String>) {
        match self {
            Expr::Num(v) => out.push_str(&format!("{v}")),
            Expr::Ref(r) => {
                out.push('{');
                out.push_str(&r.token(&name(r.part).unwrap_or_default()));
                out.push('}');
            }
            Expr::Neg(e) => {
                out.push('-');
                if matches!(**e, Expr::Bin(..)) {
                    out.push('(');
                    e.write(out, 0, name);
                    out.push(')');
                } else {
                    e.write(out, 3, name);
                }
            }
            Expr::Bin(l, op, r) => {
                let p = op.precedence();
                let wrap = p < parent;
                if wrap {
                    out.push('(');
                }
                l.write(out, p, name);
                out.push(' ');
                out.push(op.symbol());
                out.push(' ');
                // The right operand of an operator of the same rank keeps its parentheses, so the text reads back as the same tree.
                r.write(out, p + 1, name);
                if wrap {
                    out.push(')');
                }
            }
        }
    }

    /// Reads the text form (SI numbers, `{name#id.key}` references; `+ - * /`, parentheses, unary minus).
    pub fn parse(text: &str) -> Result<Expr, ExprError> {
        let mut p = TextParser { src: text, pos: 0 };
        let e = p.expr()?;
        p.skip_ws();
        if p.pos < text.len() {
            return p.fail("Unexpected text");
        }
        Ok(e)
    }
}

impl fmt::Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_text(&|_| None))
    }
}

struct TextParser<'a> {
    src: &'a str,
    pos: usize,
}

impl TextParser<'_> {
    fn fail<T>(&self, message: &str) -> Result<T, ExprError> {
        Err(ExprError::Syntax(message.to_string(), self.pos))
    }

    fn peek(&self) -> Option<char> {
        self.src[self.pos..].chars().next()
    }

    fn skip_ws(&mut self) {
        while self.peek().is_some_and(char::is_whitespace) {
            self.pos += self.peek().map_or(0, char::len_utf8);
        }
    }

    fn expr(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.term()?;
        loop {
            self.skip_ws();
            let op = match self.peek() {
                Some('+') => Op::Add,
                Some('-') => Op::Sub,
                _ => return Ok(lhs),
            };
            self.pos += 1;
            lhs = Expr::Bin(Box::new(lhs), op, Box::new(self.term()?));
        }
    }

    fn term(&mut self) -> Result<Expr, ExprError> {
        let mut lhs = self.unary()?;
        loop {
            self.skip_ws();
            let op = match self.peek() {
                Some('*') => Op::Mul,
                Some('/') => Op::Div,
                _ => return Ok(lhs),
            };
            self.pos += 1;
            lhs = Expr::Bin(Box::new(lhs), op, Box::new(self.unary()?));
        }
    }

    fn unary(&mut self) -> Result<Expr, ExprError> {
        self.skip_ws();
        match self.peek() {
            Some('-') => {
                self.pos += 1;
                Ok(Expr::Neg(Box::new(self.unary()?)))
            }
            Some('(') => {
                self.pos += 1;
                let inner = self.expr()?;
                self.skip_ws();
                if self.peek() != Some(')') {
                    return self.fail("Missing ')'");
                }
                self.pos += 1;
                Ok(inner)
            }
            Some('{') => {
                let Some(len) = self.src[self.pos..].find('}') else {
                    return self.fail("Unclosed '{'");
                };
                let token = &self.src[self.pos + 1..self.pos + len];
                let Some((r, _)) = FieldRef::from_token(token) else {
                    return self.fail("Not a reference");
                };
                self.pos += len + 1;
                Ok(Expr::Ref(r))
            }
            Some(c) if c.is_ascii_digit() || c == '.' => {
                let rest = &self.src[self.pos..];
                let mut end = rest.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(rest.len());
                // An exponent: `1e-7`.
                if rest[end..].starts_with(['e', 'E']) {
                    let tail = &rest[end + 1..];
                    let sign = usize::from(tail.starts_with(['+', '-']));
                    let digits = tail[sign..].find(|c: char| !c.is_ascii_digit()).unwrap_or(tail.len() - sign);
                    if digits > 0 {
                        end += 1 + sign + digits;
                    }
                }
                match rest[..end].parse::<f64>() {
                    Ok(v) if v.is_finite() => {
                        self.pos += end;
                        Ok(Expr::Num(v))
                    }
                    _ => self.fail("Not a number"),
                }
            }
            _ => self.fail("Expected a number or a reference"),
        }
    }
}

/// The value of a field: a plain number or a formula.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// A number.
    Plain(f64),
    /// A formula.
    Expr(Expr),
}

/// A formula in the document: the field it computes, the expression, and the names its references had when it was written (so
/// a reference to a deleted part can still be shown).
#[derive(Clone, Debug, PartialEq)]
pub struct Formula {
    /// The field the formula computes.
    pub target: FieldRef,
    /// The expression, in SI.
    pub expr: Expr,
    /// The last known name of each referenced part.
    pub names: BTreeMap<PartId, String>,
}

/// The file form of a [`Formula`].
#[derive(Serialize, Deserialize)]
struct FormulaRepr {
    part: PartId,
    dim: String,
    expr: String,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    names: BTreeMap<PartId, String>,
}

impl Serialize for Formula {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let name = |p: PartId| self.names.get(&p).cloned();
        FormulaRepr { part: self.target.part, dim: dim_key(self.target.dim), expr: self.expr.to_text(&name), names: self.names.clone() }.serialize(s)
    }
}

impl<'de> Deserialize<'de> for Formula {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        use serde::de::Error;
        let repr = FormulaRepr::deserialize(d)?;
        let dim = dim_from_key(&repr.dim).ok_or_else(|| D::Error::custom(format!("unknown dimension '{}'", repr.dim)))?;
        let expr = Expr::parse(&repr.expr).map_err(D::Error::custom)?;
        Ok(Formula { target: FieldRef { part: repr.part, dim }, expr, names: repr.names })
    }
}

/// Why a formula was not accepted.
#[derive(Clone, Debug, PartialEq)]
pub enum FormulaError {
    /// The target part has no such dimension, or the formula refers to one that does not exist.
    NoSuchField(FieldRef),
    /// `referenced` already depends on `target` (or is it): the formula would make a field depend on itself.
    Circular {
        /// The field the formula is for.
        target: FieldRef,
        /// The referenced field that already uses it.
        referenced: FieldRef,
    },
    /// The expression has no value now.
    Eval(EvalError),
    /// The units of the expression do not work out.
    Units(String),
}

impl fmt::Display for FormulaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FormulaError::NoSuchField(r) => write!(f, "Part {} has no {}", r.part, dim_key(r.dim)),
            FormulaError::Circular { .. } => f.write_str("Circular reference"),
            FormulaError::Eval(e) => e.fmt(f),
            FormulaError::Units(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for FormulaError {}

/// Whether a formula can be evaluated.
#[derive(Clone, Debug, PartialEq)]
pub enum FormulaStatus {
    /// Every reference resolves.
    Ok,
    /// These references point at parts (or dimensions) that no longer exist; the field keeps its last valid value.
    Broken(Vec<FieldRef>),
}

impl Rocket {
    /// The formulas of the document, in the order they were written.
    pub fn formulas(&self) -> &[Formula] {
        &self.formulas
    }

    /// The formula that computes `dim` of `part`, if it has one.
    pub fn formula(&self, part: PartId, dim: Dim) -> Option<&Formula> {
        self.formulas.iter().find(|f| f.target == FieldRef { part, dim })
    }

    /// The value of `dim` of `part`: its formula if it has one, else the number. `None` if the part has no such dimension.
    pub fn value(&self, part: PartId, dim: Dim) -> Option<Value> {
        let plain = self.dim(part, dim)?;
        Some(self.formula(part, dim).map_or(Value::Plain(plain), |f| Value::Expr(f.expr.clone())))
    }

    /// The fields whose formulas refer to `dim` of `part`.
    pub fn dependants(&self, part: PartId, dim: Dim) -> Vec<FieldRef> {
        let source = FieldRef { part, dim };
        self.formulas.iter().filter(|f| f.expr.refs().contains(&source)).map(|f| f.target).collect()
    }

    /// Whether every reference of the formula on `dim` of `part` resolves (`Ok` for a field without a formula).
    pub fn formula_status(&self, part: PartId, dim: Dim) -> FormulaStatus {
        let Some(f) = self.formula(part, dim) else {
            return FormulaStatus::Ok;
        };
        let broken: Vec<FieldRef> = f.expr.refs().into_iter().filter(|r| self.dim(r.part, r.dim).is_none()).collect();
        if broken.is_empty() {
            FormulaStatus::Ok
        } else {
            FormulaStatus::Broken(broken)
        }
    }

    /// The name a formula remembers for `part`, else its name in the document, else `None`.
    pub fn formula_name(&self, formula: &Formula, part: PartId) -> Option<String> {
        self.kind_of(part).map(|p| p.name()).or_else(|| formula.names.get(&part).cloned())
    }

    /// Whether `from` depends on `to`, directly or through other formulas (a field depends on itself).
    fn depends_on(&self, from: FieldRef, to: FieldRef) -> bool {
        let mut seen: BTreeSet<(PartId, String)> = BTreeSet::new();
        let mut stack = vec![from];
        while let Some(at) = stack.pop() {
            if at == to {
                return true;
            }
            if !seen.insert((at.part, dim_key(at.dim))) {
                continue;
            }
            if let Some(f) = self.formula(at.part, at.dim) {
                stack.extend(f.expr.refs());
            }
        }
        false
    }

    /// Makes `dim` of `part` the formula `expr` (in SI) and applies it. Refused, with the document unchanged, when the part has
    /// no such dimension, a reference does not exist, a reference already uses this field (a cycle), or the expression has no
    /// value. Replaces any formula the field had.
    pub fn set_formula(&mut self, part: PartId, dim: Dim, expr: Expr) -> Result<(), FormulaError> {
        let target = FieldRef { part, dim };
        if self.dim(part, dim).is_none() {
            return Err(FormulaError::NoSuchField(target));
        }
        let kind = dim_kind(dim);
        if let Err(ExprError::Units(m)) = expr.scaled(kind, 1.0) {
            return Err(FormulaError::Units(m));
        }
        for r in expr.refs() {
            if self.dim(r.part, r.dim).is_none() {
                return Err(FormulaError::NoSuchField(r));
            }
            if self.depends_on(r, target) {
                return Err(FormulaError::Circular { target, referenced: r });
            }
        }
        expr.eval(&|r| self.dim(r.part, r.dim)).map_err(FormulaError::Eval)?;
        let names = expr.refs().into_iter().filter_map(|r| self.kind_of(r.part).map(|p| (r.part, p.name()))).collect();
        self.formulas.retain(|f| f.target != target);
        self.formulas.push(Formula { target, expr, names });
        self.apply_formulas();
        Ok(())
    }

    /// Turns the formula of `dim` of `part` into a plain number (the value it has now). Returns whether there was one.
    pub fn clear_formula(&mut self, part: PartId, dim: Dim) -> bool {
        let before = self.formulas.len();
        self.formulas.retain(|f| f.target != FieldRef { part, dim });
        self.formulas.len() != before
    }

    /// The order to evaluate the formulas in: every formula after the formulas of the fields it refers to. Formulas caught in a
    /// cycle (only a hand-edited file can have one) are left out.
    fn formula_order(&self) -> Vec<usize> {
        let index: HashMap<FieldRef, usize> = self.formulas.iter().enumerate().map(|(i, f)| (f.target, i)).collect();
        // 0 unvisited, 1 in progress, 2 done.
        let mut state = vec![0u8; self.formulas.len()];
        let mut order = Vec::with_capacity(self.formulas.len());
        fn visit(i: usize, rocket: &Rocket, index: &HashMap<FieldRef, usize>, state: &mut [u8], order: &mut Vec<usize>) -> bool {
            match state[i] {
                2 => return true,
                1 => return false,
                _ => {}
            }
            state[i] = 1;
            let mut acyclic = true;
            for r in rocket.formulas[i].expr.refs() {
                if let Some(&j) = index.get(&r) {
                    acyclic &= visit(j, rocket, index, state, order);
                }
            }
            state[i] = 2;
            if acyclic {
                order.push(i);
            }
            acyclic
        }
        for i in 0..self.formulas.len() {
            visit(i, self, &index, &mut state, &mut order);
        }
        order
    }

    /// Evaluates every formula in topological order and writes the results (limited as [`set_dim`](Self::set_dim) limits them).
    /// A formula with a broken reference or no value is skipped, so its field keeps its last valid value. The names formulas
    /// remember are refreshed. Call after every edit.
    pub fn apply_formulas(&mut self) {
        if self.formulas.is_empty() {
            return;
        }
        let order = self.formula_order();
        for _ in 0..MAX_PASSES {
            let mut moved = false;
            for &i in &order {
                let (target, expr) = (self.formulas[i].target, self.formulas[i].expr.clone());
                let Ok(value) = expr.eval(&|r| self.dim(r.part, r.dim)) else {
                    continue;
                };
                let Some(current) = self.dim(target.part, target.dim) else {
                    continue;
                };
                if (current - value).abs() <= 1e-12 * value.abs().max(1.0) {
                    continue;
                }
                if self.set_dim(target.part, target.dim, value).is_some() && self.dim(target.part, target.dim).is_some_and(|now| now != current) {
                    moved = true;
                }
            }
            if !moved {
                break;
            }
        }
        for i in 0..self.formulas.len() {
            for r in self.formulas[i].expr.refs() {
                if let Some(name) = self.kind_of(r.part).map(|p| p.name()) {
                    if self.formulas[i].names.get(&r.part) != Some(&name) {
                        self.formulas[i].names.insert(r.part, name);
                    }
                }
            }
        }
    }

    /// Drops the formulas of parts that no longer exist (call after an edit that is not a drag).
    pub fn prune_formulas(&mut self) {
        let keep: Vec<bool> = self.formulas.iter().map(|f| self.dim(f.target.part, f.target.dim).is_some()).collect();
        let mut keep = keep.into_iter();
        self.formulas.retain(|_| keep.next().unwrap_or(true));
    }

    /// Makes loaded formulas consistent: one per field, none for a field that does not exist, none in a cycle.
    pub(crate) fn repair_formulas(&mut self) {
        let mut seen: BTreeSet<(PartId, String)> = BTreeSet::new();
        let formulas = std::mem::take(&mut self.formulas);
        self.formulas = formulas
            .into_iter()
            .filter(|f| f.expr.scaled(dim_kind(f.target.dim), 1.0).is_ok())
            .filter(|f| self.dim(f.target.part, f.target.dim).is_some() && seen.insert((f.target.part, dim_key(f.target.dim))))
            .collect();
        let ordered: BTreeSet<usize> = self.formula_order().into_iter().collect();
        let mut i = 0;
        self.formulas.retain(|_| {
            i += 1;
            ordered.contains(&(i - 1))
        });
        self.apply_formulas();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PartKind, RadialKind};

    /// A tube (id 0) of 76.2 mm with a 4-fin group (id 1) on it.
    fn rocket() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        let tube = r.insert(0, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let fins =
            r.attach_radial(tube, 0.3, 0.0, 4, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.08, sweep: 0.06, thickness: 0.003 }).unwrap();
        (r, tube, fins)
    }

    fn od(part: PartId) -> Expr {
        Expr::Ref(FieldRef::new(part, Dim::Diameter))
    }

    fn mul(a: Expr, b: Expr) -> Expr {
        Expr::Bin(Box::new(a), Op::Mul, Box::new(b))
    }

    #[test]
    fn text_form_round_trips() {
        let e = Expr::Bin(
            Box::new(mul(Expr::Num(0.5), od(3))),
            Op::Add,
            Box::new(Expr::Bin(Box::new(Expr::Num(0.002)), Op::Sub, Box::new(Expr::Neg(Box::new(Expr::Num(1e-7)))))),
        );
        let named = |p: PartId| Some(if p == 3 { "Body tube".to_string() } else { String::new() });
        let text = e.to_text(&named);
        assert_eq!(text, "0.5 * {body_tube#3.od} + (0.002 - -0.0000001)");
        assert_eq!(Expr::parse(&text).unwrap(), e);
    }

    #[test]
    fn parentheses_follow_precedence() {
        // (a + b) * c keeps its parentheses; a - (b - c) keeps its tree.
        let a = Expr::parse("({#1.length} + {#2.length}) * 2").unwrap();
        assert_eq!(a.to_text(&|_| None), "({#1.length} + {#2.length}) * 2");
        let b = Expr::parse("1 - (2 - 3)").unwrap();
        assert_eq!(b.to_text(&|_| None), "1 - (2 - 3)");
        assert_eq!(b.eval(&|_| None), Ok(2.0));
        assert_eq!(Expr::parse("1 - 2 - 3").unwrap().eval(&|_| None), Ok(-4.0));
    }

    #[test]
    fn parse_rejects_garbage() {
        assert!(Expr::parse("2 +").is_err());
        assert!(Expr::parse("{#x.od}").is_err());
        assert!(Expr::parse("{#1.nope}").is_err());
        assert!(Expr::parse("2 3").is_err());
        assert!(Expr::parse("(2").is_err());
    }

    #[test]
    fn a_formula_follows_its_source_in_topological_order() {
        let (mut r, tube, fins) = rocket();
        // Root chord = 2 x tube diameter; tip chord = 0.4 x root chord (declared first, evaluated second).
        r.set_formula(fins, Dim::RootChord, mul(Expr::Num(2.0), od(tube))).unwrap();
        r.set_formula(fins, Dim::TipChord, mul(Expr::Num(0.4), Expr::Ref(FieldRef::new(fins, Dim::RootChord)))).unwrap();
        assert!((r.dim(fins, Dim::RootChord).unwrap() - 0.1524).abs() < 1e-12);
        assert!((r.dim(fins, Dim::TipChord).unwrap() - 0.06096).abs() < 1e-12);
        // Change the source: both dependants follow, in one apply.
        r.set_dim(tube, Dim::Diameter, 0.1).unwrap();
        r.apply_formulas();
        assert!((r.dim(fins, Dim::RootChord).unwrap() - 0.2).abs() < 1e-12);
        assert!((r.dim(fins, Dim::TipChord).unwrap() - 0.08).abs() < 1e-12);
        assert_eq!(r.dependants(tube, Dim::Diameter), vec![FieldRef::new(fins, Dim::RootChord)]);
        assert_eq!(r.dependants(fins, Dim::RootChord), vec![FieldRef::new(fins, Dim::TipChord)]);
    }

    #[test]
    fn cycles_are_refused_with_the_field_that_already_uses_it() {
        let (mut r, _, fins) = rocket();
        let root = FieldRef::new(fins, Dim::RootChord);
        let tip = FieldRef::new(fins, Dim::TipChord);
        r.set_formula(fins, Dim::RootChord, mul(Expr::Num(2.0), Expr::Ref(tip))).unwrap();
        let before = r.clone();
        let err = r.set_formula(fins, Dim::TipChord, mul(Expr::Num(0.4), Expr::Ref(root))).unwrap_err();
        assert_eq!(err, FormulaError::Circular { target: tip, referenced: root });
        assert_eq!(r, before);
        // A field referring to itself is a cycle too.
        let err = r.set_formula(fins, Dim::TipChord, Expr::Ref(tip)).unwrap_err();
        assert_eq!(err, FormulaError::Circular { target: tip, referenced: tip });
        // A longer loop: a -> b -> c -> a.
        let sweep = FieldRef::new(fins, Dim::Sweep);
        r.set_formula(fins, Dim::TipChord, Expr::Ref(sweep)).unwrap();
        let err = r.set_formula(fins, Dim::Sweep, Expr::Ref(root)).unwrap_err();
        assert!(matches!(err, FormulaError::Circular { .. }));
    }

    #[test]
    fn deleting_the_source_breaks_the_reference_and_keeps_the_last_value() {
        let (mut r, tube, fins) = rocket();
        r.set_formula(fins, Dim::RootChord, mul(Expr::Num(2.0), od(tube))).unwrap();
        let name = |p: PartId| r.kind_of(p).map(|k| k.name());
        assert_eq!(name(tube).as_deref(), Some("Body tube"));
        // Remove the tube's fins first (they hang off it), then the tube itself: use a second tube as the source instead.
        let mut r = Rocket::default();
        let a = r.insert(0, PartKind::BodyTube { length: 0.3, diameter: 0.1 }).unwrap();
        let b = r.insert(1, PartKind::BodyTube { length: 0.3, diameter: 0.1 }).unwrap();
        r.set_formula(b, Dim::Length, mul(Expr::Num(2.0), Expr::Ref(FieldRef::new(a, Dim::Length)))).unwrap();
        assert!((r.dim(b, Dim::Length).unwrap() - 0.6).abs() < 1e-12);
        r.remove(a).unwrap();
        r.apply_formulas();
        assert_eq!(r.formula_status(b, Dim::Length), FormulaStatus::Broken(vec![FieldRef::new(a, Dim::Length)]));
        assert!((r.dim(b, Dim::Length).unwrap() - 0.6).abs() < 1e-12, "the last valid value is kept");
        assert_eq!(r.formula(b, Dim::Length).unwrap().names.get(&a).map(String::as_str), Some("Body tube"));
        // Converting to a plain value drops the formula and keeps the number.
        assert!(r.clear_formula(b, Dim::Length));
        assert_eq!(r.value(b, Dim::Length), Some(Value::Plain(r.dim(b, Dim::Length).unwrap())));
    }

    #[test]
    fn renaming_a_part_does_not_break_a_reference() {
        let (mut r, tube, fins) = rocket();
        r.set_formula(fins, Dim::RootChord, mul(Expr::Num(2.0), od(tube))).unwrap();
        r.set_name(tube, Some("Airframe".to_string()));
        r.apply_formulas();
        assert_eq!(r.formula_status(fins, Dim::RootChord), FormulaStatus::Ok);
        let f = r.formula(fins, Dim::RootChord).unwrap();
        assert_eq!(r.formula_name(f, tube).as_deref(), Some("Airframe"));
        assert!(f.expr.to_text(&|p| r.formula_name(f, p)).contains("{airframe#"));
    }

    #[test]
    fn a_reference_to_a_missing_field_is_refused() {
        let (mut r, tube, fins) = rocket();
        assert!(matches!(r.set_formula(fins, Dim::RootChord, od(99)), Err(FormulaError::NoSuchField(_))));
        // A tube has no root chord.
        assert!(matches!(r.set_formula(tube, Dim::RootChord, Expr::Num(1.0)), Err(FormulaError::NoSuchField(_))));
    }

    #[test]
    fn units_are_checked_and_scaled() {
        let (tube, fins) = (7, 8);
        let len = Expr::Ref(FieldRef::new(tube, Dim::Length));
        let ang = Expr::Ref(FieldRef::new(fins, Dim::Cant));
        // `0.5 * {len} + 2` typed in millimetres: the 0.5 stays, the 2 becomes metres.
        let typed = Expr::Bin(Box::new(mul(Expr::Num(0.5), len.clone())), Op::Add, Box::new(Expr::Num(2.0)));
        let si = typed.scaled(Kind::Length, 0.001).unwrap();
        assert_eq!(si, Expr::Bin(Box::new(mul(Expr::Num(0.5), len.clone())), Op::Add, Box::new(Expr::Num(0.002))));
        // And back to the display unit.
        assert_eq!(si.scaled(Kind::Length, 1000.0).unwrap(), typed);
        // A length divided by a length is a ratio: no good for a length field, fine for a scalar one.
        let ratio = Expr::Bin(Box::new(len.clone()), Op::Div, Box::new(len.clone()));
        assert!(ratio.scaled(Kind::Length, 0.001).is_err());
        assert!(ratio.scaled(Kind::Scalar, 1.0).is_ok());
        // `{len} / 2 * 3` : the divisor and factor are plain.
        let q = mul(Expr::Bin(Box::new(len.clone()), Op::Div, Box::new(Expr::Num(2.0))), Expr::Num(3.0));
        assert_eq!(q.scaled(Kind::Length, 0.001).unwrap(), q);
        // Lengths and angles do not mix.
        assert!(Expr::Bin(Box::new(len.clone()), Op::Add, Box::new(ang.clone())).scaled(Kind::Length, 0.001).is_err());
        assert!(ang.scaled(Kind::Length, 0.001).is_err());
        assert!(mul(len.clone(), len).scaled(Kind::Length, 0.001).is_err());
        // A pure number is fine anywhere.
        assert_eq!(Expr::Num(3.0).scaled(Kind::Length, 0.001).unwrap(), Expr::Num(0.003));
    }

    #[test]
    fn formulas_survive_json_and_old_files_load() {
        let (mut r, tube, fins) = rocket();
        r.set_formula(fins, Dim::RootChord, Expr::Bin(Box::new(mul(Expr::Num(2.0), od(tube))), Op::Add, Box::new(Expr::Num(0.002)))).unwrap();
        let json = r.to_json();
        assert!(json.contains("2 * {body_tube#0.od} + 0.002"), "{json}");
        let back = Rocket::from_json(&json).unwrap();
        assert_eq!(back.formulas(), r.formulas());
        assert_eq!(back, r);
        // A file without the field (an older document) loads with none.
        let without = serde_json::to_value(&r).unwrap();
        let mut value = without;
        value.as_object_mut().unwrap().remove("formulas");
        let text = serde_json::json!({ "format_version": crate::FORMAT_VERSION, "rocket": value }).to_string();
        assert!(Rocket::from_json(&text).unwrap().formulas().is_empty());
    }

    #[test]
    fn a_hand_edited_cycle_is_dropped_on_load() {
        let (mut r, tube, fins) = rocket();
        r.set_formula(fins, Dim::RootChord, mul(Expr::Num(2.0), od(tube))).unwrap();
        let mut value = serde_json::to_value(&r).unwrap();
        // Make the tube's diameter depend on the fin's root chord, which depends on it.
        value["formulas"].as_array_mut().unwrap().push(serde_json::json!({ "part": tube, "dim": "od", "expr": format!("{{#{fins}.root_chord}} / 2") }));
        let text = serde_json::json!({ "format_version": crate::FORMAT_VERSION, "rocket": value }).to_string();
        let loaded = Rocket::from_json(&text).unwrap();
        assert!(loaded.formulas().len() <= 1, "{:?}", loaded.formulas());
    }
}
