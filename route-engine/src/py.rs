//! Python semantics the ported controller relies on.
//!
//! * `V`: a dynamic value with Python's int/float/bool distinction and
//!   insertion-ordered dicts (evidence, targets, scans and control dicts are
//!   passed and returned exactly as the Python code builds them).
//! * Exceptions: `raise` unwinds with a `PyErr`; `py_try`/`except` catch the
//!   same exception kinds the Python code catches. Any other Rust panic is a
//!   `RustPanic`, which no ported `except` clause catches.
//! * Math that CPython does not take from the C library (hypot/dist, sum,
//!   round, statistics.mean, float %) reproduced bit for bit; `**2` calls the
//!   C library pow as CPython does.

use std::fmt::Write as _;
use std::panic;
use std::rc::Rc;

// ---------------------------------------------------------------- exceptions

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exc {
    ValueError,
    TypeError,
    KeyError,
    IndexError,
    ZeroDivisionError,
    OverflowError,
    RustPanic,
}

impl Exc {
    pub fn name(self) -> &'static str {
        match self {
            Exc::ValueError => "ValueError",
            Exc::TypeError => "TypeError",
            Exc::KeyError => "KeyError",
            Exc::IndexError => "IndexError",
            Exc::ZeroDivisionError => "ZeroDivisionError",
            Exc::OverflowError => "OverflowError",
            Exc::RustPanic => "RustPanic",
        }
    }
}

#[derive(Clone, Debug)]
pub struct PyErr {
    pub kind: Exc,
    pub msg: String,
}

pub fn raise(kind: Exc, msg: impl Into<String>) -> ! {
    panic::panic_any(PyErr { kind, msg: msg.into() })
}

pub fn value_error(msg: impl Into<String>) -> ! {
    raise(Exc::ValueError, msg)
}

pub fn type_error(msg: impl Into<String>) -> ! {
    raise(Exc::TypeError, msg)
}

/// Run `f`; any exception (or Rust panic) is returned as an error.
pub fn py_try<T>(f: impl FnOnce() -> T) -> Result<T, PyErr> {
    match panic::catch_unwind(panic::AssertUnwindSafe(f)) {
        Ok(value) => Ok(value),
        Err(payload) => Err(match payload.downcast::<PyErr>() {
            Ok(error) => *error,
            Err(other) => {
                let msg = if let Some(s) = other.downcast_ref::<String>() {
                    s.clone()
                } else if let Some(s) = other.downcast_ref::<&str>() {
                    (*s).to_string()
                } else {
                    "panic".to_string()
                };
                PyErr { kind: Exc::RustPanic, msg }
            }
        }),
    }
}

/// `try: f() except kinds as e:` -- other exceptions propagate unchanged.
pub fn except<T>(kinds: &[Exc], f: impl FnOnce() -> T) -> Result<T, PyErr> {
    match py_try(f) {
        Ok(value) => Ok(value),
        Err(error) if kinds.contains(&error.kind) => Err(error),
        Err(error) => panic::panic_any(error),
    }
}

pub const VTK: &[Exc] = &[Exc::ValueError, Exc::TypeError, Exc::KeyError];

/// Silence the default hook for Python exceptions; real panics still print.
pub fn install_panic_hook() {
    panic::set_hook(Box::new(|info| {
        if info.payload().is::<PyErr>() {
            return;
        }
        eprintln!("route-engine panic: {info}");
    }));
}

// ---------------------------------------------------------------- values

pub type Key = Rc<str>;

#[derive(Clone, Debug, Default)]
pub enum V {
    #[default]
    None,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(Rc<str>),
    List(Rc<Vec<V>>),
    Dict(Rc<Vec<(Key, V)>>),
}

impl V {
    pub fn dict() -> V {
        V::Dict(Rc::new(Vec::new()))
    }

    pub fn list(items: Vec<V>) -> V {
        V::List(Rc::new(items))
    }

    pub fn str(s: &str) -> V {
        V::Str(Rc::from(s))
    }

    pub fn is_none(&self) -> bool {
        matches!(self, V::None)
    }

    pub fn is_some(&self) -> bool {
        !self.is_none()
    }

    /// `type(x) in (int, float) and math.isfinite(x)` (bool excluded).
    pub fn is_num(&self) -> bool {
        match self {
            V::Int(_) => true,
            V::Float(x) => x.is_finite(),
            _ => false,
        }
    }

    /// `type(x) is int`.
    pub fn is_int(&self) -> bool {
        matches!(self, V::Int(_))
    }

    pub fn is_float(&self) -> bool {
        matches!(self, V::Float(_))
    }

    /// `type(x) in (int, float)` (bool excluded), finite or not.
    pub fn is_real(&self) -> bool {
        matches!(self, V::Int(_) | V::Float(_))
    }

    pub fn is_bool(&self) -> bool {
        matches!(self, V::Bool(_))
    }

    /// `x is True`.
    pub fn is_true(&self) -> bool {
        matches!(self, V::Bool(true))
    }

    /// `x is False`.
    pub fn is_false(&self) -> bool {
        matches!(self, V::Bool(false))
    }

    pub fn is_dict(&self) -> bool {
        matches!(self, V::Dict(_))
    }

    pub fn is_list(&self) -> bool {
        matches!(self, V::List(_))
    }

    pub fn is_str(&self) -> bool {
        matches!(self, V::Str(_))
    }

    /// Python truthiness.
    pub fn truthy(&self) -> bool {
        match self {
            V::None => false,
            V::Bool(b) => *b,
            V::Int(i) => *i != 0,
            V::Float(x) => *x != 0.,
            V::Str(s) => !s.is_empty(),
            V::List(l) => !l.is_empty(),
            V::Dict(d) => !d.is_empty(),
        }
    }

    /// A number as float (bool counts as int, as in Python arithmetic).
    pub fn f(&self) -> f64 {
        match self {
            V::Int(i) => *i as f64,
            V::Float(x) => *x,
            V::Bool(b) => *b as i64 as f64,
            other => type_error(format!("number expected, got {}", other.type_name())),
        }
    }

    pub fn i(&self) -> i64 {
        match self {
            V::Int(i) => *i,
            V::Bool(b) => *b as i64,
            other => type_error(format!("int expected, got {}", other.type_name())),
        }
    }

    pub fn of(&self) -> Option<f64> {
        if self.is_none() { None } else { Some(self.f()) }
    }

    pub fn oi(&self) -> Option<i64> {
        if self.is_none() { None } else { Some(self.i()) }
    }

    pub fn s(&self) -> &str {
        match self {
            V::Str(s) => s,
            other => type_error(format!("str expected, got {}", other.type_name())),
        }
    }

    /// The string, or None when this is not a string.
    pub fn os(&self) -> Option<&str> {
        match self {
            V::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn type_name(&self) -> &'static str {
        match self {
            V::None => "NoneType",
            V::Bool(_) => "bool",
            V::Int(_) => "int",
            V::Float(_) => "float",
            V::Str(_) => "str",
            V::List(_) => "list",
            V::Dict(_) => "dict",
        }
    }

    pub fn items(&self) -> &[(Key, V)] {
        match self {
            V::Dict(d) => d,
            other => type_error(format!("dict expected, got {}", other.type_name())),
        }
    }

    pub fn l(&self) -> &[V] {
        match self {
            V::List(l) => l,
            other => type_error(format!("list expected, got {}", other.type_name())),
        }
    }

    /// The list, or None when this is not a list.
    pub fn ol(&self) -> Option<&[V]> {
        match self {
            V::List(l) => Some(l),
            _ => None,
        }
    }

    pub fn len(&self) -> usize {
        match self {
            V::List(l) => l.len(),
            V::Dict(d) => d.len(),
            V::Str(s) => s.chars().count(),
            other => type_error(format!("object of type {} has no len()", other.type_name())),
        }
    }

    /// `d.get(k)` (None for a missing key or a non-dict).
    pub fn get(&self, key: &str) -> &V {
        if let V::Dict(d) = self {
            for (k, v) in d.iter() {
                if &**k == key {
                    return v;
                }
            }
        }
        &V::None
    }

    pub fn has(&self, key: &str) -> bool {
        if let V::Dict(d) = self {
            d.iter().any(|(k, _)| &**k == key)
        } else {
            false
        }
    }

    /// `d[k]`.
    pub fn idx(&self, key: &str) -> &V {
        match self {
            V::Dict(d) => {
                for (k, v) in d.iter() {
                    if &**k == key {
                        return v;
                    }
                }
                raise(Exc::KeyError, format!("'{key}'"))
            }
            V::None => type_error("'NoneType' object is not subscriptable"),
            other => type_error(format!("{} indices must be integers", other.type_name())),
        }
    }

    /// `l[i]` (negative indices count from the end).
    pub fn at(&self, index: i64) -> &V {
        let list = self.l();
        let n = list.len() as i64;
        let i = if index < 0 { index + n } else { index };
        if i < 0 || i >= n {
            raise(Exc::IndexError, "list index out of range")
        }
        &list[i as usize]
    }

    /// `d[k] = v` (an existing key keeps its position).
    pub fn set(&mut self, key: &str, value: impl Into<V>) {
        let value = value.into();
        match self {
            V::Dict(d) => {
                let d = Rc::make_mut(d);
                for (k, v) in d.iter_mut() {
                    if &**k == key {
                        *v = value;
                        return;
                    }
                }
                d.push((Rc::from(key), value));
            }
            other => type_error(format!("'{}' object does not support item assignment", other.type_name())),
        }
    }

    pub fn remove(&mut self, key: &str) -> Option<V> {
        if let V::Dict(d) = self {
            let d = Rc::make_mut(d);
            if let Some(pos) = d.iter().position(|(k, _)| &**k == key) {
                return Some(d.remove(pos).1);
            }
        }
        None
    }

    /// `{**self, **other}` / `dict.update`.
    pub fn update(&mut self, other: &V) {
        for (k, v) in other.items() {
            self.set(k, v.clone());
        }
    }

    pub fn merged(&self, other: &V) -> V {
        let mut out = self.clone();
        out.update(other);
        out
    }

    pub fn keys(&self) -> Vec<Key> {
        self.items().iter().map(|(k, _)| k.clone()).collect()
    }

    pub fn push(&mut self, value: impl Into<V>) {
        match self {
            V::List(l) => Rc::make_mut(l).push(value.into()),
            other => type_error(format!("'{}' object has no attribute 'append'", other.type_name())),
        }
    }

    pub fn list_mut(&mut self) -> &mut Vec<V> {
        match self {
            V::List(l) => Rc::make_mut(l),
            other => type_error(format!("list expected, got {}", other.type_name())),
        }
    }

    /// A shallow dict copy restricted to these keys (`{k: d.get(k) for k in keys}`).
    pub fn pick(&self, keys: &[&str]) -> V {
        let mut out = V::dict();
        for k in keys {
            out.set(k, self.get(k).clone());
        }
        out
    }
}

impl From<bool> for V {
    fn from(v: bool) -> V {
        V::Bool(v)
    }
}
impl From<i64> for V {
    fn from(v: i64) -> V {
        V::Int(v)
    }
}
impl From<i32> for V {
    fn from(v: i32) -> V {
        V::Int(v as i64)
    }
}
impl From<usize> for V {
    fn from(v: usize) -> V {
        V::Int(v as i64)
    }
}
impl From<f64> for V {
    fn from(v: f64) -> V {
        V::Float(v)
    }
}
impl From<&str> for V {
    fn from(v: &str) -> V {
        V::Str(Rc::from(v))
    }
}
impl From<String> for V {
    fn from(v: String) -> V {
        V::Str(Rc::from(v.as_str()))
    }
}
impl From<Rc<str>> for V {
    fn from(v: Rc<str>) -> V {
        V::Str(v)
    }
}
impl From<Vec<V>> for V {
    fn from(v: Vec<V>) -> V {
        V::List(Rc::new(v))
    }
}
impl From<&V> for V {
    fn from(v: &V) -> V {
        v.clone()
    }
}
impl<T: Into<V>> From<Option<T>> for V {
    fn from(v: Option<T>) -> V {
        match v {
            Some(x) => x.into(),
            None => V::None,
        }
    }
}
impl From<Vec<f64>> for V {
    fn from(v: Vec<f64>) -> V {
        V::List(Rc::new(v.into_iter().map(V::Float).collect()))
    }
}
impl From<Vec<i64>> for V {
    fn from(v: Vec<i64>) -> V {
        V::List(Rc::new(v.into_iter().map(V::Int).collect()))
    }
}
impl From<(f64, f64)> for V {
    fn from(v: (f64, f64)) -> V {
        V::list(vec![V::Float(v.0), V::Float(v.1)])
    }
}

/// `{"k": v, ...}` in insertion order.
#[macro_export]
macro_rules! dict {
    () => { $crate::py::V::dict() };
    ($($k:expr => $v:expr),+ $(,)?) => {{
        let mut d: Vec<($crate::py::Key, $crate::py::V)> = Vec::new();
        $( {
            let key: &str = $k;
            let value: $crate::py::V = ($v).into();
            if let Some(slot) = d.iter_mut().find(|(k, _)| &**k == key) {
                slot.1 = value;
            } else {
                d.push((std::rc::Rc::from(key), value));
            }
        } )+
        $crate::py::V::Dict(std::rc::Rc::new(d))
    }};
}

#[macro_export]
macro_rules! list {
    () => { $crate::py::V::list(Vec::new()) };
    ($($v:expr),+ $(,)?) => { $crate::py::V::list(vec![$(($v).into()),+]) };
}

/// Python `==`.
pub fn py_eq(a: &V, b: &V) -> bool {
    match (a, b) {
        (V::None, V::None) => true,
        (V::Str(x), V::Str(y)) => x == y,
        (V::List(x), V::List(y)) => {
            Rc::ptr_eq(x, y) || (x.len() == y.len() && x.iter().zip(y.iter()).all(|(p, q)| py_eq(p, q)))
        }
        (V::Dict(x), V::Dict(y)) => {
            Rc::ptr_eq(x, y)
                || (x.len() == y.len()
                    && x.iter().all(|(k, v)| match y.iter().find(|(k2, _)| k2 == k) {
                        Some((_, w)) => py_eq(v, w),
                        None => false,
                    }))
        }
        (V::Int(x), V::Int(y)) => x == y,
        (V::Bool(x), V::Bool(y)) => x == y,
        (V::Int(_) | V::Float(_) | V::Bool(_), V::Int(_) | V::Float(_) | V::Bool(_)) => num_eq(a, b),
        _ => false,
    }
}

fn num_eq(a: &V, b: &V) -> bool {
    match (a, b) {
        (V::Float(x), V::Float(y)) => x == y,
        (V::Float(x), other) | (other, V::Float(x)) => {
            let i = other.i();
            // Exact int/float comparison, as CPython does.
            x.is_finite() && x.fract() == 0. && (*x as i128) == i as i128 && (i as f64) == *x
        }
        _ => a.i() == b.i(),
    }
}

/// Equal with equal types throughout (autonomy_live._same).
pub fn same(a: &V, b: &V) -> bool {
    match (a, b) {
        (V::None, V::None) => true,
        (V::Bool(x), V::Bool(y)) => x == y,
        (V::Int(x), V::Int(y)) => x == y,
        (V::Float(x), V::Float(y)) => x == y,
        (V::Str(x), V::Str(y)) => x == y,
        (V::List(x), V::List(y)) => Rc::ptr_eq(x, y) || (x.len() == y.len() && x.iter().zip(y.iter()).all(|(p, q)| same(p, q))),
        (V::Dict(x), V::Dict(y)) => {
            Rc::ptr_eq(x, y) || x.len() == y.len()
                && x.iter().zip(y.iter()).all(|((k1, _), (k2, _))| k1 == k2)
                && x.iter().zip(y.iter()).all(|((_, v), (_, w))| same(v, w))
        }
        _ => false,
    }
}

// ---------------------------------------------------------------- JSON

pub fn to_json(value: &V) -> String {
    let mut out = String::new();
    write_json(value, &mut out);
    out
}

pub fn write_float(x: f64, out: &mut String) {
    if x.is_nan() {
        out.push_str("NaN");
    } else if x == f64::INFINITY {
        out.push_str("Infinity");
    } else if x == f64::NEG_INFINITY {
        out.push_str("-Infinity");
    } else {
        let start = out.len();
        let _ = write!(out, "{x:?}");
        let text = &out[start..];
        if !text.contains(['.', 'e', 'E']) {
            out.push_str(".0");
        }
    }
}

pub fn write_str(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 || (c as u32) > 0x7e => {
                let mut buf = [0u16; 2];
                for unit in c.encode_utf16(&mut buf) {
                    let _ = write!(out, "\\u{unit:04x}");
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

pub fn write_json(value: &V, out: &mut String) {
    match value {
        V::None => out.push_str("null"),
        V::Bool(true) => out.push_str("true"),
        V::Bool(false) => out.push_str("false"),
        V::Int(i) => {
            let _ = write!(out, "{i}");
        }
        V::Float(x) => write_float(*x, out),
        V::Str(s) => write_str(s, out),
        V::List(l) => {
            out.push('[');
            for (n, item) in l.iter().enumerate() {
                if n > 0 {
                    out.push_str(", ");
                }
                write_json(item, out);
            }
            out.push(']');
        }
        V::Dict(d) => {
            out.push('{');
            for (n, (k, v)) in d.iter().enumerate() {
                if n > 0 {
                    out.push_str(", ");
                }
                write_str(k, out);
                out.push_str(": ");
                write_json(v, out);
            }
            out.push('}');
        }
    }
}

pub fn parse_json(text: &str) -> Result<V, String> {
    let mut parser = Parser { s: text.as_bytes(), i: 0 };
    let value = parser.value()?;
    parser.ws();
    if parser.i != parser.s.len() {
        return Err(format!("trailing data at {}", parser.i));
    }
    Ok(value)
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn ws(&mut self) {
        while self.i < self.s.len() && matches!(self.s[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }

    fn eat(&mut self, word: &str) -> bool {
        if self.s[self.i..].starts_with(word.as_bytes()) {
            self.i += word.len();
            true
        } else {
            false
        }
    }

    fn value(&mut self) -> Result<V, String> {
        self.ws();
        let Some(&c) = self.s.get(self.i) else { return Err("unexpected end".into()) };
        match c {
            b'{' => {
                self.i += 1;
                let mut items: Vec<(Key, V)> = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b'}') {
                    self.i += 1;
                    return Ok(V::Dict(Rc::new(items)));
                }
                loop {
                    self.ws();
                    if self.s.get(self.i) != Some(&b'"') {
                        return Err(format!("key expected at {}", self.i));
                    }
                    let key = self.string()?;
                    self.ws();
                    if self.s.get(self.i) != Some(&b':') {
                        return Err(format!("':' expected at {}", self.i));
                    }
                    self.i += 1;
                    let value = self.value()?;
                    if let Some(slot) = items.iter_mut().find(|(k, _)| **k == *key) {
                        slot.1 = value;
                    } else {
                        items.push((Rc::from(key.as_str()), value));
                    }
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b'}') => {
                            self.i += 1;
                            return Ok(V::Dict(Rc::new(items)));
                        }
                        _ => return Err(format!("',' or '}}' expected at {}", self.i)),
                    }
                }
            }
            b'[' => {
                self.i += 1;
                let mut items = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b']') {
                    self.i += 1;
                    return Ok(V::list(items));
                }
                loop {
                    items.push(self.value()?);
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b']') => {
                            self.i += 1;
                            return Ok(V::list(items));
                        }
                        _ => return Err(format!("',' or ']' expected at {}", self.i)),
                    }
                }
            }
            b'"' => Ok(V::from(self.string()?)),
            b'n' if self.eat("null") => Ok(V::None),
            b't' if self.eat("true") => Ok(V::Bool(true)),
            b'f' if self.eat("false") => Ok(V::Bool(false)),
            b'N' if self.eat("NaN") => Ok(V::Float(f64::NAN)),
            b'I' if self.eat("Infinity") => Ok(V::Float(f64::INFINITY)),
            b'-' if self.s[self.i..].starts_with(b"-Infinity") => {
                self.i += 9;
                Ok(V::Float(f64::NEG_INFINITY))
            }
            _ => self.number(),
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.i += 1;
        let mut out = String::new();
        loop {
            let Some(&c) = self.s.get(self.i) else { return Err("unterminated string".into()) };
            self.i += 1;
            match c {
                b'"' => return Ok(out),
                b'\\' => {
                    let Some(&e) = self.s.get(self.i) else { return Err("bad escape".into()) };
                    self.i += 1;
                    match e {
                        b'"' => out.push('"'),
                        b'\\' => out.push('\\'),
                        b'/' => out.push('/'),
                        b'b' => out.push('\u{8}'),
                        b'f' => out.push('\u{c}'),
                        b'n' => out.push('\n'),
                        b'r' => out.push('\r'),
                        b't' => out.push('\t'),
                        b'u' => {
                            let mut code = self.hex4()?;
                            if (0xd800..0xdc00).contains(&code) && self.s[self.i..].starts_with(b"\\u") {
                                self.i += 2;
                                let low = self.hex4()?;
                                code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
                            }
                            out.push(char::from_u32(code).unwrap_or('\u{fffd}'));
                        }
                        _ => return Err("bad escape".into()),
                    }
                }
                _ => {
                    // Copy one UTF-8 sequence.
                    let start = self.i - 1;
                    let mut end = self.i;
                    while end < self.s.len() && (self.s[end] & 0xc0) == 0x80 {
                        end += 1;
                    }
                    out.push_str(std::str::from_utf8(&self.s[start..end]).map_err(|e| e.to_string())?);
                    self.i = end;
                }
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let digits = self.s.get(self.i..self.i + 4).ok_or("short \\u escape")?;
        self.i += 4;
        u32::from_str_radix(std::str::from_utf8(digits).map_err(|e| e.to_string())?, 16).map_err(|e| e.to_string())
    }

    fn number(&mut self) -> Result<V, String> {
        let start = self.i;
        if self.s.get(self.i) == Some(&b'-') {
            self.i += 1;
        }
        let mut float = false;
        while let Some(&c) = self.s.get(self.i) {
            match c {
                b'0'..=b'9' => {}
                b'.' | b'e' | b'E' | b'+' | b'-' => float = true,
                _ => break,
            }
            self.i += 1;
        }
        let text = std::str::from_utf8(&self.s[start..self.i]).map_err(|e| e.to_string())?;
        if text.is_empty() || text == "-" {
            return Err(format!("value expected at {start}"));
        }
        if float {
            text.parse::<f64>().map(V::Float).map_err(|e| format!("{e}: {text}"))
        } else {
            match text.parse::<i64>() {
                Ok(i) => Ok(V::Int(i)),
                Err(_) => text.parse::<f64>().map(V::Float).map_err(|e| e.to_string()),
            }
        }
    }
}

// ---------------------------------------------------------------- math

pub const PI: f64 = std::f64::consts::PI;
pub const INF: f64 = f64::INFINITY;

/// math.radians (x * (pi/180), the constant as CPython computes it).
#[inline]
pub fn radians(x: f64) -> f64 {
    x * (PI / 180.0)
}

/// math.degrees.
#[inline]
pub fn degrees(x: f64) -> f64 {
    x * (180.0 / PI)
}

/// round(x) for a float: ties to even, an int.
pub fn round(x: f64) -> i64 {
    if x.is_nan() {
        value_error("cannot convert float NaN to integer")
    }
    if x.is_infinite() {
        raise(Exc::OverflowError, "cannot convert float infinity to integer")
    }
    x.round_ties_even() as i64
}

/// round(x, n) for a float: the correctly rounded decimal (ties to even),
/// read back, as CPython's double_round does.
pub fn round_n(x: f64, n: usize) -> f64 {
    if x == 0. || !x.is_finite() {
        return x;
    }
    let text = format!("{x:.n$}");
    let value: f64 = text.parse().unwrap_or(x);
    if value == 0. { 0f64.copysign(x) } else { value }
}

/// Python float `a % b`.
pub fn fmod(a: f64, b: f64) -> f64 {
    if b == 0. {
        raise(Exc::ZeroDivisionError, "float modulo")
    }
    let mut m = a % b;
    if m != 0. {
        if (b < 0.) != (m < 0.) {
            m += b;
        }
    } else {
        m = 0f64.copysign(b);
    }
    m
}

/// Python int `a % b`.
pub fn imod(a: i64, b: i64) -> i64 {
    if b == 0 {
        raise(Exc::ZeroDivisionError, "integer modulo by zero")
    }
    let m = a % b;
    if m != 0 && ((m < 0) != (b < 0)) { m + b } else { m }
}

/// Python int `a // b`.
pub fn ifloordiv(a: i64, b: i64) -> i64 {
    if b == 0 {
        raise(Exc::ZeroDivisionError, "integer division by zero")
    }
    let q = a / b;
    if (a % b != 0) && ((a < 0) != (b < 0)) { q - 1 } else { q }
}

/// Python float `a / b` (ZeroDivisionError at 0).
#[inline]
pub fn div(a: f64, b: f64) -> f64 {
    if b == 0. {
        raise(Exc::ZeroDivisionError, "float division by zero")
    }
    a / b
}

/// `x**2` for a float: the C library pow, as CPython calls it.
pub fn sq(x: f64) -> f64 {
    let a = x.abs();
    if a == 1. {
        return 1.;
    }
    if !a.is_finite() || a == 0. {
        return a * a;
    }
    a.powf(std::hint::black_box(2.0))
}

/// `x**y` for floats with y a positive integer valued constant.
pub fn powf(x: f64, y: f64) -> f64 {
    let (base, negate) = if x < 0. && y.fract() == 0. { (-x, (y % 2.).abs() == 1.) } else { (x, false) };
    if base == 1. {
        return if negate { -1. } else { 1. };
    }
    let r = base.powf(std::hint::black_box(y));
    if negate { -r } else { r }
}

pub fn sqrt(x: f64) -> f64 {
    if x < 0. {
        value_error("math domain error")
    }
    x.sqrt()
}

pub fn asin(x: f64) -> f64 {
    if !(-1. ..=1.).contains(&x) && !x.is_nan() {
        value_error("math domain error")
    }
    x.asin()
}

pub fn acos(x: f64) -> f64 {
    if !(-1. ..=1.).contains(&x) && !x.is_nan() {
        value_error("math domain error")
    }
    x.acos()
}

pub fn ceil(x: f64) -> i64 {
    if !x.is_finite() {
        value_error("cannot convert float to integer")
    }
    x.ceil() as i64
}

pub fn floor(x: f64) -> i64 {
    if !x.is_finite() {
        value_error("cannot convert float to integer")
    }
    x.floor() as i64
}

/// math.isclose.
pub fn isclose(a: f64, b: f64, rel_tol: f64, abs_tol: f64) -> bool {
    if a == b {
        return true;
    }
    if a.is_infinite() || b.is_infinite() {
        return false;
    }
    let diff = (b - a).abs();
    diff <= (rel_tol * b).abs() || diff <= (rel_tol * a).abs() || diff <= abs_tol
}

#[inline]
fn dl_mul(x: f64, y: f64) -> (f64, f64) {
    let z = x * y;
    (z, x.mul_add(y, -z))
}

#[inline]
fn dl_fast_sum(a: f64, b: f64) -> (f64, f64) {
    let x = a + b;
    let z = x - a;
    (x, b - z)
}

/// CPython's vector_norm (math.hypot / math.dist).
fn vector_norm(vec: &mut [f64], max: f64, found_nan: bool) -> f64 {
    if max.is_infinite() {
        return max;
    }
    if found_nan {
        return f64::NAN;
    }
    if max == 0. || vec.len() <= 1 {
        return max;
    }
    let max_e = frexp_exp(max);
    if max_e < -1023 {
        for x in vec.iter_mut() {
            *x /= f64::MIN_POSITIVE;
        }
        return f64::MIN_POSITIVE * vector_norm(vec, max / f64::MIN_POSITIVE, found_nan);
    }
    let scale = ldexp(1.0, -max_e);
    let (mut csum, mut frac1, mut frac2) = (1.0f64, 0.0f64, 0.0f64);
    for &v in vec.iter() {
        let x = v * scale;
        let pr = dl_mul(x, x);
        let sm = dl_fast_sum(csum, pr.0);
        csum = sm.0;
        frac1 += pr.1;
        frac2 += sm.1;
    }
    let mut h = (csum - 1.0 + (frac1 + frac2)).sqrt();
    let pr = dl_mul(-h, h);
    let sm = dl_fast_sum(csum, pr.0);
    csum = sm.0;
    frac1 += pr.1;
    frac2 += sm.1;
    let x = csum - 1.0 + (frac1 + frac2);
    h += x / (2.0 * h);
    h / scale
}

/// frexp's exponent: x = m * 2**e with .5 <= |m| < 1.
fn frexp_exp(x: f64) -> i32 {
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7ff) as i32;
    if exp == 0 {
        // Subnormal: normalise.
        let y = x * f64::from_bits(0x4350000000000000); // 2**54
        frexp_exp(y) - 54
    } else {
        exp - 1022
    }
}

fn ldexp(x: f64, e: i32) -> f64 {
    let mut x = x;
    let mut e = e;
    while e > 1000 {
        x *= f64::from_bits(((1023 + 1000) as u64) << 52);
        e -= 1000;
    }
    while e < -1000 {
        x *= f64::from_bits(((1023 - 1000) as u64) << 52);
        e += 1000;
    }
    x * f64::from_bits(((1023 + e) as u64) << 52)
}

/// math.hypot(x, y).
pub fn hypot(x: f64, y: f64) -> f64 {
    let (a, b) = (x.abs(), y.abs());
    let found_nan = a.is_nan() || b.is_nan();
    let mut max = 0.0f64;
    for v in [a, b] {
        if v > max {
            max = v;
        }
    }
    let mut vec = [a, b];
    vector_norm(&mut vec, max, found_nan)
}

/// math.dist(p, q) for 2-D points.
pub fn dist(p: (f64, f64), q: (f64, f64)) -> f64 {
    hypot(p.0 - q.0, p.1 - q.1)
}

/// Built-in sum() of floats (Neumaier compensation, CPython >= 3.12).
pub fn fsum_py<I: IntoIterator<Item = f64>>(items: I) -> f64 {
    let mut iter = items.into_iter();
    let Some(first) = iter.next() else { return 0. };
    // sum() starts from int 0; the first float item gives 0 + x.
    let mut f = 0.0f64 + first;
    let mut c = 0.0f64;
    for x in iter {
        let t = f + x;
        if f.abs() >= x.abs() {
            c += (f - t) + x;
        } else {
            c += (x - t) + f;
        }
        f = t;
    }
    if c != 0. && c.is_finite() {
        f += c;
    }
    f
}

/// statistics.median of floats.
pub fn median(mut data: Vec<f64>) -> f64 {
    if data.is_empty() {
        value_error("no median for empty data")
    }
    sort_floats(&mut data);
    let n = data.len();
    if n % 2 == 1 { data[n / 2] } else { (data[n / 2 - 1] + data[n / 2]) / 2. }
}

/// Python's sort of floats (stable; NaN never occurs here).
pub fn sort_floats(data: &mut [f64]) {
    data.sort_by(|a, b| {
        if a < b {
            std::cmp::Ordering::Less
        } else if b < a {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });
}

/// statistics.mean of floats: the correctly rounded exact mean.
pub fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        value_error("mean requires at least one data point")
    }
    // Exact sum as an integer times 2**emin.
    let mut emin = i32::MAX;
    let mut parts: Vec<(bool, u64, i32)> = Vec::with_capacity(data.len());
    for &x in data {
        if !x.is_finite() {
            return data.iter().sum::<f64>() / data.len() as f64;
        }
        if x == 0. {
            continue;
        }
        let bits = x.to_bits();
        let neg = bits >> 63 == 1;
        let exp = ((bits >> 52) & 0x7ff) as i32;
        let frac = bits & ((1u64 << 52) - 1);
        let (m, e) = if exp == 0 { (frac, -1074) } else { (frac | (1u64 << 52), exp - 1075) };
        emin = emin.min(e);
        parts.push((neg, m, e));
    }
    if parts.is_empty() {
        return 0.;
    }
    let mut pos = Big::zero();
    let mut negs = Big::zero();
    for (neg, m, e) in parts {
        let term = Big::from_u64(m).shl((e - emin) as u32);
        if neg { negs.add(&term) } else { pos.add(&term) }
    }
    let (negative, magnitude) = if pos.cmp(&negs) != std::cmp::Ordering::Less {
        (false, pos.sub(&negs))
    } else {
        (true, negs.sub(&pos))
    };
    if magnitude.is_zero() {
        return 0.;
    }
    let value = magnitude.div_round(data.len() as u32, emin);
    if negative { -value } else { value }
}

/// Minimal unsigned big integer for statistics.mean.
#[derive(Clone)]
struct Big(Vec<u32>);

impl Big {
    fn zero() -> Big {
        Big(Vec::new())
    }

    fn from_u64(v: u64) -> Big {
        let mut b = Big(vec![v as u32, (v >> 32) as u32]);
        b.trim();
        b
    }

    fn trim(&mut self) {
        while self.0.last() == Some(&0) {
            self.0.pop();
        }
    }

    fn is_zero(&self) -> bool {
        self.0.is_empty()
    }

    fn bits(&self) -> u32 {
        match self.0.last() {
            None => 0,
            Some(&top) => (self.0.len() as u32 - 1) * 32 + (32 - top.leading_zeros()),
        }
    }

    fn shl(&self, n: u32) -> Big {
        let words = (n / 32) as usize;
        let bits = n % 32;
        let mut out = vec![0u32; words];
        let mut carry = 0u32;
        for &w in &self.0 {
            if bits == 0 {
                out.push(w);
            } else {
                out.push((w << bits) | carry);
                carry = w >> (32 - bits);
            }
        }
        if carry != 0 {
            out.push(carry);
        }
        let mut b = Big(out);
        b.trim();
        b
    }

    fn add(&mut self, other: &Big) {
        let n = self.0.len().max(other.0.len());
        self.0.resize(n, 0);
        let mut carry = 0u64;
        for i in 0..n {
            let s = self.0[i] as u64 + *other.0.get(i).unwrap_or(&0) as u64 + carry;
            self.0[i] = s as u32;
            carry = s >> 32;
        }
        if carry != 0 {
            self.0.push(carry as u32);
        }
    }

    fn sub(&self, other: &Big) -> Big {
        let mut out = self.0.clone();
        let mut borrow = 0i64;
        for (i, slot) in out.iter_mut().enumerate() {
            let d = *slot as i64 - *other.0.get(i).unwrap_or(&0) as i64 - borrow;
            if d < 0 {
                *slot = (d + (1i64 << 32)) as u32;
                borrow = 1;
            } else {
                *slot = d as u32;
                borrow = 0;
            }
        }
        let mut b = Big(out);
        b.trim();
        b
    }

    fn cmp(&self, other: &Big) -> std::cmp::Ordering {
        if self.0.len() != other.0.len() {
            return self.0.len().cmp(&other.0.len());
        }
        for i in (0..self.0.len()).rev() {
            if self.0[i] != other.0[i] {
                return self.0[i].cmp(&other.0[i]);
            }
        }
        std::cmp::Ordering::Equal
    }

    fn divmod_small(&self, d: u32) -> (Big, u32) {
        let mut out = vec![0u32; self.0.len()];
        let mut rem = 0u64;
        for i in (0..self.0.len()).rev() {
            let cur = (rem << 32) | self.0[i] as u64;
            out[i] = (cur / d as u64) as u32;
            rem = cur % d as u64;
        }
        let mut b = Big(out);
        b.trim();
        (b, rem as u32)
    }

    /// The double nearest to self * 2**exp / d (ties to even).
    fn div_round(&self, d: u32, exp: i32) -> f64 {
        // Scale so the quotient has at least 55 bits.
        let need: u32 = 56 + 32;
        let shift = need.saturating_sub(self.bits());
        let (q, r) = self.shl(shift).divmod_small(d);
        let exp = exp - shift as i32;
        let qb = q.bits();
        let drop = qb - 53;
        // mantissa = q >> drop; rest = low drop bits.
        let mut mant: u64 = 0;
        for k in 0..53u32 {
            let bit = drop + k;
            let w = (bit / 32) as usize;
            if (q.0[w] >> (bit % 32)) & 1 == 1 {
                mant |= 1u64 << k;
            }
        }
        let half_bit = drop - 1;
        let half = (q.0[(half_bit / 32) as usize] >> (half_bit % 32)) & 1 == 1;
        let mut below = r != 0;
        for bit in 0..half_bit {
            if (q.0[(bit / 32) as usize] >> (bit % 32)) & 1 == 1 {
                below = true;
                break;
            }
        }
        if half && (below || mant & 1 == 1) {
            mant += 1;
        }
        ldexp(mant as f64, exp + drop as i32)
    }
}

/// `max(a, b)` keeping the first of equals (Python).
#[inline]
pub fn pmax(a: f64, b: f64) -> f64 {
    if b > a { b } else { a }
}

/// `min(a, b)` keeping the first of equals (Python).
#[inline]
pub fn pmin(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `max(lo, min(hi, x))` for ints.
#[inline]
pub fn clampi(x: i64, lo: i64, hi: i64) -> i64 {
    lo.max(hi.min(x))
}

/// turn_motion._wrap: (angle + pi) % (2 pi) - pi.
#[inline]
pub fn wrap(angle: f64) -> f64 {
    fmod(angle + PI, 2. * PI) - PI
}

/// Index of the first maximal item (Python `max(..., key=...)`).
pub fn argmax_first<T>(items: &[T], key: impl Fn(&T) -> f64) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, item) in items.iter().enumerate() {
        let k = key(item);
        match best {
            None => best = Some((i, k)),
            Some((_, b)) if k > b => best = Some((i, k)),
            _ => {}
        }
    }
    best.map(|(i, _)| i)
}

/// Index of the first minimal item (Python `min(..., key=...)`).
pub fn argmin_first<T>(items: &[T], key: impl Fn(&T) -> f64) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (i, item) in items.iter().enumerate() {
        let k = key(item);
        match best {
            None => best = Some((i, k)),
            Some((_, b)) if k < b => best = Some((i, k)),
            _ => {}
        }
    }
    best.map(|(i, _)| i)
}

/// A float as V (int-valued results stay floats, as in Python).
#[inline]
pub fn fv(x: f64) -> V {
    V::Float(x)
}

/// `max(0, x)` as Python returns it: int 0 unless x > 0.
pub fn max0(x: f64) -> V {
    if x > 0. { V::Float(x) } else { V::Int(0) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_round_trip() {
        let text = r#"{"a": 1, "b": 1.0, "c": [null, true, -0.5, Infinity, -Infinity], "d": "x\"y", "e": 1e-07}"#;
        let v = parse_json(text).unwrap();
        assert!(v.get("a").is_int());
        assert!(v.get("b").is_float());
        assert_eq!(v.get("c").l()[3].f(), f64::INFINITY);
        let again = parse_json(&to_json(&v)).unwrap();
        assert!(same(&v, &again));
    }

    #[test]
    fn exceptions_unwind_and_are_caught() {
        install_panic_hook();
        let r = except(VTK, || value_error("x"));
        assert_eq!(r.unwrap_err().kind, Exc::ValueError);
        let r = py_try(|| raise(Exc::ZeroDivisionError, "y"));
        assert_eq!(r.unwrap_err().kind, Exc::ZeroDivisionError);
    }

    #[test]
    fn python_arithmetic() {
        assert_eq!(fmod(-1., 360.), 359.);
        assert_eq!(imod(-10, 360), 350);
        assert_eq!(ifloordiv(-7, 2), -4);
        assert_eq!(round(2.5), 2);
        assert_eq!(round(-2.5), -2);
        assert_eq!(round_n(0.125, 2), 0.12);
        assert_eq!(hypot(3., 4.), 5.);
        assert_eq!(mean(&[1., 2., 4.]), 7. / 3.);
        assert_eq!(median(vec![3., 1., 2., 4.]), 2.5);
        assert_eq!(fsum_py([0.1, 0.2, 0.3]), 0.6);
    }
}
