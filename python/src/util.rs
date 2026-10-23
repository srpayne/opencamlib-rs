use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;

/// Format like C++ `std::ostream << double` with default settings (`%g`, 6 significant
/// digits), so `__str__` output matches the C++ bindings.
pub fn g(x: f64) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if x == 0.0 {
        return if x.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let sci = format!("{:.5e}", x);
    let (mant, exp) = sci.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    if !(-4..6).contains(&exp) {
        format!(
            "{}e{}{:02}",
            strip_zeros(mant),
            if exp < 0 { '-' } else { '+' },
            exp.abs()
        )
    } else {
        strip_zeros(&format!("{:.*}", (5 - exp) as usize, x)).to_string()
    }
}

/// Re-format every number in `s` with [`g`], e.g. "BallCutter(d=2.5450000000000004, ..."
/// becomes "BallCutter(d=2.545, ...", as the C++ library prints it.
pub fn g_all(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut num = String::new();
    let flush = |num: &mut String, out: &mut String| {
        if !num.is_empty() {
            match num.parse::<f64>() {
                Ok(v) => out.push_str(&g(v)),
                Err(_) => out.push_str(num),
            }
            num.clear();
        }
    };
    for ch in s.chars() {
        let continues = ch.is_ascii_digit()
            || (ch == '.' && !num.is_empty())
            || (ch == '-' && (num.is_empty() || num.ends_with('e')))
            || (ch == 'e' && !num.is_empty() && !num.contains('e'));
        if continues {
            num.push(ch);
        } else {
            flush(&mut num, &mut out);
            out.push(ch);
        }
    }
    flush(&mut num, &mut out);
    out
}

fn strip_zeros(s: &str) -> &str {
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.')
    } else {
        s
    }
}

/// The value of a setter that must be called before `run()`.
pub fn need<'a, T>(o: &'a Option<T>, setter: &str) -> PyResult<&'a T> {
    o.as_ref()
        .ok_or_else(|| PyRuntimeError::new_err(format!("{setter} must be called before run()")))
}

#[cfg(test)]
mod tests {
    use super::g;

    #[test]
    fn formats_like_cpp() {
        assert_eq!(g(1.0), "1");
        assert_eq!(g(0.5), "0.5");
        assert_eq!(g(1.0 / 3.0), "0.333333");
        assert_eq!(g(123456.7), "123457");
        assert_eq!(g(1234567.0), "1.23457e+06");
        assert_eq!(g(0.0001), "0.0001");
        assert_eq!(g(0.00001234), "1.234e-05");
        assert_eq!(g(-2.5), "-2.5");
        assert_eq!(g(999999.7), "1e+06");
        assert_eq!(
            super::g_all("BallCutter(d=2.5450000000000004, r=1.2725000000000002, L=6.1)"),
            "BallCutter(d=2.545, r=1.2725, L=6.1)"
        );
        assert_eq!(super::g_all("r1=1 r2=-0.25"), "r1=1 r2=-0.25");
    }
}
