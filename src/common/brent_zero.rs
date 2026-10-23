//! Brent's root-finding method.

/// Find a zero of `f` in the interval `[a, b]`, where `f(a)` and `f(b)` have opposite signs.
///
/// `eps` is the relative and `t` the absolute tolerance. This follows Brent's ZERO
/// algorithm as used by the C++ library.
pub fn brent_zero<F: FnMut(f64) -> f64>(mut a: f64, mut b: f64, eps: f64, t: f64, mut f: F) -> f64 {
    let mut fa = f(a);
    let mut fb = f(b);
    let mut c = a;
    let mut fc = fa;
    let mut e = b - a;
    let mut d = e;
    loop {
        if fc.abs() < fb.abs() {
            // c is better than b: swap so that b is the best root so far
            a = b;
            b = c;
            c = a;
            fa = fb;
            fb = fc;
            fc = fa;
        }
        let tol = 2.0 * eps * b.abs() + t;
        let m = 0.5 * (c - b);
        if m.abs() <= tol || fb == 0.0 {
            break;
        }
        if e.abs() < tol || fa.abs() <= fb.abs() {
            // bisection
            e = m;
            d = e;
        } else {
            let mut s = fb / fa;
            let mut p;
            let mut q;
            if a == c {
                // linear interpolation
                p = 2.0 * m * s;
                q = 1.0 - s;
            } else {
                // inverse quadratic interpolation. Note: the C++ library uses `a` where
                // textbook Brent has `q` in the first term; kept for identical results.
                // The step is still guarded below, so convergence is unaffected.
                q = fa / fc;
                let r = fb / fc;
                p = s * (2.0 * m * a * (q - r) - (b - a) * (r - 1.0));
                q = (q - 1.0) * (r - 1.0) * (s - 1.0);
            }
            if p > 0.0 {
                q = -q;
            } else {
                p = -p;
            }
            s = e;
            e = d;
            if 2.0 * p < 3.0 * m * q - (tol * q).abs() && p < (0.5 * s * q).abs() {
                d = p / q;
            } else {
                e = m;
                d = e;
            }
        }
        a = b;
        fa = fb;
        if d.abs() > tol {
            b += d;
        } else if m > 0.0 {
            b += tol;
        } else {
            b -= tol;
        }
        fb = f(b);
        if fb.is_sign_negative() == fc.is_sign_negative() {
            c = a;
            fc = fa;
            e = b - a;
            d = e;
        }
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_sqrt2() {
        let r = brent_zero(0.0, 2.0, 1e-16, 1e-12, |x| x * x - 2.0);
        assert!((r - 2f64.sqrt()).abs() < 1e-11);
    }
}
