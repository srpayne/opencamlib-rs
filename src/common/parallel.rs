//! Thread-count aware helpers (rayon replaces OpenMP from the C++ version).

/// Number of threads used by default: the number of available CPUs.
pub(crate) fn default_threads() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
}

/// Apply `f` to every element of `items`, using up to `threads` threads.
pub(crate) fn for_each_mut<T, F>(threads: usize, items: &mut [T], f: F)
where
    T: Send,
    F: Fn(&mut T) + Sync + Send,
{
    #[cfg(feature = "parallel")]
    {
        if threads > 1 && items.len() > 1 {
            if let Ok(pool) = rayon::ThreadPoolBuilder::new().num_threads(threads).build() {
                use rayon::prelude::*;
                pool.install(|| items.par_iter_mut().for_each(&f));
                return;
            }
        }
    }
    let _ = threads;
    items.iter_mut().for_each(f);
}

/// Run `a` and `b`, in parallel if `threads > 1`.
pub(crate) fn join<A, B, RA, RB>(threads: usize, a: A, b: B) -> (RA, RB)
where
    A: FnOnce() -> RA + Send,
    B: FnOnce() -> RB + Send,
    RA: Send,
    RB: Send,
{
    #[cfg(feature = "parallel")]
    {
        if threads > 1 {
            return rayon::join(a, b);
        }
    }
    let _ = threads;
    (a(), b())
}
