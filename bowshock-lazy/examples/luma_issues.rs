//! Minimal reproductions of the Luma problems met while writing the solver (see README, "Luma issues").
//!
//! `cargo run --release --example luma_issues`

use luma::{Array, Df32};

fn main() {
    let ctx = luma::default_context().unwrap();
    let df = |x: &[f64]| Array::<Df32>::from_slice_on(&ctx, &x.iter().map(|&v| Df32::from(v)).collect::<Vec<_>>(), x.len()).unwrap();

    println!("== 1. a Df32 kernel that only compares / abs / min / max and has no scalar operand cannot bind its uniform");
    let (a, b) = (df(&[1.0, -2.0, 3.0]), df(&[2.0, 1.0, -3.0]));
    for (name, r) in [
        ("a.minimum(Df32::from(0.5))  (a scalar operand: works)", a.minimum(Df32::from(0.5)).to_vec().map(|v| v.iter().map(|d| d.hi).collect::<Vec<_>>())),
        ("(&a - &b).abs()  (also an arithmetic op: works)", (&a - &b).abs().to_vec().map(|v| v.iter().map(|d| d.hi).collect::<Vec<_>>())),
        ("a.abs()", a.abs().to_vec().map(|v| v.iter().map(|d| d.hi).collect::<Vec<_>>())),
        ("a.minimum(&b)", a.minimum(&b).to_vec().map(|v| v.iter().map(|d| d.hi).collect::<Vec<_>>())),
        ("a.lt(&b)", a.lt(&b).to_vec().map(|v| v.iter().map(|&m| m as f32).collect::<Vec<_>>())),
    ] {
        match r {
            Ok(v) => println!("   {name}: {v:?}"),
            Err(e) => println!("   {name}: ERROR {}", e.to_string().replace('\n', " ")),
        }
    }
    println!("   afterwards, (&a - &b).abs() again: {:?}", (&a - &b).abs().to_vec().map(|v| v.iter().map(|d| d.hi).collect::<Vec<_>>()));

    println!("== 2. x - x.to_f32().to_df32() is zero when fused into one kernel (the cast `x.hi + x.lo` is not guarded)");
    let x = Array::<Df32>::from_slice_on(&ctx, &[Df32::from(1.0 / 3.0), Df32::from(2.0 / 3.0)], 2).unwrap();
    let hi = x.to_f32();
    let lo = (&x - hi.to_df32()).to_f32();
    println!("   lo alone, fused:      {:?}   (want [{:e}, {:e}])", lo.to_vec().unwrap(), Df32::from(1.0 / 3.0).lo, Df32::from(2.0 / 3.0).lo);
    let hi = x.to_f32().cache();
    let lo = (&x - hi.to_df32()).to_f32();
    println!("   after hi.cache():     {:?}", lo.to_vec().unwrap());

    println!("== 3. an expression that reads 14 distinct buffers is cut for \"21 input buffers > 16\" (one kernel becomes 69)");
    type A = Array<f32>;
    // a chain of `depth` mixed operations over `xs`
    fn mix(xs: &[&A], seed: usize, depth: usize) -> A {
        let mut acc = xs[seed % xs.len()] * 1.0;
        for k in 0..depth {
            let y = xs[(seed + k + 1) % xs.len()];
            acc = match k % 4 {
                0 => &acc * y + 0.1,
                1 => (&acc - y).abs() + 0.2,
                2 => &acc / (y + 1.5),
                _ => &acc + y * 0.5,
            };
        }
        acc
    }
    let n = 20000usize;
    let w: Vec<A> = (0..4).map(|k| Array::from_slice_on(&ctx, &(0..n + 4).map(|i| 1.0 + 0.1 * ((i + k) as f32 * 0.37).sin()).collect::<Vec<_>>(), n + 4).unwrap()).collect();
    let (nx, nr) = (Array::from_slice_on(&ctx, &vec![0.5f32; n], n).unwrap(), Array::from_slice_on(&ctx, &vec![0.7f32; n], n).unwrap());
    for cache_s in [false, true] {
        let build = || {
            let m1: Vec<A> = w.iter().map(|a| a.slice(0, 1..n + 1)).collect();
            let p0: Vec<A> = w.iter().map(|a| a.slice(0, 2..n + 2)).collect();
            let views: Vec<&A> = m1.iter().chain(p0.iter()).collect();
            // l, r: 8 arrays from the 8 views, evaluated together (one kernel)
            let l: Vec<A> = (0..4).map(|k| mix(&views, k, 12).cache()).collect();
            let r: Vec<A> = (0..4).map(|k| mix(&views, k + 4, 12).cache()).collect();
            let lr: Vec<&A> = l.iter().chain(r.iter()).chain([&nx, &nr]).collect();
            // s: three more arrays from l, r (optionally evaluated)
            let s: Vec<A> = (0..3).map(|k| if cache_s { mix(&lr, k, 16).cache() } else { mix(&lr, k, 16) }).collect();
            let lrs: Vec<&A> = lr.iter().copied().chain(s.iter()).collect();
            // f: four arrays from l, r, nx, nr, s (13 buffers), times t from two of the first views (the 14th buffer)
            let f: Vec<A> = (0..4).map(|k| mix(&lrs, k, 24)).collect();
            let t = (&p0[3] - &m1[3]).abs() / m1[3].minimum(&p0[3]);
            for o in f.iter().map(|f| f * &t) {
                o.cache();
            }
        };
        build();
        ctx.sync().unwrap();
        let (_, profile) = ctx.profile(|| {
            build();
            ctx.sync().unwrap()
        })
        .unwrap();
        println!("   s evaluated: {cache_s:5}: {} kernel launches, {} fusion cuts; first: {}", profile.launches(), profile.splits.len(), profile.splits.first().map_or(String::new(), |s| s.to_string()));
    }
}
