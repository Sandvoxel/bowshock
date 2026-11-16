//! Perf ledger check: groups `perf/ledger.jsonl` by (host, case, precision), prints a Markdown table and exits 1 when the latest
//! entry of any group is more than 10 % above the best prior one.
//!
//! `cargo run --release -p bowshock-validate --example perfcheck -- [perf/ledger.jsonl]`
//!
//! The metric is `us_per_iter`; entries without it (e.g. the `3d-anf-L0-table` entry) are compared on `wall_s`.

use serde_json::Value;

const LIMIT: f64 = 1.10;

struct Entry {
    value: f64,
    sha: String,
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "perf/ledger.jsonl".into());
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("perfcheck: cannot read {path}: {e}");
            std::process::exit(2);
        }
    };
    let mut groups: Vec<((String, String, String, &'static str), Vec<Entry>)> = Vec::new();
    for (n, line) in text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty()) {
        let v: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("perfcheck: {path}:{}: {e}", n + 1);
                continue;
            }
        };
        let s = |k: &str| v.get(k).and_then(Value::as_str).unwrap_or("?").to_string();
        let (unit, value) = match (v.get("us_per_iter").and_then(Value::as_f64), v.get("wall_s").and_then(Value::as_f64)) {
            (Some(us), _) => ("us/iter", us),
            (None, Some(w)) => ("wall s", w),
            _ => {
                eprintln!("perfcheck: {path}:{}: no us_per_iter or wall_s", n + 1);
                continue;
            }
        };
        let key = (s("host"), s("case"), s("precision"), unit);
        let sha = s("sha").chars().take(8).collect();
        match groups.iter_mut().find(|(k, _)| *k == key) {
            Some((_, e)) => e.push(Entry { value, sha }),
            None => groups.push((key, vec![Entry { value, sha }])),
        }
    }

    println!("| host | case | precision | metric | latest | previous | best prior | Δ% vs best prior | sha |");
    println!("|---|---|---|---|---|---|---|---|---|");
    let mut regressed = false;
    for ((host, case, precision, unit), e) in &groups {
        let (last, prior) = e.split_last().expect("non-empty group");
        let (prev, best, delta) = match prior.last() {
            None => ("-".to_string(), "-".to_string(), "-".to_string()),
            Some(p) => {
                let best = prior.iter().map(|x| x.value).fold(f64::INFINITY, f64::min);
                let bad = last.value > LIMIT * best;
                regressed |= bad;
                (
                    format!("{:.1}", p.value),
                    format!("{best:.1}"),
                    format!("{:+.1}{}", 100.0 * (last.value / best - 1.0), if bad { " REGRESSION" } else { "" }),
                )
            }
        };
        println!("| {host} | {case} | {precision} | {unit} | {:.1} | {prev} | {best} | {delta} | {} |", last.value, last.sha);
    }
    if regressed {
        eprintln!("perfcheck: regression above {:.0} % against the best prior entry", 100.0 * (LIMIT - 1.0));
        std::process::exit(1);
    }
}
