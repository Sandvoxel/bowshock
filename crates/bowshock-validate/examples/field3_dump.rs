//! Solves a 3D Army-Navy Basic Finner case on the GPU and writes its final flow as a `bowshock_core::Field3` (CBOR, raw deflate): the
//! fixture of the app's flow viewer tests and drive scripts.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example field3_dump -- [--case tiny|level0] [--mach 2] [--alpha 4] [--out out/field3/anf.field3]
//! ```
//!
//! `tiny` is `Case3::tiny_anf` (about 9 000 cells, the committed fixture), `level0` the ANF grid of level 0 (about 10 times larger). Prints a
//! `FIELD3` line with the iterations, the stop reason, CN and the cost of building the `Field3`. Runs through `tools/gpuq` like every GPU job:
//! `tools/gpuq/gpuq submit --host local -- cargo run --release -p bowshock-validate --example field3_dump -- --out out/field3/anf.field3`.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::field3::field3_of;
use bowshock_core::engine::gpu3::GpuRunner3;
use std::io::Write;
use std::time::Instant;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let case_name = value("--case").unwrap_or_else(|| "tiny".into());
    let mach: f64 = value("--mach").map_or(Ok(2.0), |v| v.parse().map_err(|e| format!("--mach: {e}")))?;
    let alpha_deg: f64 = value("--alpha").map_or(Ok(4.0), |v| v.parse().map_err(|e| format!("--alpha: {e}")))?;
    let out = value("--out").unwrap_or_else(|| "out/field3/anf.field3".into());
    let alpha = alpha_deg.to_radians();
    let case = match case_name.as_str() {
        "tiny" => Case3::tiny_anf(mach, alpha),
        "level0" => Case3::new(mach, alpha, Case3::anf_body(), Some(Case3::anf_fins()), 0),
        other => return Err(format!("--case tiny|level0, got `{other}`")),
    };
    let mut runner = GpuRunner3::new(&case, Precision::F32).map_err(|e| e.to_string())?;
    let t0 = Instant::now();
    let r = runner.run_on(&case, false).map_err(|e| e.to_string())?;
    let solve_s = t0.elapsed().as_secs_f64();
    let u = runner.gpu.state_f64().map_err(|e| e.to_string())?;
    // scale 1: the fixture keeps the solver's lengths, in nose diameters (the app scales a viewer picture to the body it shows)
    let t1 = Instant::now();
    let field = field3_of(&case, &runner.host.grid, &u, r.iterations, 1.0);
    let build_s = t1.elapsed().as_secs_f64();
    let t2 = Instant::now();
    let mut cbor = Vec::new();
    ciborium::into_writer(&field, &mut cbor).map_err(|e| e.to_string())?;
    let mut enc = flate2::write::DeflateEncoder::new(Vec::new(), flate2::Compression::new(6));
    enc.write_all(&cbor).map_err(|e| e.to_string())?;
    let bytes = enc.finish().map_err(|e| e.to_string())?;
    let encode_s = t2.elapsed().as_secs_f64();
    if let Some(dir) = std::path::Path::new(&out).parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    std::fs::write(&out, &bytes).map_err(|e| e.to_string())?;
    println!(
        "FIELD3 {{\"case\":\"{case_name}\",\"mach\":{mach},\"alpha_deg\":{alpha_deg},\"cells\":{},\"walls\":{},\"iterations\":{},\"stop\":\"{:?}\",\"cn\":{:.5},\"x_cp\":{:.4},\"solve_s\":{solve_s:.2},\"field3_of_ms\":{:.2},\"encode_ms\":{:.1},\"bytes\":{},\"out\":\"{out}\"}}",
        field.cells(),
        field.walls.len(),
        r.iterations,
        r.stop_reason,
        r.cn,
        r.x_cp,
        1e3 * build_s,
        1e3 * encode_s,
        bytes.len()
    );
    Ok(())
}
