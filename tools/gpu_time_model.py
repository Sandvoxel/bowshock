#!/usr/bin/env python3
"""gpu_time_model.py: predicts the app's solve of the example rocket on common GPUs from two measured runs (DESIGN3D item 57).

    tools/gpu_time_model.py REF_LOG SLOW_LOG [--wgpu-factor=F]

REF_LOG and SLOW_LOG are job logs of `engine3d_draft_first` on a fast and a slower NVIDIA GPU (the RTX 3090 and the RTX 3080 Laptop, both through `gpuclk`): the `RUN` lines of its two
passes (see `perf.rs`: `RUN <Draft|Coarse> <dim>D <mach> <alpha> <cells> <iterations> <stop> <run seconds> <ms per iteration>`) and the `CLK` line (the clock the GPU held under load).

The model, per run of the solver (one Mach number on one domain), with `S` the throughput of a card relative to the reference:

    t_run = iterations * (a + k * cells / S) + f          (runs of more than 100 000 cells: the sectors and the half domains)
    t_run = iterations * w + f                             (the finless wedges of 7-22 k cells that seed the sectors: launch-latency bound)

* `a` (ms), `k` (ns per cell and iteration), `w` (ms per iteration) and `f` (s per run: grid build, upload, kernel compile) are the reference GPU's (`a`, `k` by least squares over the
  big runs of both passes);
* `S` is predicted from a spec of the card two ways and both are printed: by memory bandwidth, `S_bw = (BW / BW_ref)^q_bw`, and by FP32 throughput at the clock the card holds under load,
  `S_tf = (TF / TF_ref)^q_tf`; each exponent `q` is solved from the slower measured GPU's throughput ratio (one point each, so the exponent is a calibration, not a validation: the two
  measured GPUs have bandwidth and compute ratios within 10 % of each other and the data cannot say which one the step follows);
* the latency terms (`a`, `w`, `f`) scale with the card's own launch cost: 1 at the reference, the measured ratio of the slower GPU (`lat`) at its throughput, linear in log S between
  and beyond; `--wgpu-factor` multiplies them for a backend other than CUDA (measure it with the same GPU under `LUMA_BACKEND=wgpu`).

The times printed are the range of the two predictions. Cards and their numbers are the spec-sheet table below; for laptop parts the clock is the typical sustained one. Everything for a
card that was not measured is a prediction: +-25 % for NVIDIA cards on CUDA, wider for AMD, Apple and integrated GPUs (other drivers, caches, and unified memory).
"""
import json
import re
import sys
from math import log

# name, FP32 TFLOPs (spec boost, or the typical sustained clock for a laptop part), memory bandwidth GB/s, device memory GiB, backend
CARDS = [
    ("RTX 3090 (measured, 1965 MHz held)", 41.2, 936, 24, "CUDA"),
    ("RTX 3080 Laptop 16 GB (measured, 1302 MHz held)", 16.0, 384, 16, "CUDA"),
    ("RTX 4090", 82.6, 1008, 24, "CUDA"),
    ("RTX 4070 12 GB", 29.1, 504, 12, "CUDA"),
    ("RTX 3060 12 GB", 12.7, 360, 12, "CUDA"),
    ("RTX 4060 8 GB", 15.1, 272, 8, "CUDA"),
    ("RTX 4060 Laptop 8 GB", 11.0, 256, 8, "CUDA"),
    ("RTX 3060 Laptop 6 GB", 8.5, 336, 6, "CUDA"),
    ("RTX 3050 8 GB", 9.1, 224, 8, "CUDA"),
    ("GTX 1660 Super 6 GB", 5.0, 336, 6, "CUDA"),
    ("RX 7600 8 GB (dual-issue peak halved)", 10.9, 288, 8, "wgpu"),
    ("RX 6600 8 GB", 8.9, 224, 8, "wgpu"),
    ("RX 6700 XT 12 GB", 13.2, 384, 12, "wgpu"),
    ("Apple M1 (8-core GPU)", 2.6, 68, 8, "Metal"),
    ("Apple M2 (10-core GPU)", 3.6, 100, 8, "Metal"),
    ("Apple M1 Pro (16-core GPU)", 5.3, 200, 16, "Metal"),
    ("Apple M1 Max (32-core GPU)", 10.4, 400, 32, "Metal"),
    ("Apple M3 Max (40-core GPU)", 16.4, 400, 36, "Metal"),
    ("AMD 780M, Ryzen 7 7840U (12 CU)", 4.1, 100, 16, "wgpu"),
    ("Intel Iris Xe (96 EU)", 2.0, 60, 16, "wgpu"),
    ("Ryzen 9 7950X iGPU (2 CU; measured under wgpu: see the log)", 0.56, 83, 16, "wgpu"),
]

RUN = re.compile(r"^RUN (\w+) (\d)D ([\d.]+) ([\d.]+) (\d+) (\d+) (\w+) ([\d.]+)")
BIG = 100_000


def read(path):
    out = {"Draft": [], "Coarse": []}
    clk = None
    for line in open(path):
        m = RUN.match(line)
        if m:
            out[m.group(1)].append(dict(alpha=float(m.group(4)), cells=int(m.group(5)), it=int(m.group(6)), run_s=float(m.group(8))))
        elif line.startswith("CLK "):
            clk = json.loads(line[4:].replace("[N/A]", "null"))
    return out, clk


def fit(rs):
    """(a ms, k ns/cell/it, w ms/it) of one GPU's runs."""
    big = [r for r in rs if r["cells"] > BIG]
    small = [r for r in rs if r["cells"] <= BIG]
    xs, ys = [r["cells"] for r in big], [1000 * r["run_s"] / r["it"] for r in big]
    n, sx, sy = len(xs), sum(xs), sum(ys)
    k = (n * sum(x * y for x, y in zip(xs, ys)) - sx * sy) / (n * sum(x * x for x in xs) - sx * sx)
    a = (sy - k * sx) / n
    w = sum(1000 * r["run_s"] / r["it"] for r in small) / len(small)
    return a, k * 1e6, w


def predict(rs, a, k, w, f, s, lat):
    total = 0.0
    for r in rs:
        per_iter = w * lat if r["cells"] <= BIG else a * lat + k * r["cells"] / s / 1e6
        total += r["it"] * per_iter / 1000 + f * lat
    return total


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    wgpu = float(next((x.split("=")[1] for x in sys.argv[1:] if x.startswith("--wgpu-factor=")), 1.0))
    (ref, clk_r), (slow, clk_s) = read(args[0]), read(args[1])
    both = lambda d: d["Draft"] + d["Coarse"]
    a, k, w = fit(both(ref))
    a_s, k_s, w_s = fit(both(slow))
    f = sum(r["run_s"] - r["it"] * ((w if r["cells"] <= BIG else a + k * r["cells"] / 1e6) / 1000) for r in both(ref)) / len(both(ref))
    s_meas, lat = k / k_s, w_s / w
    (_, tf_r, bw_r, _, _), (_, tf_s, bw_s, _, _) = CARDS[0], CARDS[1]
    q_bw, q_tf = log(s_meas) / log(bw_s / bw_r), log(s_meas) / log(tf_s / tf_r)
    print(f"reference: a {a:.2f} ms, k {k:.2f} ns/cell/it, wedge {w:.2f} ms/it, fixed {f:.2f} s/run; clock {clk_r and clk_r['sm_mhz_mean']} MHz")
    print(f"slower:    a {a_s:.2f} ms, k {k_s:.2f} ns/cell/it, wedge {w_s:.2f} ms/it; clock {clk_s and clk_s['sm_mhz_mean']} MHz")
    print(f"throughput ratio {s_meas:.3f} (bandwidth ratio {bw_s / bw_r:.3f}, FP32 ratio {tf_s / tf_r:.3f}); exponents q_bw {q_bw:.2f}, q_tf {q_tf:.2f}; latency factor {lat:.2f}")
    print()
    print("| card | FP32 TF | BW GB/s | S (bw / fp32) | Coarse + angles s | Coarse drag only s | Draft drag only s (first result) | backend |")
    print("|---|---|---|---|---|---|---|---|")
    for name, tf, bw, _, backend in CARDS:
        row = []
        for s in ((bw / bw_r) ** q_bw, (tf / tf_r) ** q_tf):
            latency = 1.0 if s >= 1 else 1 + (lat - 1) * log(1 / s) / log(1 / s_meas)
            if backend != "CUDA":
                latency *= wgpu
            row.append(
                (
                    predict(ref["Coarse"], a, k, w, f, s, latency),
                    predict([r for r in ref["Coarse"] if r["alpha"] == 0], a, k, w, f, s, latency),
                    predict(ref["Draft"], a, k, w, f, s, latency),
                )
            )
        span = lambda i: f"{min(r[i] for r in row):.0f}-{max(r[i] for r in row):.0f}"
        print(f"| {name} | {tf:.1f} | {bw} | {(bw / bw_r) ** q_bw:.2f} / {(tf / tf_r) ** q_tf:.2f} | {span(0)} | {span(1)} | {span(2)} | {backend} |")


if __name__ == "__main__":
    main()
