#!/usr/bin/env python3
"""gpu_time_model.py: predicts the full solve of the example rocket on common GPUs from two measured runs (DESIGN3D item 57).

    tools/gpu_time_model.py REF_LOG SLOW_LOG [--wgpu-factor=F]

REF_LOG and SLOW_LOG are job logs of `engine3d_draft_first` on a fast and a slower NVIDIA GPU (the RTX 3090 and the RTX 3080 Laptop): the `RUN` lines of its two passes (see
`perf.rs`): `RUN <Draft|Coarse> <dim>D <mach> <alpha> <cells> <iterations> <stop> <run seconds> <ms per iteration>`.

The model, per run of the solver (one Mach number on one domain):

    t_run = iterations * (a + k * cells / S) + f          (runs of more than 100 000 cells)
    t_run = iterations * w + f                             (the finless wedges of 7-22 k cells: launch-latency bound)

* `a` (ms), `k` (ns per cell and iteration), `w` (ms per iteration) and `f` (s per run: grid build, upload, kernel compile) come from the reference GPU's runs (`k` and `a` by least
  squares over the big runs);
* `S` is the throughput of a card relative to the reference, `S = (TF / TF_ref)^p * (BW / BW_ref)^(1 - p)` with TF the FP32 peak (spec boost clock) and BW the memory bandwidth; the
  exponent `p` is solved from the slower measured GPU's ratio of `k`: a one-parameter fit to one ratio, there is no second point to validate it on;
* the latency terms (`a`, `w`, `f`) scale with the card's own launch cost: 1 at the reference, the measured ratio of the slower GPU (`lat`) at its throughput, linear in log S between
  and beyond; `--wgpu-factor` multiplies them for a backend other than CUDA (measure it with the same GPU under `LUMA_BACKEND=wgpu`).

Cards and their numbers are the spec-sheet table below. Everything for a card that was not measured is a prediction (+-25 % for NVIDIA cards, wider for the others).
"""
import re
import sys
from math import log

# name, FP32 TFLOPs (spec boost), memory bandwidth GB/s, device memory GB, backend
CARDS = [
    ("RTX 3090 (measured)", 35.6, 936, 24, "CUDA"),
    ("RTX 3080 Laptop 16 GB (measured)", 21.0, 384, 16, "CUDA"),
    ("RTX 4090", 82.6, 1008, 24, "CUDA"),
    ("RTX 4070 12 GB", 29.1, 504, 12, "CUDA"),
    ("RTX 3060 12 GB", 12.7, 360, 12, "CUDA"),
    ("RTX 4060 8 GB", 15.1, 272, 8, "CUDA"),
    ("RTX 4060 Laptop 8 GB", 12.0, 256, 8, "CUDA"),
    ("RTX 3060 Laptop 6 GB", 10.9, 336, 6, "CUDA"),
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
    ("Ryzen 9 7950X iGPU (2 CU, measured under wgpu)", 0.56, 83, 16, "wgpu"),
]

RUN = re.compile(r"^RUN (\w+) (\d)D ([\d.]+) ([\d.]+) (\d+) (\d+) (\w+) ([\d.]+)")
BIG = 100_000


def runs(path):
    out = {"Draft": [], "Coarse": []}
    for line in open(path):
        m = RUN.match(line)
        if m:
            out[m.group(1)].append(dict(alpha=float(m.group(4)), cells=int(m.group(5)), it=int(m.group(6)), run_s=float(m.group(8))))
    return out


def fit(rs):
    """(a ms, k ns/cell/it, w ms/it, f s/run) of one GPU's runs."""
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
    wgpu = float(dict(x.split("=") for x in sys.argv[1:] if x.startswith("--"))["--wgpu-factor"]) if any(x.startswith("--wgpu-factor") for x in sys.argv) else 1.0
    ref, slow = runs(args[0]), runs(args[1])
    a, k, w = fit(ref["Coarse"])
    a_s, k_s, w_s = fit(slow["Coarse"])
    # the fixed cost per run: what the iterations do not explain, on the reference
    f = sum(r["run_s"] - r["it"] * ((w if r["cells"] <= BIG else a + k * r["cells"] / 1e6) / 1000) for r in ref["Coarse"]) / len(ref["Coarse"])
    s_meas, lat = k / k_s, w_s / w
    (_, tf_r, bw_r, _, _), (_, tf_s, bw_s, _, _) = CARDS[0], CARDS[1]
    p = (log(s_meas) - log(bw_s / bw_r)) / (log(tf_s / tf_r) - log(bw_s / bw_r))
    print(f"reference: a {a:.2f} ms, k {k:.2f} ns/cell/it, wedge {w:.2f} ms/it, fixed {f:.2f} s/run")
    print(f"slower:    a {a_s:.2f} ms, k {k_s:.2f} ns/cell/it, wedge {w_s:.2f} ms/it; throughput {s_meas:.3f}, latency factor {lat:.2f}, exponent p {p:.2f}")
    print()
    print("| card | FP32 TF | BW GB/s | S | Coarse full solve s | Coarse drag only s | Draft drag only s | backend |")
    print("|---|---|---|---|---|---|---|---|")
    for name, tf, bw, _, backend in CARDS:
        s = (tf / tf_r) ** p * (bw / bw_r) ** (1 - p)
        latency = 1.0 if s >= 1 else 1 + (lat - 1) * log(1 / s) / log(1 / s_meas)
        if backend != "CUDA":
            latency *= wgpu
        full = predict(ref["Coarse"], a, k, w, f, s, latency)
        drag = predict([r for r in ref["Coarse"] if r["alpha"] == 0], a, k, w, f, s, latency)
        draft = predict(ref["Draft"], a, k, w, f, s, latency)
        print(f"| {name} | {tf:.1f} | {bw} | {s:.2f} | {full:.0f} | {drag:.0f} | {draft:.0f} | {backend} |")


if __name__ == "__main__":
    main()
