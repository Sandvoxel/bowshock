#!/usr/bin/env python3
"""Post-processing of the `subdrag` test logs (DESIGN3D item 62).
  subdrag_post.py fox            boattail pressure drag of the Fox TM X-2439 data (tangent ogive, 10 degree boattail, sting 0.13 d) from the tabulated wall Cp
  subdrag_post.py cf LOG VARIANT local Cf of a `sd1` log against the flat-plate estimate of subdrag_estimate.py"""
import math, re, sys, os
sys.path.insert(0, os.path.dirname(__file__))
import subdrag_estimate as est

X = [0.289, 0.512, 0.740, 0.972, 1.207, 1.443, 1.682, 1.921, 1.965, 2.008, 2.051, 2.321, 2.657, 2.993, 3.327, 3.659, 3.927, 3.972, 4.014, 4.057, 4.348, 4.638, 4.928, 4.971]
CP = {
    0.404: [0.2837, 0.1283, 0.0291, -0.0631, -0.1347, -0.1838, -0.1838, -0.1578, -0.1352, -0.1195, -0.0989, -0.0584, -0.0401, -0.0382, -0.0571, -0.0868, -0.2346, -0.3326, -0.3495, -0.2257, -0.0369, 0.0242, 0.0621, 0.0698],
    0.608: [0.3075, 0.1450, 0.0410, -0.0606, -0.1407, -0.1959, -0.1975, -0.1660, -0.1437, -0.1251, -0.1017, -0.0548, -0.0359, -0.0337, -0.0521, -0.0843, -0.2534, -0.3678, -0.3893, -0.2465, -0.0299, 0.0339, 0.0771, 0.0867],
    0.831: [0.3813, 0.2063, 0.0873, -0.0311, -0.1293, -0.1988, -0.2110, -0.1671, -0.1402, -0.1128, -0.0842, -0.0256, -0.0024, -0.0024, -0.0209, -0.0527, -0.2244, -0.3507, -0.9392, -0.3141, 0.0068, 0.0783, 0.1254, 0.1297],
}
BASE_CP = {0.404: 0.0691, 0.608: 0.0836, 0.831: 0.1317}
STING_R, BOATTAIL = 0.13, math.radians(10)


def fox():
    """Boattail pressure drag on S = pi d^2 / 4 (d = 1): -int Cp d(A/S), A the projected area 4 r^2, trapezoid over the orifices from the start of the boattail (x = 4, the first
    point's Cp) to the base (x = 5, the last orifice's Cp). M 0.831: the 4.014 orifice reads -0.94 (a shock-induced spike of the data, kept: it is 1.5 % of the area)."""
    r = lambda x: 0.5 - (x - 4.0) * math.tan(BOATTAIL)
    for m, cp in CP.items():
        pts = [(x, c) for x, c in zip(X, cp) if x > 3.99]
        xs = [4.0] + [p[0] for p in pts] + [5.0]
        cs = [pts[0][1]] + [p[1] for p in pts] + [pts[-1][1]]
        d = 0.0
        for (x0, c0), (x1, c1) in zip(zip(xs, cs), zip(xs[1:], cs[1:])):
            d += -0.5 * (c0 + c1) * (4 * r(x0) ** 2 - 4 * r(x1) ** 2)
        base = (4 * r(5.0) ** 2 - 4 * STING_R**2)
        print(f"Fox M {m}: boattail drag {d:+.4f} (S), base annulus {base:.3f} S x Cp_b {BASE_CP[m]:.4f} = {-base * BASE_CP[m]:+.4f}, boattail + base {d - base * BASE_CP[m]:+.4f}")


def cf(log, variant):
    rows = {}
    for l in open(log):
        m = re.match(r"SD (\S+) M(\S+) x (\S+) (\S+) cf (\S+) cp (\S+)", l)
        if m and m.group(1) == variant:
            rows.setdefault(float(m.group(2)), []).append((float(m.group(3)), float(m.group(4)), float(m.group(5))))
    for mach, r in rows.items():
        print("M", mach)
        for x0, x1, c in r:
            if x0 in (2.0, 5.0, 7.0, 10.0, 14.0, 18.0, 21.0):
                xm = 0.5 * (x0 + x1)
                re_x = est.RE_PER_M * xm * est.D
                f = est.cf_local(re_x, mach)
                print(f"  x/d {xm:.1f} Re_x {re_x:.2e} solver {c:.5f} flat plate {f:.5f} ratio {c / f:.3f}")


if __name__ == "__main__":
    if sys.argv[1] == "fox":
        fox()
    else:
        cf(sys.argv[2], sys.argv[3])
