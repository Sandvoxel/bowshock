#!/usr/bin/env python3
"""Hand estimates for the Arcas long model, fins off (DESIGN3D item 62): turbulent flat-plate skin friction (Eckert reference temperature on the White / Schoenherr
local law, virtual origin at the transition band) integrated over the wetted area of the body, and the empirical base drag (Niskanen / Fleeman, Hoerner).
Coefficients on the model cross-section pi/4 d^2 (the data's reference area).  usage: subdrag_estimate.py"""
import math

IN = 0.0254
D = 2.250 * IN
L_NOSE, L_TUBE, L_BT, D_BASE = 10.600 * IN, 41.090 * IN, 1.810 * IN, 1.470 * IN
RE_PER_M, T0 = 9.8e6, 322.0
GAMMA, PR_T, R_T = 1.4, 0.72, 0.89


def profile(n_nose=400, n_tube=200, n_bt=40):
    """(x, r) nodes along the body."""
    R = D / 2
    rho = (R * R + L_NOSE**2) / (2 * R)
    pts = []
    for i in range(n_nose + 1):
        x = L_NOSE * i / n_nose
        pts.append((x, math.sqrt(rho**2 - (L_NOSE - x) ** 2) + R - rho))
    for i in range(1, n_tube + 1):
        pts.append((L_NOSE + L_TUBE * i / n_tube, R))
    for i in range(1, n_bt + 1):
        f = i / n_bt
        pts.append((L_NOSE + L_TUBE + L_BT * f, R + f * (D_BASE / 2 - R)))
    return pts


def mu_suth(T):
    return 1.458e-6 * T**1.5 / (T + 110.4)


def cf_local(Rex, M, Tw_adiabatic=True):
    """Local turbulent Cf, adiabatic wall, Eckert reference temperature on White's incompressible law Cf = 0.455 / ln^2(0.06 Re_x)."""
    Te = T0 / (1 + 0.2 * M * M)
    Taw = Te * (1 + R_T * 0.2 * M * M)
    Ts = Te * (1 + 0.5 * (Taw / Te - 1) + 0.16 * R_T * 0.2 * M * M)
    rho_ratio = Te / Ts  # rho*/rho_e
    mu_ratio = mu_suth(Ts) / mu_suth(Te)
    Rs = Rex * rho_ratio / mu_ratio
    return rho_ratio * 0.455 / math.log(0.06 * Rs) ** 2


def cf_laminar(Rex, M):
    Te = T0 / (1 + 0.2 * M * M)
    Taw = Te * (1 + 0.85 * 0.2 * M * M)
    Ts = Te * (1 + 0.5 * (Taw / Te - 1) + 0.22 * 0.85 * 0.2 * M * M)
    rho_ratio = Te / Ts
    mu_ratio = mu_suth(Ts) / mu_suth(Te)
    return rho_ratio * 0.664 / math.sqrt(Rex * rho_ratio / mu_ratio)


def friction_ca(M, x_trip=None, segments=False):
    """C_A friction on pi/4 d^2: fully turbulent from the tip (`x_trip` None) or laminar to the arc length `x_trip` and turbulent behind it with the momentum-thickness virtual origin."""
    Te = T0 / (1 + 0.2 * M * M)
    ue = M * math.sqrt(GAMMA * 287.05 * Te)
    rho_e_over_mu = (RE_PER_M / ue)  # rho_e / mu_e = Re_per_m / u_e
    pts = profile()
    s, tot, parts = 0.0, 0.0, [0.0, 0.0, 0.0]
    Sref = math.pi * D * D / 4
    xv = None
    for (x0, r0), (x1, r1) in zip(pts[:-1], pts[1:]):
        ds = math.hypot(x1 - x0, r1 - r0)
        sm = s + 0.5 * ds
        if x_trip is None:
            cf = cf_local(RE_PER_M * sm, M)
        elif sm < x_trip:
            cf = cf_laminar(RE_PER_M * sm, M)
        else:
            if xv is None:
                # laminar theta at the trip (flat plate, incompressible scaling): Re_theta = 0.664 sqrt(Re_x); turbulent: Re_theta = 0.036 Re_x^0.8 -> equivalent length
                re_th = 0.664 * math.sqrt(RE_PER_M * x_trip)
                xv = x_trip - (re_th / 0.036) ** 1.25 / RE_PER_M
            cf = cf_local(RE_PER_M * (sm - xv), M)
        dA = 2 * math.pi * 0.5 * (r0 + r1) * ds
        tot += cf * dA / Sref
        parts[0 if x0 < L_NOSE else (1 if x0 < L_NOSE + L_TUBE - 1e-12 else 2)] += cf * dA / Sref
        s += ds
    return (tot, parts) if segments else tot


if __name__ == "__main__":
    print(f"L = {(L_NOSE + L_TUBE + L_BT):.4f} m, Re_L = {RE_PER_M * (L_NOSE + L_TUBE + L_BT):.3e}")
    for M in (0.6, 0.8):
        tt, pt = friction_ca(M, None, True)
        tr, pr = friction_ca(M, 1.25 * IN, True)
        print(f"M {M}: flat-plate turbulent CA_f nose/tube/boattail {pt[0]:.4f}/{pt[1]:.4f}/{pt[2]:.4f} = {tt:.4f}; tripped at 1.25 in {pr[0]:.4f}/{pr[1]:.4f}/{pr[2]:.4f} = {tr:.4f}")
        # empirical base drag on the model reference area: base area (1.47 in disc) over S
        ab = (D_BASE / D) ** 2
        cdb_free = 0.12 + 0.13 * M * M
        print(f"   base drag, free-flight blunt base (Niskanen/Fleeman 0.12+0.13M^2 on the base area {ab:.3f} S): {cdb_free * ab:.4f}")
        # Hoerner: C_Db = 0.029 / sqrt(C_Df) (d_b/d)^3 on S, C_Df the forebody drag (friction + pressure) on S
        for cdf in (tr, tr + 0.03):
            print(f"   Hoerner 0.029 (db/d)^3 / sqrt(CDf = {cdf:.3f}) = {0.029 * (D_BASE / D) ** 3 / math.sqrt(cdf):.4f}")


def trip_re_k(M, k=0.0049 * IN, x=1.25 * IN, ue_over_u=0.94):
    """Roughness Reynolds number of the carborundum band of D-4013 (No. 120 grit, nominal 0.0049 in, Braslow-Knox) on the laminar boundary layer 1.25 in aft of the nose tip: Blasius
    profile at y = k, u_k k / nu_k with nu_k the free-stream viscosity ratio at the wall temperature of the layer (adiabatic, ~ free-stream; error a few per cent). Effective trips need
    Re_k >~ 600 (Braslow, Knox, NACA TN 4363; the carborundum strips of the tunnel reports are sized to it)."""
    Te = T0 / (1 + 0.2 * M * M)
    u = M * math.sqrt(GAMMA * 287.05 * Te)
    nu = u / RE_PER_M
    ue = ue_over_u * u
    delta = 5 * x / math.sqrt(RE_PER_M * x * ue_over_u)
    eta = 5 * k / delta
    # Blasius f'(eta): fit of the table (f'(1)=0.330, 2: 0.630, 3: 0.846, 4: 0.956, 5: 0.992)
    tab = [(0, 0), (1, 0.3298), (2, 0.6298), (3, 0.8461), (4, 0.9555), (5, 0.9915)]
    fp = next(a[1] + (b[1] - a[1]) * (eta - a[0]) for a, b in zip(tab, tab[1:]) if eta <= b[0]) if eta < 5 else 1.0
    return fp * ue * k / nu, fp, delta, eta


if __name__ == "__main__":
    for M in (0.6, 0.8):
        rk, fp, delta, eta = trip_re_k(M)
        print(f"M {M}: trip band Re_k = {rk:.0f} (u_k/u_e {fp:.2f}, laminar delta {delta * 1e3:.2f} mm, k/delta {0.0049 * IN / delta:.2f})")
