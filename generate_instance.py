#!/usr/bin/env python3
"""
generate_instance.py

Generate random (or deterministic) problem instances for the helicopter delivery problem.

Output format (single instance):
Line 1: time_limit_minutes (float or int)
Line 2: DMax (float)
Line 3: w(d) v(d) w(p) v(p) w(o) v(o)   (six numbers: weights and values)
Line 4: C x1 y1 x2 y2 ... xC yC        (C followed by 2*C coords)
Line 5: V x1 y1 n1 x2 y2 n2 ... xV yV nV  (V followed by 3*V numbers)
Line 6: H home1 wcap1 dcap1 F1 alpha1 home2 wcap2 dcap2 F2 alpha2 ...

Example:
  python3 generate_instance.py --cities 2 --villages 2 --helicopters 2 --seed 42

You can also emit the exact sample instance by using --sample.
"""
import argparse
import random
import math
import sys

def format_float(x):
    # Keep concise but readable floats
    if abs(x - round(x)) < 1e-9:
        return str(int(round(x)))
    return f"{x:.6g}"

def generate_instance(
    time_limit_minutes=1,
    d_max=100.0,
    package_weights=(0.01, 0.1, 0.005),   # weights (in the same units your solver expects)
    package_values=(1.0, 2.0, 0.1),
    C=2,
    city_bounds=(0.0, 10.0),
    V=2,
    village_bounds=(0.0, 10.0),
    pop_min=50,
    pop_max=1000,
    H=2,
    heli_weight_cap=(50.0, 200.0),   # range for helicopter weight capacity
    heli_dist_cap=(20.0, 200.0),
    heli_F_range=(5.0, 50.0),
    heli_alpha_range=(0.1, 5.0),
    allow_same_home=True,
    seed=None,
    sample=False
):
    if seed is not None:
        random.seed(seed)

    if sample:
        # Emit the sample you provided in the prompt.
        # (Matches the sample exactly: unit choices are as in the prompt.)
        s = []
        s.append("1")
        s.append(format_float(d_max))
        s.append("0.01 1 0.1 2 0.005 0.1")
        s.append("2 0 0 10 10")
        s.append("2 0 5 1000 0 10 1000")
        # two helicopters: home city id, wcap, dcap, F, alpha
        s.append("2 1 100 25 10 1 2 100 50 10 1")
        return "\n".join(s) + "\n"

    # 1) time & Dmax
    lines = []
    lines.append(format_float(time_limit_minutes))
    lines.append(format_float(d_max))

    # 2) package weights/values
    w_d, w_p, w_o = package_weights
    v_d, v_p, v_o = package_values
    lines.append(" ".join(map(format_float, (w_d, v_d, w_p, v_p, w_o, v_o))))

    # 3) cities
    cities = []
    xmin, xmax = city_bounds
    ymin, ymax = city_bounds
    for i in range(C):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        cities.append((x, y))
    city_line = str(C) + " " + " ".join(f"{format_float(x)} {format_float(y)}" for (x, y) in cities)
    lines.append(city_line)

    # 4) villages (x, y, population)
    villages = []
    vxmin, vxmax = village_bounds
    vymin, vymax = village_bounds
    for i in range(V):
        x = random.uniform(vxmin, vxmax)
        y = random.uniform(vymin, vymax)
        # population typically integer
        n = random.randint(pop_min, pop_max)
        villages.append((x, y, n))
    village_line = str(V) + " " + " ".join(f"{format_float(x)} {format_float(y)} {n}" for (x, y, n) in villages)
    lines.append(village_line)

    # 5) helicopters: home_city_id, wcap, dcap, F, alpha
    helis = []
    for i in range(H):
        if allow_same_home:
            home = random.randint(1, C)
        else:
            # if H <= C try sample without replacement, otherwise wrap
            home = (i % C) + 1
        wcap = random.uniform(heli_weight_cap[0], heli_weight_cap[1])
        dcap = random.uniform(heli_dist_cap[0], heli_dist_cap[1])
        F = random.uniform(heli_F_range[0], heli_F_range[1])
        alpha = random.uniform(heli_alpha_range[0], heli_alpha_range[1])
        helis.append((home, wcap, dcap, F, alpha))
    heli_line = str(H) + " " + " ".join(" ".join(map(format_float, h)) for h in helis)
    lines.append(heli_line)

    return "\n".join(lines) + "\n"

def main():
    ap = argparse.ArgumentParser(description="Generate helicopter-delivery problem instances.")
    ap.add_argument("--cities", type=int, default=2, help="number of cities (C)")
    ap.add_argument("--villages", type=int, default=2, help="number of villages (V)")
    ap.add_argument("--helicopters", type=int, default=2, help="number of helicopters (H)")
    ap.add_argument("--time", type=float, default=1.0, help="total processing time available (minutes)")
    ap.add_argument("--dmax", type=float, default=100.0, help="DMax (max distance in km any helicopter can travel in total)")
    ap.add_argument("--seed", type=int, default=None, help="random seed (for reproducibility)")
    ap.add_argument("--sample", action="store_true", help="emit the exact sample input from the prompt")
    ap.add_argument("--out", default=None, help="output file (if not provided prints to stdout)")

    # optional ranges
    ap.add_argument("--city-range", type=float, nargs=2, metavar=('MIN','MAX'), default=(0.0, 100.0),
                    help="coordinate range for cities (min max)")
    ap.add_argument("--village-range", type=float, nargs=2, metavar=('MIN','MAX'), default=(0.0, 100.0),
                    help="coordinate range for villages (min max)")
    ap.add_argument("--pop-range", type=int, nargs=2, metavar=('MIN','MAX'), default=(50, 1000),
                    help="population range for villages")
    ap.add_argument("--package-weights", type=float, nargs=3, metavar=('w_d','w_p','w_o'), default=(0.01, 0.1, 0.005),
                    help="package weights for d, p, o (three numbers)")
    ap.add_argument("--package-values", type=float, nargs=3, metavar=('v_d','v_p','v_o'), default=(1.0, 2.0, 0.1),
                    help="package values for d, p, o (three numbers)")
    ap.add_argument("--heli-wcap", type=float, nargs=2, default=(50.0,200.0), help="heli weight capacity range")
    ap.add_argument("--heli-dcap", type=float, nargs=2, default=(20.0,200.0), help="heli per-trip distance capacity range")
    ap.add_argument("--heli-F", type=float, nargs=2, default=(5.0,50.0), help="heli fixed cost range")
    ap.add_argument("--heli-alpha", type=float, nargs=2, default=(0.1,5.0), help="heli alpha (per-km cost) range")
    args = ap.parse_args()

    txt = generate_instance(
        time_limit_minutes=args.time,
        d_max=args.dmax,
        package_weights=tuple(args.package_weights),
        package_values=tuple(args.package_values),
        C=args.cities,
        city_bounds=tuple(args.city_range),
        V=args.villages,
        village_bounds=tuple(args.village_range),
        pop_min=args.pop_range[0],
        pop_max=args.pop_range[1],
        H=args.helicopters,
        heli_weight_cap=tuple(args.heli_wcap),
        heli_dist_cap=tuple(args.heli_dcap),
        heli_F_range=tuple(args.heli_F),
        heli_alpha_range=tuple(args.heli_alpha),
        allow_same_home=True,
        seed=args.seed,
        sample=args.sample
    )

    if args.out:
        with open(args.out, "w") as f:
            f.write(txt)
        print(f"Wrote instance to {args.out}")
    else:
        sys.stdout.write(txt)

if __name__ == "__main__":
    main()
