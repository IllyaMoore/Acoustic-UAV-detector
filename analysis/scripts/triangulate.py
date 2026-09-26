"""Multi-node demo: three nodes, one moving drone, bearing-only fusion.

    uv run python scripts/triangulate.py --out ../docs/img/triangulation.png

Each node only knows a bearing (with realistic noise from error_budget.py).
Crossing the bearings gives a position; the error ellipse grows where the
nodes see the target from similar angles - the geometry lesson every
networked-sensor deployment has to learn.
"""

import argparse

import numpy as np

from uavdoa.fusion import triangulate


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="triangulation.png")
    ap.add_argument("--sigma", type=float, default=2.0, help="bearing noise per node, deg")
    a = ap.parse_args()
    rng = np.random.default_rng(3)

    nodes = np.array([[0.0, 0.0], [800.0, 100.0], [350.0, -450.0]])
    t = np.linspace(0, 1, 40)
    path = np.stack([-600 + 1900 * t, 900 - 300 * t + 150 * np.sin(4 * t)], axis=1)

    fixes, errs = [], []
    for p in path:
        b = np.degrees(np.arctan2(p[0] - nodes[:, 0], p[1] - nodes[:, 1])) % 360
        f = triangulate(nodes, b + rng.normal(0, a.sigma, 3), sigma_deg=a.sigma)
        if f is not None:
            fixes.append(f)
            errs.append(np.linalg.norm(f.xy - p))
    print(f"{len(fixes)}/{len(path)} fixes, median miss {np.median(errs):.0f} m, "
          f"90th pct {np.percentile(errs, 90):.0f} m at sigma {a.sigma} deg")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    fig, ax = plt.subplots(figsize=(7.5, 6))
    ax.plot(*path.T, "k-", lw=1, label="true path")
    xy = np.array([f.xy for f in fixes])
    ax.plot(*xy.T, "o", ms=3, c="C0", label="fused fix")
    for f in fixes[::4]:
        w, v = np.linalg.eigh(f.cov)
        ang = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
        ax.add_patch(Ellipse(f.xy, 4 * np.sqrt(w[1]), 4 * np.sqrt(w[0]), angle=ang,
                             fill=False, ec="C0", alpha=0.5))
    ax.plot(*nodes.T, "^", ms=10, c="C3", label="acoustic node")
    p = path[len(path) // 2]
    for n in nodes:
        ax.plot([n[0], p[0]], [n[1], p[1]], c="C3", lw=0.6, alpha=0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("east, m")
    ax.set_ylabel("north, m")
    ax.set_title(f"Bearing-only fusion, 3 nodes, {a.sigma} deg bearing noise (2-sigma ellipses)")
    ax.legend(loc="lower right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f"plot -> {a.out}")


if __name__ == "__main__":
    main()
