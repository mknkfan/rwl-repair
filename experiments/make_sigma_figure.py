"""Where the sigma_min floor of 0.14 m/rad sits: how the "best" sigma_min is found.

    python -m experiments.make_sigma_figure
    python -m experiments.make_sigma_figure --out results/model/sigma_min_best

The procedure the figure walks through, for one machine height h:

1. put a top-down loading port at distance r from the robot's waist axis;
2. solve the closed-form IK for all four branches (shoulder left/right x
   elbow up/down) with :meth:`Puma560Arm.ik_vertical`;
3. drop branches that cannot reach the port or leave a joint's travel;
4. for each remaining branch, sigma_min = smallest singular value of the 3x3
   position Jacobian (:meth:`Puma560Arm.sigma_min`);
5. keep the largest over branches -> the envelope sigma*(r);
6. the "best" sigma_min is the peak of sigma*(r) over r.

sigma_min does not depend on the waist angle, so sweeping r along one ray
covers the whole floor; panel (d) checks that by computing it on a grid.

Panels:
(a) the arm at three ports at h = 0.25 m -- the best one and two near the
    singularities -- with the tool-velocity ellipse for unit joint speed;
(b) sigma_min against r for each branch at h = 0.25 m, and their envelope;
(c) the envelope at the lowest, middle and highest machine heights, with the
    0.14 floor and the best value of each;
(d) plan view of the 1.4 m cell: the envelope over the floor at h = 0.25 m.

Every number is the library's own output; nothing is redrawn by hand.
"""

from __future__ import annotations

import argparse
import os
import sys
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from robot import BRANCHES, BRANCH_NAMES, Puma560Arm  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Circle, Ellipse, Rectangle  # noqa: E402

warnings.filterwarnings("ignore", category=RuntimeWarning)

THRESHOLD = 0.14
HEIGHTS = (0.10, 0.25, 0.40)       # machine top heights: min, middle, max of the instances
FOCUS_H = 0.25
CELL_HALF = 0.70                   # 1.4 m square cell
KEEP_OUT = 0.18                    # robot column keep-out radius
ELLIPSE_SCALE = 0.35               # draw the ellipse for a joint-speed norm of 0.35 rad/s

# reference palette: categorical slots 1-4, ordinal blue ramp, chrome
BRANCH_COLOUR = {(1, 1): "#2a78d6", (1, -1): "#eb6834", (-1, 1): "#1baf7a", (-1, -1): "#eda100"}
BRANCH_LS = {(1, 1): "-", (1, -1): "--", (-1, 1): "-.", (-1, -1): ":"}
HEIGHT_COLOUR = {0.10: "#86b6ef", 0.25: "#2a78d6", 0.40: "#104281"}
INK, INK2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
REJECT = "#e34948"
SEQ = LinearSegmentedColormap.from_list(
    "blue", ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"])

plt.rcParams.update({
    "font.family": "sans-serif", "font.size": 8.5,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
    "axes.titlesize": 9.5, "axes.titleweight": "bold", "axes.titlecolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "savefig.dpi": 220, "savefig.bbox": "tight",
})


# -- the computation --------------------------------------------------------

def per_branch(arm: Puma560Arm, targets: np.ndarray):
    """sigma_min and q per branch; NaN where the branch is unusable."""
    out = {}
    for b in BRANCHES:
        q, ok = arm.ik_vertical(targets, *b)
        ok = ok & arm.within_limits(q)
        s = arm.sigma_min(q)
        s = np.where(ok, s, np.nan)
        out[b] = (s, q)
    return out


def envelope(branches):
    stack = np.stack([s for s, _ in branches.values()])
    best = np.nanmax(stack, axis=0)
    which = np.full(best.shape, -1)
    valid = ~np.all(np.isnan(stack), axis=0)
    which[valid] = np.nanargmax(stack[:, valid], axis=0)
    return best, which


def ray(arm, h, r):
    targets = np.stack([r, np.zeros_like(r), np.full_like(r, h)], axis=1)
    br = per_branch(arm, targets)
    env, which = envelope(br)
    return br, env, which


# -- drawing helpers --------------------------------------------------------

def label_line(ax, x, y, text, colour, dx=0.0, dy=0.0, ha="left", va="center", **kw):
    ax.annotate(text, (x, y), xytext=(x + dx, y + dy), color=colour, ha=ha, va=va,
                fontsize=8, fontweight="bold", **kw)


def draw_arm(ax, arm, q, r, colour, sigma, tag, label_xy, ha="center"):
    """Side view in the arm's vertical plane: radial distance vs height."""
    pts = arm.joint_positions(q)
    # The arm moves in a vertical plane turned by q1 and offset from the waist
    # axis by d3; project onto that plane's in-plane direction.
    direction = np.array([np.cos(q[0]), np.sin(q[0])])
    radial = pts[:, :2] @ direction
    z = pts[:, 2]
    ax.plot(radial[:2], z[:2], color=AXIS, lw=5, solid_capstyle="butt", zorder=2)   # column
    ax.plot(radial[1:], z[1:], color=colour, lw=2.4, zorder=3, solid_capstyle="round")
    ax.plot(radial[1:4], z[1:4], "o", ms=4.5, color="white", mec=colour, mew=1.5, zorder=4)

    # Tool-velocity ellipsoid {J qd : |qd| <= 1}, cut by the arm's vertical
    # plane (a true section, not a projection).  Its shortest 3-D axis is
    # sigma_min; report which way that axis points.
    jac = arm.position_jacobian(q[None, :])[0]
    across = np.array([-direction[1], direction[0], 0.0])
    plane = np.array([[direction[0], 0.0], [direction[1], 0.0], [0.0, 1.0]])   # columns: in-plane, up
    quad = plane.T @ np.linalg.inv(jac @ jac.T) @ plane
    w, v = np.linalg.eigh(quad)
    semi = 1.0 / np.sqrt(w)
    angle = np.degrees(np.arctan2(v[1, 0], v[0, 0]))
    tip = (radial[-1], z[-1])
    ax.add_patch(Ellipse(tip, 2 * ELLIPSE_SCALE * semi[0], 2 * ELLIPSE_SCALE * semi[1], angle=angle,
                         fc=colour, ec=colour, alpha=0.18, lw=1.2, zorder=1))
    thin = np.linalg.svd(jac)[0][:, -1]
    parts = {"sideways": abs(thin @ across), "along arm": abs(thin @ np.r_[direction, 0.0]),
             "vertical": abs(thin[2])}
    thin_dir = max(parts, key=parts.get)
    ax.annotate(f"{tag} (r = {r:.2f} m)\nσ_min {sigma:.3f}, thin {thin_dir}", tip, xytext=label_xy,
                ha=ha, va="center", fontsize=7.5, color=colour, fontweight="bold",
                arrowprops=dict(arrowstyle="-", color=colour, lw=0.7, shrinkB=4))


# -- the figure -------------------------------------------------------------

def make(out_base: str) -> None:
    arm = Puma560Arm()
    r = np.linspace(0.0, 1.0, 2001)

    fig = plt.figure(figsize=(11.5, 9.6))
    gs = fig.add_gridspec(2, 2, width_ratios=(1.1, 1.0), hspace=0.30, wspace=0.22)
    ax_a = fig.add_subplot(gs[0, 0])
    gs_b = gs[0, 1].subgridspec(2, 1, height_ratios=(3.2, 1.0), hspace=0.08)
    ax_b, ax_s = fig.add_subplot(gs_b[0]), fig.add_subplot(gs_b[1])
    ax_c, ax_d = fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])

    # (b) sigma_min against r, and where each IK branch is usable
    br, env, which = ray(arm, FOCUS_H, r)
    spread = np.nanmax(np.stack([s for s, _ in br.values()]), axis=0) - \
        np.nanmin(np.stack([s for s, _ in br.values()]), axis=0)
    i_best = int(np.nanargmax(env))
    ax_b.axhspan(0, THRESHOLD, color=REJECT, alpha=0.06, lw=0)
    ax_b.plot(r, env, color=INK, lw=2.2)
    ax_b.axhline(THRESHOLD, color=REJECT, lw=1.2, ls="--")
    ax_b.text(0.5, THRESHOLD - 0.008, "floor 0.14: ports below are rejected", color=REJECT,
              ha="center", va="top", fontsize=7.5, transform=ax_b.get_yaxis_transform())
    ax_b.plot(r[i_best], env[i_best], "o", ms=8, color=INK, mec="white", mew=1.5, zorder=5)
    ax_b.annotate(f"best: {env[i_best]:.3f} m/rad at r = {r[i_best]:.2f} m",
                  (r[i_best], env[i_best]), xytext=(r[i_best] + 0.06, 0.37),
                  fontsize=8, color=INK, fontweight="bold",
                  arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    ok_r = r[env >= THRESHOLD]
    for edge in (ok_r.min(), ok_r.max()):
        ax_b.plot([edge, edge], [0, THRESHOLD], color=REJECT, lw=0.8, ls=":")
    ax_b.text(ok_r.min() - 0.01, 0.02, "tool near\nwaist axis", ha="right", fontsize=7.5, color=REJECT)
    ax_b.text(ok_r.max() + 0.01, 0.02, "elbow\nlock-out", ha="left", fontsize=7.5, color=REJECT)
    max_spread = float(np.nanmax(spread))
    ax_b.set_xlim(0, 1.0)
    ax_b.set_ylim(0, 0.40)
    ax_b.tick_params(labelbottom=False)
    ax_b.set_ylabel("σ_min [m/rad]")
    ax_b.set_title(f"(b) Steps 2–5 along the floor (h = {FOCUS_H} m)", loc="left", x=0.0)

    # usable range of each branch: reachable and inside joint travel
    for k, b in enumerate(BRANCHES):
        valid = ~np.isnan(br[b][0])
        edges = np.flatnonzero(np.diff(np.r_[0, valid.astype(int), 0]))
        for lo, hi in zip(edges[::2], edges[1::2]):
            ax_s.plot([r[lo], r[hi - 1]], [k, k], color=BRANCH_COLOUR[b], lw=5,
                      solid_capstyle="butt")
    ax_s.set_yticks(range(len(BRANCHES)))
    ax_s.set_yticklabels([BRANCH_NAMES[b] for b in BRANCHES], fontsize=7.5)
    ax_s.set_ylim(len(BRANCHES) - 0.5, -0.5)
    ax_s.set_xlim(0, 1.0)
    ax_s.grid(axis="y", visible=False)
    ax_s.set_xlabel("Port distance from waist axis r [m]")

    # (a) the arm at three ports, chosen off the curve
    reach = ~np.isnan(env)
    inner = np.flatnonzero(reach & (r < r[i_best]))
    outer = np.flatnonzero(reach & (r > r[i_best]))
    i_in = inner[np.nanargmin(np.abs(env[inner] - 0.06))]
    i_out = outer[np.nanargmin(np.abs(env[outer] - 0.06))]
    poses = [(i_in, "#1baf7a", "near waist axis", (-0.19, 1.36), "left"),
             (i_best, INK, "best", (0.42, 0.98), "center"),
             (i_out, REJECT, "near lock-out", (1.04, 1.17), "right")]
    for i, colour, tag, xy, ha in poses:
        b = BRANCHES[which[i]]
        draw_arm(ax_a, arm, br[b][1][i], r[i], colour, env[i], tag, xy, ha)
    ax_a.add_patch(Rectangle((-0.2, -0.4), 1.4, 0.4 + FOCUS_H, fc=GRID, ec="none", alpha=0.45, zorder=0))
    ax_a.axhline(FOCUS_H, color=MUTED, lw=0.8, ls=":")
    ax_a.text(0.99, FOCUS_H - 0.015, f"machine tops, h = {FOCUS_H} m", ha="right", va="top",
              fontsize=7.5, color=INK2, transform=ax_a.get_yaxis_transform())
    ax_a.set_aspect("equal")
    ax_a.set_xlim(-0.2, 1.05)
    ax_a.set_ylim(-0.45, 1.42)
    ax_a.set_xlabel("In-plane distance √(r² − d₃²) [m], d₃ = 0.15 m")
    ax_a.set_ylabel("Height [m]")
    ax_a.set_title("(a) Steps 1, 4: the arm at three ports", loc="left")
    ax_a.text(0.01, 0.015,
              f"Ellipse: tool velocities at joint speed {ELLIPSE_SCALE} rad/s,\n"
              "cut through this view. σ_min is the shortest\n"
              "3-D axis per rad/s; 'thin' is where it points.\n"
              "Near the waist axis: sideways, out of view.",
              transform=ax_a.transAxes, fontsize=7.5, color=INK2, va="bottom")

    # (c) envelope at three heights
    summary = []
    for h in HEIGHTS:
        _, env_h, _ = ray(arm, h, r)
        c = HEIGHT_COLOUR[h]
        ax_c.plot(r, env_h, color=c, lw=2.0, label=f"h = {h:.2f} m")
        j = int(np.nanargmax(env_h))
        ax_c.plot(r[j], env_h[j], "o", ms=6, color=c, mec="white", mew=1.2, zorder=5)
        ok = env_h >= THRESHOLD
        reach_h = ~np.isnan(env_h)
        summary.append((h, env_h[j], r[j], r[reach_h].min(), r[reach_h].max(), r[ok].min(), r[ok].max()))
    ax_c.axhline(THRESHOLD, color=REJECT, lw=1.2, ls="--")
    best_lo = min(s[1] for s in summary)
    best_hi = max(s[1] for s in summary)
    ax_c.axhspan(0, THRESHOLD, color=REJECT, alpha=0.06, lw=0)
    ax_c.text(0.01, THRESHOLD - 0.008, "rejected: σ_min < 0.14", color=REJECT, fontsize=8,
              va="top", transform=ax_c.get_yaxis_transform())
    ax_c.axhspan(best_lo / 2, best_hi / 2, color=MUTED, alpha=0.15, lw=0)
    ax_c.text(0.99, best_hi / 2 + 0.004,
              f"half the best: {best_lo / 2:.3f}–{best_hi / 2:.3f}", color=INK2, fontsize=7.5,
              ha="right", va="bottom", transform=ax_c.get_yaxis_transform())
    for h, best, rb, *_ in summary:
        offset = {0.10: (8, -12), 0.25: (-16, 8), 0.40: (16, 6)}[h]
        ax_c.annotate(f"{best:.3f}", (rb, best), xytext=offset, textcoords="offset points",
                      ha="center", va="center", fontsize=7.5, color=HEIGHT_COLOUR[h], fontweight="bold")
    ax_c.set_xlim(0, 1.0)
    ax_c.set_ylim(0, 0.36)
    ax_c.set_xlabel("Port distance from waist axis r [m]")
    ax_c.set_ylabel("Best-branch σ_min [m/rad]")
    ax_c.set_title("(c) Step 6: the best σ_min at three machine heights", loc="left")
    ax_c.legend(loc="upper right", fontsize=7.5, title="machine top height", title_fontsize=7.5,
                alignment="left")

    # (d) plan view of the cell at the focus height
    n = 281
    xs = np.linspace(-CELL_HALF, CELL_HALF, n)
    gx, gy = np.meshgrid(xs, xs)
    targets = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, FOCUS_H)], axis=1)
    env_grid, _ = envelope(per_branch(arm, targets))
    env_grid = env_grid.reshape(gx.shape)
    floor_r = np.hypot(gx, gy)
    env_grid[floor_r < KEEP_OUT] = np.nan
    im = ax_d.pcolormesh(gx, gy, env_grid, cmap=SEQ, vmin=0, vmax=0.3, shading="auto")
    ax_d.contour(gx, gy, np.nan_to_num(env_grid, nan=-1), levels=[THRESHOLD], colors=REJECT,
                 linewidths=1.4, linestyles="--")
    ax_d.add_patch(Circle((0, 0), KEEP_OUT, fc="white", ec=AXIS, lw=1, zorder=3))
    ax_d.text(0, 0, "robot\ncolumn", ha="center", va="center", fontsize=7, color=INK2, zorder=4)
    ax_d.add_patch(Rectangle((-CELL_HALF, -CELL_HALF), 2 * CELL_HALF, 2 * CELL_HALF,
                             fc="none", ec=INK2, lw=1.2, zorder=3))
    usable = (floor_r >= KEEP_OUT) & ~np.isnan(env_grid)
    kept = usable & (env_grid >= THRESHOLD)
    share = kept.sum() / usable.sum()
    ax_d.set_aspect("equal")
    ax_d.grid(False)
    ax_d.set_xlim(-CELL_HALF - 0.03, CELL_HALF + 0.03)
    ax_d.set_ylim(-CELL_HALF - 0.03, CELL_HALF + 0.03)
    ax_d.set_xlabel("x [m]")
    ax_d.set_ylabel("y [m]")
    ax_d.set_title(f"(d) The cell floor at h = {FOCUS_H} m", loc="left")
    cb = fig.colorbar(im, ax=ax_d, fraction=0.046, pad=0.03)
    cb.set_label("Best-branch σ_min [m/rad]", color=INK2)
    cb.outline.set_edgecolor(AXIS)
    ax_d.text(0.02, 0.02,
              f"Dashed: σ_min = 0.14. White: unreachable.\n"
              f"{share:.0%} of the reachable floor clears the floor.",
              transform=ax_d.transAxes, fontsize=7.5, color=INK, va="bottom",
              bbox=dict(fc="white", ec="none", alpha=0.85, pad=2))

    fig.suptitle("How the best σ_min is found, and where the 0.14 m/rad floor sits",
                 x=0.06, ha="left", y=0.995, fontsize=12, fontweight="bold", color=INK)
    fig.text(0.06, 0.968,
             "1. Place a top-down port at distance r and height h.   2. Solve the IK for all four branches.   "
             "3. Drop branches out of reach or joint travel.\n"
             "4. σ_min = smallest singular value of the 3×3 position Jacobian.   "
             "5. Keep the largest over branches.   6. The best σ_min is the peak over r.\n"
             f"All four branches give the same σ_min at a port (at most {max_spread:.3f} m/rad apart); "
             "they differ only in where they can reach inside joint travel (bars under b).",
             ha="left", va="top", fontsize=8.5, color=INK2)

    os.makedirs(os.path.dirname(out_base), exist_ok=True)
    for ext in ("png",):
        fig.savefig(f"{out_base}.{ext}")
    plt.close(fig)
    print("wrote", out_base + ".png")
    print("h [m]  best  at r   reach r       sigma>=0.14 r   floor share kept (h=%.2f): %.1f%%"
          % (FOCUS_H, 100 * share))
    for h, best, rb, r0, r1, k0, k1 in summary:
        print(f"{h:.2f}  {best:.3f} {rb:.2f}  {r0:.2f}-{r1:.2f}   {k0:.2f}-{k1:.2f}   0.14 = {THRESHOLD / best:.0%} of best")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--out", default=os.path.join(results_dir("model"), "sigma_min_best"))
    args = p.parse_args()
    make(args.out)


if __name__ == "__main__":
    main()
