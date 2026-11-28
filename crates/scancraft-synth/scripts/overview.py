"""One-page overview of the synthetic scene evaluation (synth_eval): one row per scene with the
true shape, Reconstruct and Make Solid in grazing light, and the key numbers.

    python overview.py <data dir> <eval dir> [out.png]

Writes <eval dir>/overview.png by default. The view is tilted (from the front right, above), the
light comes from 12 degrees above the part's top, so steps, rings, waviness and filled holes
show. Only the part's box (plus 3 mm) is drawn: what is left of a table or clutter inside it
shows, the rest is cut away. Needs numpy, matplotlib and Pillow.
"""

import json
import os
import sys
import tempfile

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_features import MeshData  # noqa: E402

VIEW = np.array([0.35, -0.55, 0.76])
ROW_H = 2.1
DPI = 72


def shade_panel(ax, m, lo, hi, e1, e2, n, light):
    ax.set_facecolor((0.02, 0.02, 0.03))
    ax.set_xticks([])
    ax.set_yticks([])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    c = 0.5 * (lo + hi)
    u, v = (corners - c) @ e1, (corners - c) @ e2
    span = max(u.max() - u.min(), (v.max() - v.min()) * 1.25) * 0.52
    ax.set_xlim(-span, span)
    ax.set_ylim(-span / 1.25, span / 1.25)
    if m is None:
        ax.text(0.5, 0.5, "no mesh", color="w", ha="center", va="center", transform=ax.transAxes)
        return
    inside = np.all((m.centroid >= lo - 3.0) & (m.centroid <= hi + 3.0), axis=1)
    idx = np.nonzero(inside)[0]
    if len(idx) == 0:
        ax.text(0.5, 0.5, "nothing here", color="w", ha="center", va="center", transform=ax.transAxes)
        return
    tri = m.p[m.f[idx]] - c
    order = np.argsort(tri.mean(axis=1) @ n)
    tri = tri[order]
    uv = np.stack([tri @ e1, tri @ e2], axis=2)
    nf = m.normal[idx][order]
    nf = np.where((nf @ n)[:, None] < 0, -nf, nf)
    # A face square to "up" is mid grey; tilts towards / away from the light brighten / darken it.
    up = np.array([0.0, 0.0, 1.0])
    s = np.clip(0.5 + 1.1 * (nf @ light - light @ up), 0.0, 1.0)
    colours = np.stack([s * 0.93, s * 0.92, s * 0.88, np.ones_like(s)], axis=1)
    ax.add_collection(PolyCollection(uv, facecolors=colours, edgecolors="none", antialiased=False))


def fmt_eval(ev, solid, truth_genus):
    if ev is None:
        return "-"
    g, t = ev["global"], ev["topology"]
    ok = sum(1 for f in ev["features"] if not f["fails"])
    s = (f"rms {g['out_to_truth']['rms_mm']:.3f} p99 {g['out_to_truth']['p99_mm']:.2f} | t->o p99 {g['truth_to_out']['p99_mm']:.2f} "
         f"miss {100 * g['missing_share']:.1f}% | probes {ev['probes_passed']}/{ev['probes']} | feat {ok}/{len(ev['features'])}")
    if solid:
        s += f" | {'W' if t['watertight'] else 'open'}{' c' + str(t['components']) if t['components'] > 1 else ''} g{t['genus']} (true {truth_genus})"
    return s


def failing(ev, limit=7):
    if ev is None:
        return ""
    ids = [f["id"] for f in ev["features"] if f["fails"]]
    more = f" +{len(ids) - limit}" if len(ids) > limit else ""
    return ", ".join(ids[:limit]) + more


def render_row(task):
    data, evals, scene, out = task
    res = json.load(open(os.path.join(evals, scene, "eval.json"), encoding="utf-8"))
    truth = json.load(open(os.path.join(data, scene, "truth.json"), encoding="utf-8"))
    lo, hi = np.array(truth["bounds"][0], float), np.array(truth["bounds"][1], float)
    # Looking along -VIEW, z up in the image.
    n = VIEW / np.linalg.norm(VIEW)
    up = np.array([0.0, 0.0, 1.0])
    e2 = up - (up @ n) * n
    e2 /= np.linalg.norm(e2)
    e1 = np.cross(e2, n)
    # Light 12 degrees above the part's top, from the left of the view.
    horiz = -e1 - (-e1 @ up) * up
    horiz /= np.linalg.norm(horiz)
    light = np.cos(np.radians(12)) * horiz + np.sin(np.radians(12)) * up
    paths = {"truth": os.path.join(data, scene, "reference_mesh.ply"), "reconstruct": os.path.join(evals, scene, "reconstruct.ply"), "solid": os.path.join(evals, scene, "solid.ply")}
    fig = plt.figure(figsize=(16.5, ROW_H), dpi=DPI)
    gs = fig.add_gridspec(1, 4, width_ratios=[2.6, 2.6, 2.6, 8.7], wspace=0.03, left=0.003, right=0.997, top=0.98, bottom=0.02)
    for k, key in enumerate(("truth", "reconstruct", "solid")):
        ax = fig.add_subplot(gs[0, k])
        m = MeshData(paths[key]) if os.path.isfile(paths[key]) else None
        shade_panel(ax, m, lo, hi, e1, e2, n, light)
    ax = fig.add_subplot(gs[0, 3])
    ax.axis("off")
    evs = {e["mesh"]: e for e in res.get("evals", [])}
    tg = (res.get("truth_topology") or {}).get("genus", "?")
    iso = [p for p in res.get("passes", []) if p.get("dropped_by_label", [0])[0] > 1000]
    lines = [
        (scene, 10.5, "bold", "black"),
        (truth["description"][:150] + ("..." if len(truth["description"]) > 150 else ""), 7.2, "normal", "dimgray"),
        ("Reconstruct: " + fmt_eval(evs.get("reconstruct"), False, tg), 8.2, "normal", "black"),
        ("   fails: " + (failing(evs.get("reconstruct")) or "none"), 7.6, "normal", "darkred" if failing(evs.get("reconstruct")) else "darkgreen"),
        ("Make Solid: " + fmt_eval(evs.get("solid"), True, tg), 8.2, "normal", "black"),
        ("   fails: " + (failing(evs.get("solid")) or "none"), 7.6, "normal", "darkred" if failing(evs.get("solid")) else "darkgreen"),
    ]
    if iso:
        lines.append(("   isolation dropped part points: " + ", ".join(f"pass {p['id']} {p['dropped_by_label'][0]}" for p in iso), 7.6, "normal", "darkorange"))
    if res.get("error"):
        lines.append(("ERROR " + res["error"][:120], 8, "bold", "red"))
    y = 0.97
    for text, size, weight, colour in lines:
        ax.text(0.005, y, text, fontsize=size, weight=weight, color=colour, va="top", ha="left", transform=ax.transAxes)
        y -= 0.128 if size > 8 else 0.12
    fig.savefig(out)
    plt.close(fig)
    return out


def header(out, count):
    fig = plt.figure(figsize=(16.5, 0.55), dpi=DPI)
    for x, t in ((0.08, "True shape (reference mesh)"), (0.24, "Reconstruct"), (0.395, "Make Solid")):
        fig.text(x, 0.55, t, fontsize=10, weight="bold", ha="center", va="center")
    fig.text(0.47, 0.55, f"{count} synthetic scenes through the app's pipeline (defaults). rms / p99: output -> truth (mm); t->o: truth -> output p99 over the seen surface; "
             "miss: seen surface with no mesh within 1 mm; feat: features without a failure; W = watertight, g = genus.", fontsize=7.5, ha="left", va="center", wrap=True)
    fig.savefig(out)
    plt.close(fig)


def main():
    import multiprocessing
    from PIL import Image
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    data, evals = args[0], args[1]
    out = args[2] if len(args) > 2 else os.path.join(evals, "overview.png")
    order = []
    idx = os.path.join(data, "index.json")
    if os.path.isfile(idx):
        order = [s.get("scene", s.get("name")) for s in json.load(open(idx, encoding="utf-8")).get("scenes", [])]
    scenes = [s for s in os.listdir(evals) if os.path.isfile(os.path.join(evals, s, "eval.json"))]
    scenes.sort(key=lambda s: (order.index(s) if s in order else 999, s))
    tmp = tempfile.mkdtemp(prefix="overview-", dir=os.environ.get("TMP"))
    tasks = [(data, evals, s, os.path.join(tmp, f"{k:03d}.png")) for k, s in enumerate(scenes)]
    with multiprocessing.Pool(max(1, min(len(tasks), (os.cpu_count() or 2) // 2))) as pool:
        rows = pool.map(render_row, tasks)
    head = os.path.join(tmp, "header.png")
    header(head, len(scenes))
    images = [Image.open(p).convert("RGB") for p in [head] + rows]
    w = max(i.width for i in images)
    sheet = Image.new("RGB", (w, sum(i.height for i in images)), "white")
    y = 0
    for i in images:
        sheet.paste(i, (0, y))
        y += i.height
    sheet.save(out, optimize=True)
    for p in [head] + rows:
        os.remove(p)
    os.rmdir(tmp)
    print(f"wrote {out} ({sheet.width} x {sheet.height})")


if __name__ == "__main__":
    main()
