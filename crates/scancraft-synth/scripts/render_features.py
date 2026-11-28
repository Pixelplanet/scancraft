"""Grazing-light renders of every feature of the synthetic scene evaluation (synth_eval).

    python render_features.py <data dir> <eval dir> [filter] [--only-failed] [--skip-existing]

Scenes are rendered in parallel (half the CPU threads).

For each scene in <eval dir> with an eval.json, and each feature of its truth.json (plus an
overview), writes <eval dir>/img/<scene>__<feature>.png with six panels:

    truth (reference mesh)   | Reconstruct            | Make Solid          (grazing light)
    merged points (by dev)   | Reconstruct deviation  | Make Solid deviation (signed distance
                                                                              to the true part)

The view looks straight down the feature's normal; the light comes from 12 degrees above the
surface so that steps, rings, waviness and filled holes show. Needs numpy and matplotlib.
"""

import json
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "uchar": "u1", "uint8": "u1", "char": "i1",
             "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4", "short": "<i2", "ushort": "<u2"}


def read_ply(path):
    """Binary little-endian PLY: (vertex record array, faces (n, 3) int array or None)."""
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            header.append(line)
            if line == "end_header":
                break
        elements = []
        for line in header:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "element":
                elements.append([parts[1], int(parts[2]), []])
            elif parts[0] == "property":
                elements[-1][2].append(parts[1:])
        verts, faces = None, None
        for name, count, props in elements:
            if name == "vertex":
                dt = np.dtype([(p[-1], PLY_TYPES[p[0]]) for p in props])
                verts = np.frombuffer(f.read(dt.itemsize * count), dtype=dt, count=count)
            elif name == "face":
                p = props[0]
                assert p[0] == "list"
                cdt, idt = PLY_TYPES[p[1]], PLY_TYPES[p[2]]
                dt = np.dtype([("n", cdt), ("i", idt, (3,))])
                raw = np.frombuffer(f.read(dt.itemsize * count), dtype=dt, count=count)
                faces = raw["i"].astype(np.int64)
            else:
                dt = np.dtype([(p[-1], PLY_TYPES[p[0]]) for p in props])
                f.read(dt.itemsize * count)
    return verts, faces


def xyz(v):
    return np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64)


def frame(n):
    n = np.asarray(n, float)
    n = n / np.linalg.norm(n)
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.8 else np.array([0, 1.0, 0])
    e1 = np.cross(n, a)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(n, e1), n


class MeshData:
    def __init__(self, path):
        v, f = read_ply(path)
        self.p = xyz(v)
        self.dev = np.asarray(v["dev"], float) if "dev" in v.dtype.names else None
        self.f = f
        tri = self.p[f]
        self.centroid = tri.mean(axis=1)
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(nrm, axis=1)
        ln[ln == 0] = 1
        self.normal = nrm / ln[:, None]


def crop_faces(m, c, radius):
    d = np.linalg.norm(m.centroid - c, axis=1)
    return np.nonzero(d < radius * 1.2)[0]


def draw_mesh(ax, m, c, e1, e2, n, radius, mode, light, gain):
    if m is None:
        ax.text(0.5, 0.5, "no mesh", ha="center", va="center", transform=ax.transAxes)
        return
    idx = crop_faces(m, c, radius)
    if len(idx) == 0:
        ax.text(0.5, 0.5, "nothing here", ha="center", va="center", transform=ax.transAxes)
        return
    tri = m.p[m.f[idx]] - c
    depth = tri.mean(axis=1) @ n
    order = np.argsort(depth)
    tri = tri[order]
    uv = np.stack([tri @ e1, tri @ e2], axis=2)
    if mode == "shade":
        nf = m.normal[idx][order]
        nf = np.where((nf @ n)[:, None] < 0, -nf, nf)
        # A face square to the view is mid grey; tilts towards / away from the light (12 degrees
        # above the surface) brighten / darken it (about 3 % per degree at gain 2).
        s = np.clip(0.5 + gain * (nf @ light - light @ n), 0.0, 1.0)
        colours = np.stack([s * 0.93, s * 0.92, s * 0.88, np.ones_like(s)], axis=1)
    else:
        dv = m.dev[m.f[idx]].mean(axis=1)[order] if m.dev is not None else np.zeros(len(idx))
        colours = plt.get_cmap("RdBu_r")(np.clip(dv / 0.2 + 0.5, 0, 1))
    pc = PolyCollection(uv, facecolors=colours, edgecolors="none", antialiased=False)
    ax.add_collection(pc)


def draw_points(ax, pts, dev, c, e1, e2, n, radius):
    if pts is None:
        return
    rel = pts - c
    keep = np.linalg.norm(rel, axis=1) < radius * 1.2
    rel, dv = rel[keep], dev[keep]
    order = np.argsort(rel @ n)
    rel, dv = rel[order], dv[order]
    size = max(0.6, min(6.0, 2400.0 / max(1.0, radius) ** 2))
    ax.scatter(rel @ e1, rel @ e2, c=np.clip(dv / 0.2 + 0.5, 0, 1), cmap="RdBu_r", vmin=0, vmax=1, s=size, linewidths=0)


def short(metrics, limit=9):
    out = []
    for k, v in metrics.items():
        if isinstance(v, list) or v is None:
            continue
        if isinstance(v, float):
            v = f"{v:.3f}".rstrip("0").rstrip(".") if abs(v) < 1000 else f"{v:.0f}"
        out.append(f"{k}={v}")
        if len(out) >= limit:
            break
    return ", ".join(out)


def render(job, meshes, points, out_path):
    c = np.asarray(job["center"], float)
    e1, e2, n = frame(job["normal"])
    radius = job["radius"]
    el, az = np.radians(12.0), np.radians(35.0)
    light = np.cos(el) * (np.cos(az) * e1 + np.sin(az) * e2) + np.sin(el) * n
    fig, axes = plt.subplots(2, 3, figsize=(15, 10.2), dpi=80)
    panels = [("truth", "shade"), ("reconstruct", "shade"), ("solid", "shade"), (None, "points"), ("reconstruct", "dev"), ("solid", "dev")]
    names = {"truth": "Truth (reference mesh)", "reconstruct": "Reconstruct", "solid": "Make Solid"}
    for ax, (key, mode) in zip(axes.flat, panels):
        ax.set_aspect("equal")
        ax.set_xlim(-radius, radius)
        ax.set_ylim(-radius, radius)
        ax.set_facecolor({"shade": (0.02, 0.02, 0.03), "points": (0.3, 0.3, 0.3)}.get(mode, (0.85, 0.85, 0.85)))
        ax.tick_params(labelsize=7)
        if mode == "points":
            draw_points(ax, points[0], points[1], c, e1, e2, n, radius)
            ax.set_title("Merged points, signed distance (red out, blue in, ±0.1 mm)", fontsize=9)
            continue
        draw_mesh(ax, meshes.get(key), c, e1, e2, n, radius, mode, light, 0.8 if job["kind"] == "overview" else 2.0)
        title = names[key] + (" deviation (±0.1 mm)" if mode == "dev" else "")
        status = job["status"].get(key)
        if status is not None and mode == "shade":
            colour = "darkgreen" if not status["fails"] else "darkred"
            title += ": " + ("ok" if not status["fails"] else "; ".join(status["fails"]))[:95]
            ax.set_title(title, fontsize=9, color=colour)
        else:
            ax.set_title(title, fontsize=9)
    lines = [f"{job['scene']} / {job['id']} ({job['kind']})", job.get("expect", "")[:180]]
    for key in ("reconstruct", "solid"):
        st = job["status"].get(key)
        if st:
            lines.append(f"{names[key]}: probes {st['probes_passed']}/{st['probes']}; {short(st['metrics'])}"[:230])
    fig.suptitle("\n".join(lines), fontsize=9.5, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out_path)
    plt.close(fig)


def render_scene(task):
    data, evals, scene, only_failed, skip_existing = task
    img = os.path.join(evals, "img")
    sdir = os.path.join(evals, scene)
    res = json.load(open(os.path.join(sdir, "eval.json"), encoding="utf-8"))
    if not res.get("evals"):
        return f"{scene}: no results"
    truth = json.load(open(os.path.join(data, scene, "truth.json"), encoding="utf-8"))
    meshes = {}
    for key, path in (("truth", os.path.join(data, scene, "reference_mesh.ply")), ("reconstruct", os.path.join(sdir, "reconstruct.ply")), ("solid", os.path.join(sdir, "solid.ply"))):
        if os.path.isfile(path):
            meshes[key] = MeshData(path)
    points = (None, None)
    mp = os.path.join(sdir, "merged.ply")
    if os.path.isfile(mp):
        v, _ = read_ply(mp)
        points = (xyz(v), np.asarray(v["dev"], float))
    by_mesh = {e["mesh"]: {f["id"]: f for f in e["features"]} for e in res["evals"]}
    lo, hi = np.array(truth["bounds"][0]), np.array(truth["bounds"][1])
    centre = 0.5 * (lo + hi)
    half = 0.5 * np.linalg.norm((hi - lo)[:2])
    top = {"id": "overview", "kind": "overview", "scene": scene, "center": list(centre), "normal": [0.0, 0.0, 1.0], "radius": half * 0.9, "expect": truth["description"], "status": {}}
    jobs = [top, dict(top, id="overview_tilted", normal=[0.35, -0.55, 0.76])]
    for f in truth["features"]:
        st = {k: v[f["id"]] for k, v in by_mesh.items() if f["id"] in v}
        if not st or (only_failed and not any(s["fails"] for s in st.values())):
            continue
        view = next(iter(st.values()))["view"]
        jobs.append({"id": f["id"], "kind": f["kind"], "scene": scene, "center": view["center"], "normal": view["normal"], "radius": view["radius"], "expect": f["expect"], "status": st})
    done = 0
    for job in jobs:
        out = os.path.join(img, f"{scene}__{job['id']}.png")
        if skip_existing and os.path.isfile(out):
            continue
        try:
            render(job, meshes, points, out)
            done += 1
        except Exception as e:  # keep going: one bad view must not stop the rest
            print(f"  {scene}/{job['id']}: {e}", flush=True)
    return f"{scene}: {done} images"


def main():
    import multiprocessing
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    only_failed = "--only-failed" in sys.argv
    skip = "--skip-existing" in sys.argv
    data, evals = args[0], args[1]
    filt = args[2] if len(args) > 2 else ""
    os.makedirs(os.path.join(evals, "img"), exist_ok=True)
    scenes = [s for s in sorted(os.listdir(evals)) if filt in s and os.path.isfile(os.path.join(evals, s, "eval.json"))]
    workers = max(1, min(len(scenes), (os.cpu_count() or 2) // 2))
    with multiprocessing.Pool(workers) as pool:
        for line in pool.imap_unordered(render_scene, [(data, evals, s, only_failed, skip) for s in scenes]):
            print(line, flush=True)


if __name__ == "__main__":
    main()
