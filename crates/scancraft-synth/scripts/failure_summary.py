"""Failures of a synth_eval run grouped by feature kind, per mesh: how many features of the kind
fail, in which scenes, the reasons and the metrics that say how badly.

    python failure_summary.py <eval dir>/results.json [kind filter]
"""

import json
import sys
from collections import defaultdict

KEYS = ["centre_dev_mm", "site_rms_mm", "open_share", "ring_mm", "curvature_followed", "width_mid_mm", "diameter_mid_mm", "open", "stations",
        "depth_kept", "height_kept", "pilot_mm", "edge_dist_median_mm", "rounding_radius_median_mm", "edge_bead_max_mm", "top_relief_std_mm",
        "inner_sheet_share", "window_p01_mm", "strays_after_merge", "strays_after_prepare", "tops_missing"]


def main():
    res = json.load(open(sys.argv[1], encoding="utf-8"))
    filt = sys.argv[2] if len(sys.argv) > 2 else ""
    total = defaultdict(int)
    fails = defaultdict(list)
    for s in res:
        for m in s.get("evals", []):
            for f in m["features"]:
                if filt not in f["kind"]:
                    continue
                total[(m["mesh"], f["kind"])] += 1
                if f["fails"]:
                    met = {k: f["metrics"][k] for k in KEYS if k in f["metrics"] and f["metrics"][k] is not None}
                    fails[(m["mesh"], f["kind"])].append((s["scene"], f["id"], "; ".join(f["fails"]), met))
    for key in sorted(total, key=lambda k: (-len(fails[k]), k)):
        mesh, kind = key
        print(f"== {mesh} / {kind}: {len(fails[key])} of {total[key]} fail")
        for scene, fid, why, met in fails[key]:
            ms = ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}" for k, v in met.items())
            print(f"   {scene}/{fid}: {why[:150]} | {ms[:200]}")
        # Pass offsets.
    for s in res:
        for m in s.get("evals", []):
            po = m.get("pass_offset")
            if po and po.get("handled") is False:
                print(f"== pass offset {s['scene']} {m['mesh']}: step {po.get('step_mm')} bias {po.get('bias_mm')} "
                      f"(pass 1 {po['pass_1_only']['mean_mm']}, other {po['other_passes_only']['mean_mm']}, overlap {po['overlap']['mean_mm']})")


if __name__ == "__main__":
    main()
