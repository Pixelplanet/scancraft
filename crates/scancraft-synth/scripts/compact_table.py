"""Compact per-scene table from synth_eval's results.json:  python compact_table.py <results.json>"""
import json, sys
R = json.load(open(sys.argv[1]))
print(f"{'scene':27} {'R rms':>6} {'R p99':>6} {'R t>o':>6} {'Rmiss':>6} {'Rprb':>8} {'Rft':>5} {'loops':>5} | {'S rms':>6} {'S p99':>6} {'S t>o':>6} {'Smiss':>6} {'Sprb':>8} {'Sft':>5} {'topo':>9} {'tg':>3} | {'Rs':>4} {'Ss':>4} {'MB':>5}")
for r in R:
    e = {m['mesh']: m for m in r['evals']}
    def c(m, solid):
        g = m['global']; t = m['topology']; nf = len(m['features']); ok = nf - sum(1 for f in m['features'] if f['fails'])
        s = f"{g['out_to_truth']['rms_mm']:6.3f} {g['out_to_truth']['p99_mm']:6.3f} {g['truth_to_out']['p99_mm']:6.3f} {100*g['missing_share']:5.1f}% {m['probes_passed']:>3}/{m['probes']:<4} {ok:>2}/{nf:<2}"
        s += f" {('W' if t['watertight'] else 'open')+' c'+str(t['components'])+' g'+str(t['genus']):>9}" if solid else f" {t['boundary_loops']:>5}"
        return s
    if 'reconstruct' not in e:
        print(r['scene'], 'ERROR', r.get('error')); continue
    sol = c(e['solid'], True) if 'solid' in e else '-'
    print(f"{r['scene']:27} {c(e['reconstruct'], False)} | {sol} {r['truth_topology']['genus']:>3} | {r['reconstruct']['seconds']:4.1f} {r['solid']['seconds']:4.1f} {r['peak_mb'] or 0:5.0f}")
