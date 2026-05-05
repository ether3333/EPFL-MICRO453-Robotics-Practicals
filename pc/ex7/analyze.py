#!/usr/bin/env python3
"""
Exercise 7 swimming experiments analysis.

Reads all CSV files from pc/ex7/data/ and produces:
  speed_vs_phase.png   - mean ± std speed vs phase lag (2x2 grid: freq x ampl)
  trajectories_*.png   - x/y trajectories per (freq, ampl) condition

Speed is computed as the linear regression slope of x vs time (m/s), which
is robust to the robot's lateral oscillations and any brief backwards motion.

"""

import os
import re
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DATA_DIR = Path(__file__).parent / "data"
OUT_DIR  = Path(__file__).parent

START_X = 1.0   # m  – virtual start line (matches ex7.cc START constant)
STOP_X  = 5.0   # m  – virtual stop line  (matches ex7.cc STOP  constant)

# ---------------------------------------------------------------------------
# Filename parsing
# ---------------------------------------------------------------------------
_RE = re.compile(r'swim_f([\d.]+)_a([\d.]+)_p([\d.]+)\((\d+)\)\.csv')

def parse_filename(fname):
    m = _RE.match(fname)
    if not m:
        return None
    freq  = float(m.group(1))
    ampl  = float(m.group(2))
    phase = float(m.group(3))
    run   = int(m.group(4))
    return freq, ampl, phase, run

# ---------------------------------------------------------------------------
# Speed computation
# ---------------------------------------------------------------------------
COMPLETED_X = STOP_X - 2   # robot must have reached this x to count

def compute_speed(df):
    """
    Returns (speed_m_s, completed) where completed=True if the robot
    reached STOP_X during the run.

    - Drops NaN rows (from truncated last lines in some CSV files).
    - Drops the initial (0,0) tracker artifact row at the very start.
    - Uses only data up to the maximum x reached, so a robot that drifts
      backwards at the end of a run does not bias the speed estimate.
    """
    t = df['time_s'].values.astype(float)
    x = df['x'].values.astype(float)

    # Drop NaN rows (truncated file endings)
    valid = ~(np.isnan(t) | np.isnan(x))
    t, x = t[valid], x[valid]

    # Drop pre-start tracker artifact (x≈0 at the very beginning)
    keep = ~((np.abs(x) < 0.05) & (t < 0.5))
    t, x = t[keep], x[keep]

    completed = len(x) > 0 and float(np.max(x)) >= COMPLETED_X

    if len(t) < 3:
        return None, completed

    # Truncate at max-x index so backward drift at the end is ignored
    peak_idx = int(np.argmax(x))
    t, x = t[:peak_idx + 1], x[:peak_idx + 1]

    if len(t) < 3:
        return None, completed

    slope = np.polyfit(t, x, 1)[0]   # linear fit → average speed in x (m/s)
    return float(slope), completed

# ---------------------------------------------------------------------------
# Load all data
# ---------------------------------------------------------------------------
# groups[(freq, ampl, phase)] = list of (run_label, speed_m_s, dataframe)
groups = defaultdict(list)

for fname in sorted(os.listdir(DATA_DIR)):
    if not fname.endswith('.csv'):
        continue
    parsed = parse_filename(fname)
    if parsed is None:
        print(f"[skip] unrecognised filename: {fname}", file=sys.stderr)
        continue
    freq, ampl, phase, run = parsed
    df = pd.read_csv(DATA_DIR / fname)
    speed, completed = compute_speed(df)
    groups[(freq, ampl, phase)].append((run, speed, completed, df))

# Sort runs within each group
for key in groups:
    groups[key].sort(key=lambda t: t[0])

# Summary table
print(f"\n{'Setting':<32}  {'runs':>5}  {'mean speed (m/s)':>18}  {'std (m/s)':>10}")
print("-" * 72)
for key in sorted(groups):
    freq, ampl, phase = key
    done_speeds = [s for _, s, c, _ in groups[key] if c and s is not None]
    n_total = len(groups[key])
    mean_v  = np.mean(done_speeds) if done_speeds else float('nan')
    std_v   = np.std(done_speeds, ddof=1) if len(done_speeds) > 1 else float('nan')
    print(f"  f={freq:.2f} Hz  A={ampl:4.1f}°  φ={phase:.2f}  "
          f"  {len(done_speeds)}/{n_total:>2}  {mean_v:>18.4f}  {std_v:>10.4f}")

settings  = sorted(groups)
fa_combos = sorted(set((k[0], k[1]) for k in settings))   # (freq, ampl) pairs

# ---------------------------------------------------------------------------
# Figure 1 – Speed vs phase lag
# ---------------------------------------------------------------------------
freqs  = sorted(set(f for f, _ in fa_combos))
ampls  = sorted(set(a for _, a in fa_combos))
nf, na = len(freqs), len(ampls)

fig1, axes1 = plt.subplots(na, nf, figsize=(5.5 * nf, 4.5 * na),
                            sharex=False, sharey=False, squeeze=False)
fig1.suptitle('Swimming speed vs total phase lag', fontsize=14, fontweight='bold')

for row, ampl in enumerate(ampls):
    for col, freq in enumerate(freqs):
        ax = axes1[row][col]
        if (freq, ampl) not in fa_combos:
            ax.set_visible(False)
            continue

        phases = sorted(set(k[2] for k in settings if k[0] == freq and k[1] == ampl))
        means, stds, ns = [], [], []
        for phase in phases:
            # Only include runs where the robot reached the stop line
            speeds = [s for _, s, c, _ in groups[(freq, ampl, phase)] if c and s is not None]
            means.append(np.mean(speeds) if speeds else float('nan'))
            stds.append(np.std(speeds, ddof=1) if len(speeds) > 1 else 0.0)
            ns.append(len(speeds))

        ax.errorbar(phases, means, yerr=stds,
                    fmt='o-', capsize=7, capthick=1.8,
                    linewidth=2, markersize=8, color='steelblue')

        # Annotate individual completed-run speeds as scatter
        for phase in phases:
            speeds = [s for _, s, c, _ in groups[(freq, ampl, phase)] if c and s is not None]
            ax.scatter([phase] * len(speeds), speeds,
                       color='steelblue', alpha=0.35, s=25, zorder=3)

        # n= annotation above each mean
        for p, m, n in zip(phases, means, ns):
            ax.annotate(f'n={n}', xy=(p, m), xytext=(0, 10),
                        textcoords='offset points',
                        ha='center', fontsize=8, color='gray')

        ax.set_xlabel('Total phase lag φ', fontsize=11)
        ax.set_ylabel('Average speed (m/s)', fontsize=11)
        ax.set_title(f'f = {freq} Hz,   A = {ampl}°', fontsize=11)
        ax.set_xticks(phases)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(min(phases) - 0.1, max(phases) + 0.1)

fig1.tight_layout()
out1 = OUT_DIR / 'speed_vs_phase.png'
fig1.savefig(out1, dpi=150)
print(f"\nSaved {out1}")

# ---------------------------------------------------------------------------
# Figure 2+ – Trajectories, one figure per (freq, ampl)
# ---------------------------------------------------------------------------
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

for freq, ampl in fa_combos:
    phases = sorted(set(k[2] for k in settings if k[0] == freq and k[1] == ampl))
    n_ph   = len(phases)

    fig2, axes2 = plt.subplots(1, n_ph,
                                figsize=(5.5 * n_ph, 4.5),
                                sharey=True, squeeze=False)
    fig2.suptitle(f'Trajectories  -  f = {freq} Hz,   A = {ampl}°',
                  fontsize=13, fontweight='bold')

    for col, phase in enumerate(phases):
        ax = axes2[0][col]
        runs = groups[(freq, ampl, phase)]

        for idx, (run, speed, completed, df) in enumerate(runs):
            color   = colors[idx % len(colors)]
            ax.plot(df['x'], df['y'],
                    color=color, alpha=0.8, linewidth=1.2,
                    linestyle='-' if completed else '--',
                    label=f'run {idx + 1}  ({speed:.3f} m/s)' if (completed and speed is not None) else f'run {idx + 1}  (DNF)')

        ax.set_xlabel('x (m)', fontsize=11)
        if col == 0:
            ax.set_ylabel('y (m)', fontsize=11)
        ax.set_title(f'φ = {phase}', fontsize=11)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

    fig2.tight_layout()
    out2 = OUT_DIR / f'trajectories_f{freq:.2f}_a{ampl:.1f}.png'
    fig2.savefig(out2, dpi=150)
    print(f"Saved {out2}")

plt.show()
