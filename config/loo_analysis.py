"""
Before vs After Optimisation Accuracy Analysis

Compares landmark positions before and after optimisation
against Total Station ground truth, using a 3D similarity transform.

Expects in loo_results/:
    beforeoptimisation.csv   - raw SLAM landmark estimates
    afteroptimisation.csv    - optimised landmark estimates

Expects in loo_splits/:
    Any LOO split CSV (used only to get the surveyed ground truth points)
    OR place a surveyed_points.csv directly in loo_results/
"""

import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm

RESULTS_DIR    = "loo_results"
SPLITS_DIR     = "loo_splits"
BEFORE_CSV     = os.path.join(RESULTS_DIR, "beforeoptimisation.csv")
AFTER_CSV      = os.path.join(RESULTS_DIR, "afteroptimisation.csv")
OUTPUT_SUMMARY = os.path.join(RESULTS_DIR, "before_after_summary.csv")


# ── 3D similarity transform ───────────────────────────────────────────────────

def compute_transform_3d(src, tgt):
    src_c = np.mean(src, axis=0)
    tgt_c = np.mean(tgt, axis=0)
    src_  = src - src_c
    tgt_  = tgt - tgt_c

    scale = np.sqrt(np.sum(tgt_**2) / np.sum(src_**2))

    H = src_.T @ tgt_
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tgt_c - scale * R @ src_c
    return R, t, scale


def transform_3d(pts, R, t, s):
    return (s * pts @ R.T) + t


# ── Helpers ───────────────────────────────────────────────────────────────────

def extract_id(point_name):
    m = re.search(r'\d+', str(point_name))
    return int(m.group()) if m else None


def load_ground_truth():
    """
    Load surveyed ground truth from any LOO split CSV —
    all splits have the same point list, just different split labels.
    """
    split_files = sorted(f for f in os.listdir(SPLITS_DIR) if f.startswith('loo_') and f.endswith('.csv'))
    if not split_files:
        raise FileNotFoundError(f"No LOO split CSVs found in '{SPLITS_DIR}/'")

    df = pd.read_csv(os.path.join(SPLITS_DIR, split_files[0]))

    if 'Z' in df.columns and 'Z (Elevation)' not in df.columns:
        df.rename(columns={'Z': 'Z (Elevation)'}, inplace=True)

    # All points regardless of split label
    rows = []
    for _, row in df.iterrows():
        tag_id = extract_id(row['Point Name'])
        if tag_id is None:
            continue
        rows.append({
            'point_name': row['Point Name'],
            'tag_id':     tag_id,
            'ts_x':       row['X (East)'],
            'ts_y':       row['Y (North)'],
            'ts_z':       row['Z (Elevation)'],
        })
    return pd.DataFrame(rows)


def match_to_ground_truth(opt_df, gt_df):
    """Match optimised landmarks to surveyed ground truth points."""
    rows = []
    for _, gt_row in gt_df.iterrows():
        opt = opt_df[opt_df['id'] == gt_row['tag_id']]
        if opt.empty:
            continue
        rows.append({
            'point_name': gt_row['point_name'],
            'lio_x':      opt.iloc[0]['x'],
            'lio_y':      opt.iloc[0]['y'],
            'lio_z':      opt.iloc[0]['z'],
            'ts_x':       gt_row['ts_x'],
            'ts_y':       gt_row['ts_y'],
            'ts_z':       gt_row['ts_z'],
        })
    return pd.DataFrame(rows)


def compute_errors(matched_df):
    """Fit 3D transform and return per-point errors."""
    src = matched_df[['lio_x', 'lio_y', 'lio_z']].values
    tgt = matched_df[['ts_x',  'ts_y',  'ts_z' ]].values

    R, t, s = compute_transform_3d(src, tgt)
    pred    = transform_3d(src, R, t, s)
    errors  = np.linalg.norm(pred - tgt, axis=1)
    return errors, matched_df['point_name'].values


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_results(points, err_before, err_after):
    rmse_b = np.sqrt(np.mean(err_before**2))
    rmse_a = np.sqrt(np.mean(err_after**2))

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Before vs After Optimisation — 3D Landmark Error Against Survey",
                 fontsize=13, fontweight='bold')

    x = np.arange(len(points))
    w = 0.38

    # ── 1. Side-by-side per-point bars ────────────────────────────────────────
    ax = axes[0]
    ax.bar(x - w/2, err_before, w, label=f'Before  (RMSE={rmse_b:.3f}m)',
           color='steelblue', alpha=0.85, edgecolor='white')
    ax.bar(x + w/2, err_after,  w, label=f'After   (RMSE={rmse_a:.3f}m)',
           color='seagreen',   alpha=0.85, edgecolor='white')
    ax.axhline(rmse_b, color='steelblue', linestyle='--', linewidth=1.2, alpha=0.7)
    ax.axhline(rmse_a, color='seagreen',  linestyle='--', linewidth=1.2, alpha=0.7)
    ax.set_xticks(x)
    ax.set_xticklabels(points, rotation=60, ha='right', fontsize=7)
    ax.set_ylabel('3D Error (m)')
    ax.set_title('Per-Point Error: Before vs After')
    ax.legend(fontsize=8)
    ax.grid(axis='y', alpha=0.3)

    # ── 2. Improvement per point (before - after) ─────────────────────────────
    ax = axes[1]
    improvement = err_before - err_after
    colors = ['seagreen' if v >= 0 else 'tomato' for v in improvement]
    ax.bar(x, improvement, color=colors, alpha=0.85, edgecolor='white')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(points, rotation=60, ha='right', fontsize=7)
    ax.set_ylabel('Error Reduction (m)  [+ve = improved]')
    ax.set_title('Improvement After Optimisation')
    ax.grid(axis='y', alpha=0.3)

    # ── 3. Scatter: before vs after (diagonal = no change) ────────────────────
    ax = axes[2]
    max_val = max(err_before.max(), err_after.max()) * 1.1
    ax.plot([0, max_val], [0, max_val], 'k--', linewidth=1, alpha=0.5, label='No change')
    sc = ax.scatter(err_before, err_after,
                    c=improvement, cmap='RdYlGn', s=60, zorder=3,
                    vmin=-abs(improvement).max(), vmax=abs(improvement).max())
    for i, name in enumerate(points):
        ax.annotate(name, (err_before[i], err_after[i]),
                    textcoords='offset points', xytext=(4, 3), fontsize=6, alpha=0.8)
    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label('Improvement (m)', fontsize=8)
    ax.set_xlabel('Before Optimisation Error (m)')
    ax.set_ylabel('After Optimisation Error (m)')
    ax.set_title('Before vs After Scatter\n(below diagonal = improved)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, 'before_after_comparison.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to '{out_path}'")
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("Loading ground truth...")
    gt_df = load_ground_truth()
    print(f"  {len(gt_df)} surveyed points loaded")

    print("\nLoading before/after CSVs...")
    before_df = pd.read_csv(BEFORE_CSV)
    after_df  = pd.read_csv(AFTER_CSV)

    before_matched = match_to_ground_truth(before_df, gt_df)
    after_matched  = match_to_ground_truth(after_df,  gt_df)

    # Keep only points present in both
    common = set(before_matched['point_name']) & set(after_matched['point_name'])
    before_matched = before_matched[before_matched['point_name'].isin(common)].reset_index(drop=True)
    after_matched  = after_matched[after_matched['point_name'].isin(common)].reset_index(drop=True)

    print(f"  {len(common)} points matched in both before and after")

    err_before, points = compute_errors(before_matched)
    err_after,  _      = compute_errors(after_matched)

    # Summary table
    summary = pd.DataFrame({
        'point_name':    points,
        'err_before_m':  np.round(err_before, 4),
        'err_after_m':   np.round(err_after,  4),
        'improvement_m': np.round(err_before - err_after, 4),
    })

    rmse_b = np.sqrt(np.mean(err_before**2))
    rmse_a = np.sqrt(np.mean(err_after**2))

    print(f"\n{'='*60}")
    print(summary.to_string(index=False))
    print(f"\n{'='*60}")
    print(f"{'':20s}  {'Before':>10s}  {'After':>10s}")
    print(f"{'RMSE':20s}  {rmse_b:>10.4f}m  {rmse_a:>10.4f}m")
    print(f"{'Mean':20s}  {np.mean(err_before):>10.4f}m  {np.mean(err_after):>10.4f}m")
    print(f"{'Median':20s}  {np.median(err_before):>10.4f}m  {np.median(err_after):>10.4f}m")
    print(f"{'Max':20s}  {np.max(err_before):>10.4f}m  {np.max(err_after):>10.4f}m")
    print(f"{'Min':20s}  {np.min(err_before):>10.4f}m  {np.min(err_after):>10.4f}m")
    print(f"{'RMSE improvement':20s}  {rmse_b - rmse_a:>10.4f}m  ({(rmse_b - rmse_a)/rmse_b*100:.1f}%)")

    summary.to_csv(OUTPUT_SUMMARY, index=False)
    print(f"\nSummary saved to '{OUTPUT_SUMMARY}'")

    plot_results(points, err_before, err_after)


if __name__ == "__main__":
    main()
