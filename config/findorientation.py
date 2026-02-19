#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

CORRECT_CSV = "./afteroptimisation_correct.csv"
CURRENT_CSV = "./afteroptimisation.csv"


def load_landmarks(filepath):
    df = pd.read_csv(filepath)
    df.columns = df.columns.str.strip().str.lower()

    id_col = next((c for c in df.columns if 'id' in c or 'tag' in c), df.columns[0])
    x_col  = next((c for c in df.columns if c == 'x'), None)
    y_col  = next((c for c in df.columns if c == 'y'), None)
    z_col  = next((c for c in df.columns if c == 'z'), None)

    if not all([x_col, y_col, z_col]):
        id_col, x_col, y_col, z_col = df.columns[0], df.columns[1], df.columns[2], df.columns[3]

    landmarks = {}
    for _, row in df.iterrows():
        tag_id = int(row[id_col])
        landmarks[tag_id] = np.array([row[x_col], row[y_col], row[z_col]])
    return landmarks


def find_common_landmarks(correct, current):
    common_ids = sorted(set(correct.keys()) & set(current.keys()))
    if len(common_ids) < 3:
        raise ValueError(f"Only {len(common_ids)} common landmarks — need at least 3.")
    print(f"Using {len(common_ids)} common landmarks: {common_ids}")
    src = np.array([current[i] for i in common_ids])
    dst = np.array([correct[i] for i in common_ids])
    return src, dst, common_ids


def estimate_rigid_transform(src, dst):
    centroid_src = src.mean(axis=0)
    centroid_dst = dst.mean(axis=0)
    H = (src - centroid_src).T @ (dst - centroid_dst)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        print("Warning: reflection detected, correcting...")
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_dst - R @ centroid_src
    return R, t


def main():
    print(f"Correct CSV : {CORRECT_CSV}")
    print(f"Current CSV : {CURRENT_CSV}")

    correct = load_landmarks(CORRECT_CSV)
    current = load_landmarks(CURRENT_CSV)
    print(f"Loaded {len(correct)} correct / {len(current)} current landmarks")

    src, dst, common_ids = find_common_landmarks(correct, current)

    R, t = estimate_rigid_transform(src, dst)

    # Residuals
    transformed = (R @ src.T).T + t
    errors = np.linalg.norm(dst - transformed, axis=1)
    print(f"\nResiduals per landmark (metres):")
    for i, tag_id in enumerate(common_ids):
        print(f"  L{tag_id}: {errors[i]:.4f} m")
    print(f"  Mean: {errors.mean():.4f} m  |  Max: {errors.max():.4f} m")

    rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    roll, pitch, yaw = rpy

    print("\n" + "="*55)
    print("  Paste this into your C++ code:")
    print("="*55)
    print(f"  double init_x     = {t[0]:.6f};")
    print(f"  double init_y     = {t[1]:.6f};")
    print(f"  double init_z     = {t[2]:.6f};")
    print(f"  double init_roll  = {roll:.6f};")
    print(f"  double init_pitch = {pitch:.6f};")
    print(f"  double init_yaw   = {yaw:.6f};")
    print("="*55)
    print(f"\nRPY (degrees): roll={np.degrees(roll):.3f}, pitch={np.degrees(pitch):.3f}, yaw={np.degrees(yaw):.3f}")


if __name__ == "__main__":
    main()
