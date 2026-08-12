#!/usr/bin/env python3
"""Estimate the upward floor normal and floor elevation in a binary PCD.

The RANSAC search is constrained around an expected normal.  This prevents a
large wall from being selected as the floor in room/building maps.
"""

import argparse
import math

import numpy as np

from analyze_pcd_room_coverage import load_xyz


def unit(vector):
    length = float(np.linalg.norm(vector))
    if length <= 1.0e-12:
        raise ValueError("normal must be non-zero")
    return vector / length


def angle_deg(first, second):
    cosine = float(np.clip(np.dot(unit(first), unit(second)), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def refine_plane(points, reference):
    center = np.median(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = unit(vh[-1])
    if np.dot(normal, reference) < 0.0:
        normal = -normal
    offset = float(np.median(points @ normal))
    return normal, offset


def strongest_plane(sample, reference, threshold, max_angle, iterations, seed):
    rng = np.random.default_rng(seed)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        selected = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(selected[1] - selected[0], selected[2] - selected[0])
        length = np.linalg.norm(normal)
        if length <= 1.0e-7:
            continue
        normal /= length
        if np.dot(normal, reference) < 0.0:
            normal = -normal
        if angle_deg(normal, reference) > max_angle:
            continue
        offset = float(np.dot(selected[0], normal))
        mask = np.abs(sample @ normal - offset) <= threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 300:
        raise ValueError("no floor/ceiling plane matched the expected normal")
    return refine_plane(sample[best_mask], reference), best_count


def find_floor_height(
    points, normal, bin_width, minimum_peak_points, minimum_peak_ratio
):
    projected = points @ normal
    low, high = np.percentile(projected, [0.5, 99.5])
    edges = np.arange(low, high + bin_width, bin_width)
    counts, edges = np.histogram(projected, bins=edges)
    peak_threshold = max(
        minimum_peak_points, int(math.ceil(float(counts.max()) * minimum_peak_ratio))
    )
    candidates = np.flatnonzero(counts >= peak_threshold)
    if len(candidates) == 0:
        raise ValueError(
            f"no projected-height bin contains {peak_threshold} points"
        )

    # The floor is the lowest strong plane. Join adjacent populated bins so a
    # plane that lies on a bin boundary is treated as one peak.
    groups = np.split(candidates, np.flatnonzero(np.diff(candidates) > 1) + 1)
    group = min(groups, key=lambda values: edges[int(values[0])])
    start = edges[int(group[0])]
    stop = edges[int(group[-1]) + 1]
    band = projected[(projected >= start) & (projected < stop)]
    if len(band) < peak_threshold:
        raise ValueError("floor peak became too small during refinement")
    return float(np.median(band)), projected, len(band), peak_threshold


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path")
    parser.add_argument(
        "--reference-normal",
        type=float,
        nargs=3,
        default=(0.006, -0.718, 0.696),
        metavar=("NX", "NY", "NZ"),
        help="expected upward floor normal in the unlevelled PCD frame",
    )
    parser.add_argument("--sample-points", type=int, default=120000)
    parser.add_argument("--plane-threshold", type=float, default=0.045)
    parser.add_argument("--max-normal-angle-deg", type=float, default=12.0)
    parser.add_argument("--iterations", type=int, default=360)
    parser.add_argument("--histogram-bin", type=float, default=0.04)
    parser.add_argument("--min-peak-points", type=int, default=300)
    parser.add_argument(
        "--min-peak-ratio",
        type=float,
        default=0.15,
        help="floor peak must contain this fraction of the strongest plane bin",
    )
    parser.add_argument("--seed", type=int, default=20260729)
    args = parser.parse_args()

    xyz = np.asarray(load_xyz(args.pcd_path), dtype=np.float64)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    if len(xyz) < 1000:
        raise ValueError(f"too few finite points: {len(xyz)}")

    reference = unit(np.asarray(args.reference_normal, dtype=np.float64))
    rng = np.random.default_rng(args.seed)
    if len(xyz) > args.sample_points:
        sample = xyz[rng.choice(len(xyz), args.sample_points, replace=False)]
    else:
        sample = xyz

    (normal, plane_offset), ransac_inliers = strongest_plane(
        sample,
        reference,
        args.plane_threshold,
        args.max_normal_angle_deg,
        args.iterations,
        args.seed,
    )
    reference_error = angle_deg(normal, reference)
    if reference_error > args.max_normal_angle_deg:
        raise ValueError(
            f"refined normal differs from reference by {reference_error:.3f} deg"
        )

    floor_z, projected, floor_peak_points, peak_threshold = find_floor_height(
        xyz,
        normal,
        args.histogram_bin,
        args.min_peak_points,
        args.min_peak_ratio,
    )
    floor_residuals = np.abs(projected - floor_z)
    local_residuals = floor_residuals[
        floor_residuals <= args.plane_threshold
    ]
    if len(local_residuals) < args.min_peak_points:
        raise ValueError("too few floor points within the plane threshold")

    correction = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
    p50 = float(np.percentile(local_residuals, 50))
    p99 = float(np.percentile(local_residuals, 99))
    print(f"pcd={args.pcd_path}")
    print(f"points={len(xyz)} sampled={len(sample)}")
    print(
        "reference_normal="
        f"({reference[0]:.9f},{reference[1]:.9f},{reference[2]:.9f})"
    )
    print(
        f"ransac_inliers={ransac_inliers} plane_offset={plane_offset:.6f} "
        f"reference_error_deg={reference_error:.6f}"
    )
    print(
        f"floor_peak_threshold={peak_threshold} floor_peak_points={floor_peak_points} "
        f"floor_local_points={len(local_residuals)} "
        f"residual_p50={p50:.6f}m residual_p99={p99:.6f}m"
    )
    print(f"RESULT_NORMAL={normal[0]:.9f} {normal[1]:.9f} {normal[2]:.9f}")
    print(f"RESULT_FLOOR_Z={floor_z:.9f}")
    print(f"RESULT_TILT_CORRECTION_DEG={correction:.6f}")
    print(f"RESULT_FLOOR_RESIDUAL_P99={p99:.9f}")
    print("RESULT: PASS constrained floor estimate")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"RESULT: FAIL {error}")
        raise SystemExit(2)
