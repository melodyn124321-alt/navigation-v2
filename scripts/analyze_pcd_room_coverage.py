#!/usr/bin/env python3
"""Summarize a binary Fast-LIO PCD and fit its dominant room planes."""

import argparse
import math
import re

import numpy as np


def percentile(values, ratio):
    return float(np.percentile(values, ratio * 100.0))


def load_xyz(path):
    header = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header ended before DATA")
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                data_offset = stream.tell()
                break
    if header.get("DATA") != "binary":
        raise ValueError("only DATA binary PCD files are supported")
    fields = header["FIELDS"].split()
    sizes = [int(value) for value in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(value) for value in header["COUNT"].split()]
    if any(count != 1 for count in counts):
        raise ValueError("multi-count PCD fields are not supported")
    formats = []
    for name, size, kind in zip(fields, sizes, types):
        code = {("F", 4): "<f4", ("F", 8): "<f8",
                ("I", 4): "<i4", ("U", 4): "<u4"}.get((kind, size))
        if code is None:
            formats.append((name, f"V{size}"))
        else:
            formats.append((name, code))
    count = int(header.get("POINTS", header["WIDTH"]))
    records = np.memmap(
        path, mode="r", dtype=np.dtype(formats), offset=data_offset,
        shape=(count,))
    return np.column_stack((records["x"], records["y"], records["z"]))


def fit_dominant_planes(points, plane_count, threshold, iterations, seed):
    rng = np.random.default_rng(seed)
    remaining = points.copy()
    results = []
    for _ in range(plane_count):
        if len(remaining) < 1000:
            break
        best_mask = None
        best_count = 0
        for _ in range(iterations):
            sample = remaining[rng.choice(len(remaining), 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1.0e-6:
                continue
            normal /= norm
            distance = np.abs((remaining - sample[0]) @ normal)
            mask = distance <= threshold
            count = int(mask.sum())
            if count > best_count:
                best_count = count
                best_mask = mask
        if best_mask is None or best_count < 300:
            break
        inliers = remaining[best_mask]
        center = inliers.mean(axis=0)
        _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal
        residual = np.abs((inliers - center) @ normal)
        tilt = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
        results.append((center, normal, len(inliers), float(np.median(residual)), tilt))
        remaining = remaining[~best_mask]
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path")
    parser.add_argument("--sample-points", type=int, default=80000)
    parser.add_argument("--planes", type=int, default=8)
    parser.add_argument("--plane-threshold", type=float, default=0.06)
    parser.add_argument("--iterations", type=int, default=180)
    parser.add_argument("--max-level-tilt", type=float, default=3.0)
    args = parser.parse_args()

    xyz = np.asarray(load_xyz(args.pcd_path), dtype=np.float64)
    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    print(f"pcd={args.pcd_path}")
    print(f"points={len(xyz)} finite={int(finite.sum())}")
    for axis, name in enumerate(("x", "y", "z")):
        values = xyz[:, axis]
        print(
            f"{name}: min={values.min():.3f} p01={percentile(values, .01):.3f} "
            f"p05={percentile(values, .05):.3f} p50={percentile(values, .50):.3f} "
            f"p95={percentile(values, .95):.3f} p99={percentile(values, .99):.3f} "
            f"max={values.max():.3f} span={np.ptp(values):.3f}m"
        )

    rng = np.random.default_rng(20260714)
    if len(xyz) > args.sample_points:
        sample = xyz[rng.choice(len(xyz), args.sample_points, replace=False)]
    else:
        sample = xyz
    planes = fit_dominant_planes(
        sample, args.planes, args.plane_threshold, args.iterations, 20260714)
    level_planes = []
    print("dominant_planes:")
    for index, (center, normal, count, residual, tilt) in enumerate(planes, 1):
        kind = "floor_or_ceiling" if abs(normal[2]) >= 0.70 else "wall"
        if kind == "floor_or_ceiling":
            level_planes.append(tilt)
        print(
            f"  {index}: kind={kind} sample_inliers={count} "
            f"center=({center[0]:.3f},{center[1]:.3f},{center[2]:.3f}) "
            f"normal=({normal[0]:.4f},{normal[1]:.4f},{normal[2]:.4f}) "
            f"tilt_to_map_z={tilt:.3f}deg median_residual={residual:.4f}m"
        )

    if not level_planes:
        print("RESULT: FAIL no dominant floor/ceiling plane found")
        raise SystemExit(2)
    best_level_tilt = min(level_planes)
    if best_level_tilt > args.max_level_tilt:
        print(
            f"RESULT: FAIL map is not level; best floor/ceiling tilt="
            f"{best_level_tilt:.3f}deg > {args.max_level_tilt:.3f}deg"
        )
        raise SystemExit(2)
    print(
        f"RESULT: PASS level floor/ceiling plane tilt={best_level_tilt:.3f}deg"
    )


if __name__ == "__main__":
    main()
