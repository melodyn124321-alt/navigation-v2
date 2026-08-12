#!/usr/bin/env python3
"""Rotate a binary PCD so a measured floor normal becomes map +Z.

The input is never modified. XYZ and normal_x/normal_y/normal_z fields are
rotated; all other binary fields and the original PCD header are preserved.
"""

import argparse
import math
import os
import struct
import sys


def normalize(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1.0e-12:
        raise ValueError("normal must be non-zero")
    return tuple(value / length for value in vector)


def rotation_from_to(source, target=(0.0, 0.0, 1.0)):
    source = normalize(source)
    target = normalize(target)
    vx = source[1] * target[2] - source[2] * target[1]
    vy = source[2] * target[0] - source[0] * target[2]
    vz = source[0] * target[1] - source[1] * target[0]
    sine = math.sqrt(vx * vx + vy * vy + vz * vz)
    cosine = max(-1.0, min(1.0, sum(a * b for a, b in zip(source, target))))
    if sine <= 1.0e-12:
        if cosine > 0.0:
            return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        raise ValueError("180 degree normal reversal is ambiguous")
    kx, ky, kz = vx / sine, vy / sine, vz / sine
    one_minus_cosine = 1.0 - cosine
    return (
        (
            cosine + kx * kx * one_minus_cosine,
            kx * ky * one_minus_cosine - kz * sine,
            kx * kz * one_minus_cosine + ky * sine,
        ),
        (
            ky * kx * one_minus_cosine + kz * sine,
            cosine + ky * ky * one_minus_cosine,
            ky * kz * one_minus_cosine - kx * sine,
        ),
        (
            kz * kx * one_minus_cosine - ky * sine,
            kz * ky * one_minus_cosine + kx * sine,
            cosine + kz * kz * one_minus_cosine,
        ),
    )


def rotate(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def read_header(stream):
    lines = []
    values = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PCD header ended before DATA")
        lines.append(line)
        text = line.decode("ascii", errors="strict").strip()
        if not text or text.startswith("#"):
            continue
        key, _, value = text.partition(" ")
        values[key.upper()] = value.strip()
        if key.upper() == "DATA":
            break
    if values.get("DATA") != "binary":
        raise ValueError("only DATA binary PCD is supported")
    return lines, values


def field_layout(header):
    fields = header["FIELDS"].split()
    sizes = [int(value) for value in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(value) for value in header["COUNT"].split()]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("inconsistent PCD field metadata")
    offsets = {}
    offset = 0
    for name, size, kind, count in zip(fields, sizes, types, counts):
        offsets[name] = (offset, size, kind, count)
        offset += size * count
    return offsets, offset


def float_offset(offsets, field):
    if field not in offsets:
        raise ValueError(f"PCD is missing required field: {field}")
    offset, size, kind, count = offsets[field]
    if (size, kind, count) != (4, "F", 1):
        raise ValueError(f"{field} must be a scalar float32 field")
    return offset


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pcd")
    parser.add_argument("output_pcd")
    parser.add_argument(
        "--normal",
        type=float,
        nargs=3,
        required=True,
        metavar=("NX", "NY", "NZ"),
        help="measured upward floor normal in the input PCD frame",
    )
    args = parser.parse_args()

    input_path = os.path.abspath(args.input_pcd)
    output_path = os.path.abspath(args.output_pcd)
    if input_path == output_path:
        raise ValueError("input and output paths must differ")
    if os.path.exists(output_path):
        raise FileExistsError(f"refusing to overwrite existing output: {output_path}")

    matrix = rotation_from_to(tuple(args.normal))
    with open(input_path, "rb") as source:
        header_lines, header = read_header(source)
        offsets, stride = field_layout(header)
        point_count = int(header.get("POINTS", header["WIDTH"]))
        data = bytearray(source.read())

    expected_bytes = point_count * stride
    if len(data) < expected_bytes:
        raise ValueError(
            f"binary payload is truncated: got {len(data)}, expected at least {expected_bytes}"
        )
    trailing_bytes = len(data) - expected_bytes

    xyz_offsets = tuple(float_offset(offsets, field) for field in ("x", "y", "z"))
    normal_names = ("normal_x", "normal_y", "normal_z")
    normal_offsets = None
    if all(name in offsets for name in normal_names):
        normal_offsets = tuple(float_offset(offsets, field) for field in normal_names)

    for index in range(point_count):
        base = index * stride
        point = tuple(struct.unpack_from("<f", data, base + off)[0] for off in xyz_offsets)
        rotated = rotate(matrix, point)
        for off, value in zip(xyz_offsets, rotated):
            struct.pack_into("<f", data, base + off, value)
        if normal_offsets is not None:
            normal = tuple(
                struct.unpack_from("<f", data, base + off)[0] for off in normal_offsets
            )
            rotated_normal = rotate(matrix, normal)
            for off, value in zip(normal_offsets, rotated_normal):
                struct.pack_into("<f", data, base + off, value)

    temporary_path = output_path + ".tmp"
    try:
        with open(temporary_path, "xb") as target:
            target.writelines(header_lines)
            target.write(data)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary_path, output_path)
    finally:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)

    corrected = rotate(matrix, normalize(tuple(args.normal)))
    angle = math.degrees(
        math.acos(max(-1.0, min(1.0, normalize(tuple(args.normal))[2])))
    )
    print(f"input={input_path}")
    print(f"output={output_path}")
    print(
        f"points={point_count} stride={stride} trailing_bytes={trailing_bytes} "
        f"bytes={os.path.getsize(output_path)}"
    )
    print(f"applied_tilt_correction_deg={angle:.6f}")
    print(
        "corrected_normal="
        f"({corrected[0]:.9f},{corrected[1]:.9f},{corrected[2]:.9f})"
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
