#!/usr/bin/env python3
"""
prepare_reference_depth_distribution.py

Utility to convert a real-world bathymetric depth dataset (e.g., NOAA/USGS survey
CSV or XYZ file) into a NumPy .npy array of depth values that can be referenced by
the validation pipeline via the `referenceDepthFile` common config key.

Usage examples:
  # From a NOAA bathymetric XYZ (space/comma-delimited X Y Z):
  python scripts/prepare_reference_depth_distribution.py \\
      --input noaa_survey.xyz \\
      --output reference_depths/noaa_survey_depths.npy \\
      --format xyz \\
      --depth-column 2

  # From a CSV with a named depth column:
  python scripts/prepare_reference_depth_distribution.py \\
      --input usgs_survey.csv \\
      --output reference_depths/usgs_depths.npy \\
      --format csv \\
      --depth-column depth_m

  # From a LAS file (uses laspy if available):
  python scripts/prepare_reference_depth_distribution.py \\
      --input survey.las \\
      --output reference_depths/las_depths.npy \\
      --format las

  # Inspect an existing .npy reference file:
  python scripts/prepare_reference_depth_distribution.py \\
      --inspect reference_depths/noaa_survey_depths.npy

After creating the .npy file, add to any trial config:
  "common": {
    "referenceDepthFile": "/path/to/reference_depths/noaa_survey_depths.npy",
    ...
  }

The validation pipeline will then run the KS test and histogram comparison
(R14 criterion: KS D < 0.15, histogram depth-bin deviation < 10%) between
the synthetic scan depths and this reference dataset.
"""

import argparse
import os
import sys

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Convert a real-world bathymetric depth dataset to a NumPy .npy reference "
            "file for use with the BLAINDER validation pipeline (R14 KS test)."
        )
    )
    sub = parser.add_subparsers(dest="command")

    # Inspect existing .npy
    inspect_parser = sub.add_parser(
        "inspect", help="Print statistics about an existing .npy reference depth file."
    )
    inspect_parser.add_argument("file", help="Path to .npy file.")

    # Convert
    convert_parser = sub.add_parser(
        "convert", help="Convert a depth dataset to .npy format."
    )
    convert_parser.add_argument("--input", required=True, help="Input file path.")
    convert_parser.add_argument(
        "--output", required=True, help="Output .npy file path."
    )
    convert_parser.add_argument(
        "--format",
        choices=["xyz", "csv", "las", "npy", "txt"],
        default="xyz",
        help="Input file format (default: xyz).",
    )
    convert_parser.add_argument(
        "--depth-column",
        default="2",
        help=(
            "Column index (0-based integer) or name for the depth/Z values. "
            "For XYZ files: 0=X, 1=Y, 2=Z. Default: '2'."
        ),
    )
    convert_parser.add_argument(
        "--delimiter",
        default=None,
        help="Column delimiter for text formats (default: auto-detect whitespace/comma).",
    )
    convert_parser.add_argument(
        "--skip-rows",
        type=int,
        default=0,
        help="Number of header rows to skip (default: 0).",
    )
    convert_parser.add_argument(
        "--depth-sign",
        choices=["positive", "negative", "auto"],
        default="auto",
        help=(
            "Sign convention for depths. 'positive' = Z as-is (deeper = more positive). "
            "'negative' = negate values (survey data often stores depth as positive but "
            "Blender uses negative Z below waterline). 'auto' = keep as-is. Default: auto."
        ),
    )
    convert_parser.add_argument(
        "--min-depth",
        type=float,
        default=None,
        help="Discard depth values below this threshold (e.g. -200 to remove outliers).",
    )
    convert_parser.add_argument(
        "--max-depth",
        type=float,
        default=None,
        help="Discard depth values above this threshold.",
    )

    # If no subcommand, fallback to old positional style (--inspect flag)
    parser.add_argument(
        "--inspect",
        metavar="FILE",
        help="Shorthand: inspect a .npy depth file and print statistics.",
    )
    parser.add_argument("--input", help="Input file path (convert mode).")
    parser.add_argument("--output", help="Output .npy path (convert mode).")
    parser.add_argument(
        "--format",
        choices=["xyz", "csv", "las", "npy", "txt"],
        default="xyz",
    )
    parser.add_argument("--depth-column", default="2")
    parser.add_argument("--delimiter", default=None)
    parser.add_argument("--skip-rows", type=int, default=0)
    parser.add_argument(
        "--depth-sign", choices=["positive", "negative", "auto"], default="auto"
    )
    parser.add_argument("--min-depth", type=float, default=None)
    parser.add_argument("--max-depth", type=float, default=None)

    return parser.parse_args()


def load_xyz_or_txt(path, depth_column, delimiter, skip_rows):
    """Load space/comma-separated XYZ or plain text file."""
    col_idx = int(depth_column)
    depths = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line_num, line in enumerate(fh, start=1):
            if line_num <= skip_rows:
                continue
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(delimiter) if delimiter else line.split()
            try:
                depths.append(float(parts[col_idx]))
            except (IndexError, ValueError):
                continue
    return np.asarray(depths, dtype=float)


def load_csv(path, depth_column, delimiter, skip_rows):
    """Load CSV with optional named column."""
    import csv

    # Try to parse depth_column as integer index first
    try:
        col_idx = int(depth_column)
        col_name = None
    except ValueError:
        col_idx = None
        col_name = depth_column

    depths = []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as fh:
        reader = csv.reader(fh, delimiter=delimiter or ",")
        header = None
        for row_num, row in enumerate(reader, start=1):
            if row_num <= skip_rows:
                continue
            if header is None and col_name is not None:
                header = [c.strip() for c in row]
                if col_name in header:
                    col_idx = header.index(col_name)
                else:
                    print(
                        "ERROR: column '%s' not found in header: %s"
                        % (col_name, header)
                    )
                    sys.exit(1)
                continue
            try:
                depths.append(float(row[col_idx]))
            except (IndexError, ValueError):
                continue
    return np.asarray(depths, dtype=float)


def load_las(path):
    """Load Z values from a LAS/LAZ file using laspy."""
    try:
        import laspy  # type: ignore[import]
    except ImportError:
        print(
            "ERROR: laspy is required to read LAS files. Install with: pip install laspy"
        )
        sys.exit(1)

    las = laspy.read(path)
    return np.asarray(las.z, dtype=float)


def apply_filters(depths, depth_sign, min_depth, max_depth):
    """Apply sign convention and range filters."""
    if depth_sign == "negative":
        depths = -depths

    original_count = len(depths)
    if min_depth is not None:
        depths = depths[depths >= min_depth]
    if max_depth is not None:
        depths = depths[depths <= max_depth]

    removed = original_count - len(depths)
    if removed > 0:
        print("Filtered %d outlier values (range filter)." % removed)

    return depths


def print_stats(depths, label=""):
    """Print descriptive statistics."""
    if len(depths) == 0:
        print("%sNo depth values." % label)
        return
    print("%sDepth statistics:" % label)
    print("  Count  : %d" % len(depths))
    print("  Min    : %.4f m" % depths.min())
    print("  Max    : %.4f m" % depths.max())
    print("  Mean   : %.4f m" % depths.mean())
    print("  Median : %.4f m" % np.median(depths))
    print("  Std    : %.4f m" % depths.std())
    print(
        "  P5/P95 : %.4f / %.4f m"
        % (np.percentile(depths, 5), np.percentile(depths, 95))
    )


def main():
    args = parse_args()

    # Inspect mode
    inspect_target = args.inspect if hasattr(args, "inspect") else None
    if inspect_target:
        if not os.path.exists(inspect_target):
            print("ERROR: File not found: %s" % inspect_target)
            sys.exit(1)
        depths = np.load(inspect_target)
        print("Reference depth file: %s" % inspect_target)
        print_stats(depths)
        return

    if not args.input or not args.output:
        print("ERROR: --input and --output are required for conversion.")
        print("Use --inspect <file> to inspect an existing .npy file.")
        sys.exit(1)

    if not os.path.exists(args.input):
        print("ERROR: Input file not found: %s" % args.input)
        sys.exit(1)

    fmt = args.format.lower()
    depth_column = args.depth_column
    delimiter = args.delimiter
    skip_rows = args.skip_rows

    print("Loading '%s' as format '%s'..." % (args.input, fmt))
    if fmt in ("xyz", "txt"):
        depths = load_xyz_or_txt(args.input, depth_column, delimiter, skip_rows)
    elif fmt == "csv":
        depths = load_csv(args.input, depth_column, delimiter, skip_rows)
    elif fmt == "las":
        depths = load_las(args.input)
    elif fmt == "npy":
        depths = np.load(args.input).ravel().astype(float)
    else:
        print("ERROR: Unknown format '%s'." % fmt)
        sys.exit(1)

    if len(depths) == 0:
        print("ERROR: No depth values extracted from input file.")
        sys.exit(1)

    depths = apply_filters(depths, args.depth_sign, args.min_depth, args.max_depth)
    print_stats(depths)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    np.save(args.output, depths)
    print("Saved %d depth values to: %s" % (len(depths), args.output))
    print()
    print("To use in a trial config, add to 'common':")
    print('  "referenceDepthFile": "%s"' % os.path.abspath(args.output))


if __name__ == "__main__":
    main()
