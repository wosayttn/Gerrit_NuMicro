#!/usr/bin/env python3
"""
gif_to_c_array.py
-----------------
Convert a GIF file into a C array suitable for use with the emWin GIF player.

Output format matches the testGIF1.c style:
    const unsigned char <array_name>[<size>+1] = {
    0xXX,0xXX,...,  (8 bytes per line)
    };

Usage:
    python gif_to_c_array.py <input.gif> [options]

Options:
    -o, --output   Output .c file path  (default: <input_name>.c)
    -n, --name     C array variable name (default: my_gif_data)
    -b, --bytes    Number of bytes per line (default: 8)
    -q, --quiet    Suppress progress messages
"""

import argparse
import os
import sys


def gif_to_c_array(
    gif_path: str,
    output_path: str,
    array_name: str = "my_gif_data",
    bytes_per_line: int = 8,
    quiet: bool = False,
) -> None:
    """Read a GIF file and write a C source file containing the raw data array."""

    # ── Validate input ──────────────────────────────────────────────────────────
    if not os.path.isfile(gif_path):
        print(f"[ERROR] File not found: {gif_path}", file=sys.stderr)
        sys.exit(1)

    with open(gif_path, "rb") as f:
        data = f.read()

    # Basic GIF signature check
    if not (data[:3] == b"GIF"):
        print(f"[WARNING] '{gif_path}' does not start with a GIF signature.", file=sys.stderr)

    total = len(data)
    if not quiet:
        print(f"[INFO] Input : {gif_path}")
        print(f"[INFO] Size  : {total} bytes")
        print(f"[INFO] Array : {array_name}[{total}+1]")
        print(f"[INFO] Output: {output_path}")

    # ── Write C source file ─────────────────────────────────────────────────────
    with open(output_path, "w", newline="\n") as out:
        # Array declaration  (+1 matches the original file style)
        out.write(f"const unsigned char {array_name}[{total}+1] = {{\n")

        for i in range(0, total, bytes_per_line):
            chunk = data[i : i + bytes_per_line]
            hex_values = ",".join(f"0x{b:02X}" for b in chunk)
            out.write(hex_values + ",\n")

        out.write("};\n")

    if not quiet:
        print(f"[INFO] Done. Written {total} bytes to '{output_path}'.")


def build_output_path(gif_path: str, output_arg: str | None) -> str:
    """Derive the output .c file path when -o is not specified."""
    if output_arg:
        return output_arg
    base = os.path.splitext(gif_path)[0]
    return base + ".c"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a GIF file to a C array (emWin / NuMicro style).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input", help="Path to the input GIF file")
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="Output .c file (default: <input_basename>.c)",
    )
    parser.add_argument(
        "-n", "--name",
        default="my_gif_data",
        help="C array variable name (default: my_gif_data)",
    )
    parser.add_argument(
        "-b", "--bytes",
        type=int,
        default=8,
        metavar="N",
        help="Number of bytes per line (default: 8)",
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Suppress progress messages",
    )

    args = parser.parse_args()

    output_path = build_output_path(args.input, args.output)

    gif_to_c_array(
        gif_path=args.input,
        output_path=output_path,
        array_name=args.name,
        bytes_per_line=args.bytes,
        quiet=args.quiet,
    )


if __name__ == "__main__":
    main()
