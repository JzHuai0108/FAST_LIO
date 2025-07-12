#!/usr/bin/env python3
"""
grep_log_warnings.py

Recursively search for forward_states.log and backward_states.log under a root directory,
grep each for configurable keywords/phrases (case‐insensitive), and write matches
to an output file, prefixed by their source path.
"""

import os
import re
import argparse

def parse_args():
    p = argparse.ArgumentParser(
        description="Recursively grep log files for warnings/errors/phrases"
    )
    p.add_argument("root_dir",
                   help="Root directory to search under (will walk subfolders),"
                        "e.g., /media/$USER/MyBookDuo/jhuai/results/front_back_snapshots")
    p.add_argument("output_file",
                   help="Text file to write grep results into")
    return p.parse_args()

def main():
    args = parse_args()

    # List of phrases (as raw strings) to grep for; extend this list as needed
    patterns = [
        r"warn",           # matches warning or WARNING
        r"error",             # matches error or ERROR
        # r"The map state at",  # exact phrase (case‐insensitive)
        # add more phrases here, e.g. r"failed", r"exception", etc.
    ]
    # build one big OR‐regex
    regex = re.compile(
        "|".join(f"(?:{pat})" for pat in patterns),
        re.IGNORECASE
    )

    with open(args.output_file, "w", encoding="utf-8") as fout:
        for dirpath, _, filenames in os.walk(args.root_dir):
            for fname in filenames:
                # only look at the two state‐log filenames
                if not (fname == "forward_states.log" or
                        fname == "backward_states.log"):
                    continue

                fullpath = os.path.join(dirpath, fname)
                try:
                    with open(fullpath, "r", encoding="utf-8", errors="ignore") as fin:
                        for lino, line in enumerate(fin, start=1):
                            if regex.search(line):
                                # write: filepath:line_number: matching line
                                fout.write(f"{fullpath}:{lino}: {line}")
                except OSError as e:
                    print(f"Could not open {fullpath}: {e}")

    print(f"Grep complete; results saved to {args.output_file}")

if __name__ == "__main__":
    main()
