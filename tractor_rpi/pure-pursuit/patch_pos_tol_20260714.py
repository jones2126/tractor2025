#!/usr/bin/env python3
"""
patch_pos_tol_20260714.py -- applies the pos_tol changes to
pure_pursuit_controller_20260714.py in place.

Changes applied (see chat 2026-07-14 for rationale):
  1. __init__ default pos_tol 0.1 -> 0.5 (~line 198)
  2. pos_tol docstring: documents it as an ALONG-TRACK goal window (~line 207)
  3. End-of-path check `abs(rel_x) <= pos_tol` -> one-sided `rel_x <= pos_tol`
     so overshooting the window can never fall through to the
     extend-lookahead branch and drive past the goal forever (~line 346)
  4. argparse: NEW --pos-tol option (default 0.5) (~line 610)
  5. PurePursuit(...) call: passes pos_tol=args.pos_tol (~line 618)

Safety behavior:
  - Verifies EVERY old string appears exactly once BEFORE writing anything.
    If any check fails (file already patched, or a different version),
    exits with an error and the file is untouched.
  - Writes a .bak backup next to the original before modifying it.
  - Verifies the result still compiles (py_compile) after patching;
    restores the backup automatically if it doesn't.

Usage:
    python3 patch_pos_tol_20260714.py [path/to/pure_pursuit_controller_20260714.py]
    (default path: ./pure_pursuit_controller_20260714.py)
"""

import sys
import shutil
import py_compile

TARGET_DEFAULT = "pure_pursuit_controller_20260714.py"

# (description, old_exact_string, new_string)
PATCHES = [
    (
        "1. __init__ default pos_tol 0.1 -> 0.5",
        "    def __init__(self, wheelbase=1.27, max_steer=0.623, pos_tol=0.1,",
        "    def __init__(self, wheelbase=1.27, max_steer=0.623, pos_tol=0.5,   # CHANGED 20260714: was 0.1",
    ),
    (
        "2. pos_tol docstring",
        "        :param pos_tol: Position tolerance for goal reaching in meters.",
        """        :param pos_tol: Along-track goal window in meters (default 0.5,
               ~half vehicle length). Mission ends when the final waypoint is
               within this distance ahead of (or anywhere behind) base_link.
               Lateral error is NOT part of this check. Physical stopping
               distance (actuator travel + hydro coast) adds on top.""",
    ),
    (
        "3. one-sided end-of-path check",
        """            if abs(rel_x) <= self.pos_tol:
                self.goal_reached = True
                return 0.0, 0.0""",
        """            # CHANGED 20260714: was `if abs(rel_x) <= self.pos_tol:`
            # rel_x is signed along-track distance to the goal (+ = still
            # ahead of us). Declare goal reached when it's within pos_tol
            # ahead OR already behind us -- so overshooting the window can
            # never fall through to the extend-lookahead branch and drive
            # past the goal indefinitely.
            if rel_x <= self.pos_tol:
                self.goal_reached = True
                return 0.0, 0.0""",
    ),
    (
        "4. argparse --pos-tol option",
        """    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f'Safety cap on commanded speed in m/s (default: {DEFAULT_MAX_SPEED_MPS})')""",
        """    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help=f'Safety cap on commanded speed in m/s (default: {DEFAULT_MAX_SPEED_MPS})')
    parser.add_argument('--pos-tol', type=float, default=0.5,
                        help='Along-track goal window in meters (default: 0.5)')""",
    ),
    (
        "5. pass pos_tol through to PurePursuit()",
        "        max_speed_mps=args.max_speed,\n    )",
        "        max_speed_mps=args.max_speed,\n        pos_tol=args.pos_tol,\n    )",
    ),
]


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else TARGET_DEFAULT

    try:
        with open(target, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"ERROR: '{target}' not found. Run from the directory containing it,")
        print(f"or pass the path: python3 {sys.argv[0]} /path/to/{TARGET_DEFAULT}")
        sys.exit(1)

    # ---- Pre-flight: every patch target must appear exactly once ----
    ok = True
    for desc, old, _new in PATCHES:
        n = content.count(old)
        if n == 0:
            print(f"FAIL  {desc}: target text not found "
                  f"(file already patched, or a different version?)")
            ok = False
        elif n > 1:
            print(f"FAIL  {desc}: target text found {n} times (ambiguous)")
            ok = False
        else:
            print(f"ok    {desc}: target found")
    if not ok:
        print("\nNo changes made -- file is untouched.")
        sys.exit(1)

    # ---- Backup, then apply ----
    backup = target + ".bak"
    shutil.copy2(target, backup)
    print(f"\nBackup written: {backup}")

    for desc, old, new in PATCHES:
        content = content.replace(old, new, 1)

    with open(target, 'w') as f:
        f.write(content)

    # ---- Post-flight: must still compile ----
    try:
        py_compile.compile(target, doraise=True)
    except py_compile.PyCompileError as e:
        shutil.copy2(backup, target)
        print(f"ERROR: patched file failed to compile -- original restored from backup.")
        print(e)
        sys.exit(1)

    print(f"All {len(PATCHES)} patches applied to '{target}' -- compiles clean.")
    print("Review with:  git diff  (or: diff " + backup + " " + target + ")")


if __name__ == "__main__":
    main()
