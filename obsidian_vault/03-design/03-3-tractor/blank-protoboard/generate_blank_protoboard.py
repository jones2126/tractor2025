#!/usr/bin/env python3
"""
Generate a blank 24x18 protoboard for KiCad 9.

Grid matches a physical perf board labeled A-X across the top/bottom
(24 columns) and 1-18 down the left/right edges (18 rows), 2.54 mm pitch.

Every hole is a real plated thru-hole copper pad (routable on both layers)
named by its grid coordinate (A1, B5, X18, ...).  No components, no traces --
add the connections yourself in the PCB editor.

Follows the rules in:
  obsidian_vault/04-reference/KiCad9_Programmatic_PCB_Lessons.md
"""
import uuid

# ---- board definition -------------------------------------------------------
COLS = "ABCDEFGHIJKLMNOPQRSTUVWX"   # 24 columns, A..X
ROWS = list(range(1, 19))           # 18 rows, 1..18
PITCH = 2.54                        # mm, standard perf-board spacing

PAD_SIZE = 1.8                      # copper pad diameter, mm
DRILL = 1.0                         # hole drill diameter, mm

# top-left hole (A1) position on the page, mm
X0 = 100.0
Y0 = 80.0

MARGIN = 2.54                       # board edge distance beyond outermost holes
LABEL_GAP = 2.0                     # silkscreen label distance beyond holes


def u():
    return str(uuid.uuid4())


def col_x(c):      # column index 0..23 -> x mm
    return X0 + c * PITCH


def row_y(r):      # row index 0..17 -> y mm
    return Y0 + r * PITCH


def f(v):          # tidy float formatting
    return f"{v:.4f}".rstrip("0").rstrip(".")


# ---- geometry ---------------------------------------------------------------
x_min = col_x(0) - MARGIN
x_max = col_x(len(COLS) - 1) + MARGIN
y_min = row_y(0) - MARGIN
y_max = row_y(len(ROWS) - 1) + MARGIN

parts = []

# header ---------------------------------------------------------------------
parts.append(f"""(kicad_pcb
\t(version 20241229)
\t(generator "generate_blank_protoboard")
\t(generator_version "9.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(9 "F.Adhes" user "F.Adhesive")
\t\t(11 "B.Adhes" user "B.Adhesive")
\t\t(13 "F.Paste" user)
\t\t(15 "B.Paste" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(25 "Edge.Cuts" user)
\t\t(27 "Margin" user)
\t\t(31 "F.CrtYd" user "F.Courtyard")
\t\t(29 "B.CrtYd" user "B.Courtyard")
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints no)
\t)
\t(net 0 "")""")

# board outline (Edge.Cuts) --------------------------------------------------
def edge(x1, y1, x2, y2):
    return f"""\t(gr_line
\t\t(start {f(x1)} {f(y1)})
\t\t(end {f(x2)} {f(y2)})
\t\t(stroke (width 0.1) (type solid))
\t\t(layer "Edge.Cuts")
\t\t(uuid "{u()}")
\t)"""

parts.append(edge(x_min, y_min, x_max, y_min))
parts.append(edge(x_max, y_min, x_max, y_max))
parts.append(edge(x_max, y_max, x_min, y_max))
parts.append(edge(x_min, y_max, x_min, y_min))

# edge labels (A-X top/bottom, 1-18 left/right) ------------------------------
def text(s, x, y):
    return f"""\t(gr_text "{s}"
\t\t(at {f(x)} {f(y)})
\t\t(layer "F.SilkS")
\t\t(uuid "{u()}")
\t\t(effects (font (size 1 1) (thickness 0.15)))
\t)"""

for c, letter in enumerate(COLS):
    parts.append(text(letter, col_x(c), y_min + LABEL_GAP))         # top
    parts.append(text(letter, col_x(c), y_max - LABEL_GAP))         # bottom
for r, num in enumerate(ROWS):
    parts.append(text(str(num), x_min + LABEL_GAP, row_y(r)))       # left
    parts.append(text(str(num), x_max - LABEL_GAP, row_y(r)))       # right

# the protoboard footprint (holds every hole as a copper pad) ----------------
pad_lines = []
for c, letter in enumerate(COLS):
    for r, num in enumerate(ROWS):
        name = f"{letter}{num}"
        pad_lines.append(f"""\t\t(pad "{name}" thru_hole circle
\t\t\t(at {f(col_x(c))} {f(row_y(r))})
\t\t\t(size {f(PAD_SIZE)} {f(PAD_SIZE)})
\t\t\t(drill {f(DRILL)})
\t\t\t(layers "*.Cu" "*.Mask")
\t\t\t(uuid "{u()}")
\t\t)""")

footprint = f"""\t(footprint "Protoboard:Grid_24x18"
\t\t(layer "F.Cu")
\t\t(uuid "{u()}")
\t\t(at 0 0)
\t\t(descr "Blank 24x18 protoboard, 2.54mm pitch, A-X by 1-18")
\t\t(tags "protoboard perfboard")
\t\t(attr through_hole)
\t\t(fp_text reference "BRD1" (at {f(col_x(0))} {f(y_min + LABEL_GAP)}) (layer "F.SilkS") hide
\t\t\t(uuid "{u()}")
\t\t\t(effects (font (size 1 1) (thickness 0.15))))
\t\t(fp_text value "Protoboard_24x18" (at 0 0) (layer "F.Fab") hide
\t\t\t(uuid "{u()}")
\t\t\t(effects (font (size 1 1) (thickness 0.15))))
{chr(10).join(pad_lines)}
\t)"""
parts.append(footprint)

parts.append(")")

content = "\n".join(parts) + "\n"

# enforce ASCII-only (KiCad 9 S-expression parser rejects non-ASCII) ---------
bad = [ch for ch in content if ord(ch) > 127]
if bad:
    raise ValueError(f"Non-ASCII characters found: {bad[:10]}")

with open("Blank_Protoboard_24x18.kicad_pcb", "w", encoding="ascii") as fh:
    fh.write(content)

print(f"Wrote Blank_Protoboard_24x18.kicad_pcb")
print(f"  {len(COLS)} cols x {len(ROWS)} rows = {len(COLS)*len(ROWS)} holes")
print(f"  board {f(x_max-x_min)} x {f(y_max-y_min)} mm")
