"""
Generate a cross-hatch shaded sphere as a single continuous G1 path for
the ME306 CoreXY plotter.

The machine has no pen lift: every commanded move puts ink on the paper,
so the whole drawing is ONE polyline. Shading therefore comes from line
density -- four hatch layers at four angles, each clipped to the part of
the sphere darker than its threshold. The highlight keeps bare paper,
the shadow side collects all four layers crossing into a dense mesh.

Firmware constraints this generator respects (see GCodeParser.cpp):
  * X and Y are INTEGERS -- a decimal point rejects the whole line.
  * Every G1 needs both X and Y; F is modal, clamped to 1200.
  * X/Y are RELATIVE offsets, accumulated from the post-G28 origin, and
    a move may not leave 0..MAX_X / 0..MAX_Y.

Outputs the .gcode file and a PNG preview rendered from a simulation of
the emitted file, so the preview shows what the machine will actually
draw rather than the idealised path.
"""

import math
import os

from PIL import Image, ImageDraw

# ---------------------------------------------------------------------
# Machine limits (mirror of Manager::max_x / max_y and cfg::MAX_FEED)
# ---------------------------------------------------------------------

MAX_X = 210
MAX_Y = 140
FEED = 1200

# ---------------------------------------------------------------------
# The sphere
# ---------------------------------------------------------------------
#  Centred in the workspace, radius chosen to keep a 15 mm margin from
#  every soft limit so encoder-count drift over a few hundred moves can
#  never carry the pen into a limit switch.

CX, CY, R = 105.0, 70.0, 55.0

# Light from the upper left, toward the viewer.
_LX, _LY, _LZ = -0.45, 0.55, 0.70
_LN = math.sqrt(_LX * _LX + _LY * _LY + _LZ * _LZ)
LX, LY, LZ = _LX / _LN, _LY / _LN, _LZ / _LN


def _smoothstep(edge0, edge1, x):
    if edge0 == edge1:
        return 0.0 if x < edge0 else 1.0
    t = (x - edge0) / (edge1 - edge0)
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def tone(x, y):
    """Darkness of the sphere at (x, y), 0 = bare paper, 1 = solid."""

    dx, dy = x - CX, y - CY
    d = math.hypot(dx, dy)

    if d > R:
        return 0.0

    nx, ny = dx / R, dy / R
    nz = math.sqrt(max(0.0, 1.0 - nx * nx - ny * ny))

    diffuse = max(0.0, nx * LX + ny * LY + nz * LZ)

    lit = 0.06 + 0.94 * (diffuse ** 0.80)

    # A little bounce light climbing the lower-right limb keeps the dark
    # side rounded instead of blocking up into a flat silhouette.
    bounce = max(0.0, -(nx * LX + ny * LY)) * _smoothstep(0.55, 1.0, d / R)
    lit += 0.20 * bounce

    return max(0.0, min(1.0, 1.0 - lit))


def in_sphere(x, y):
    # Fractionally inside the outline so hatch ends land on the drawn
    # circle instead of dithering across it.
    return math.hypot(x - CX, y - CY) <= R - 0.5


# ---------------------------------------------------------------------
# Hatching
# ---------------------------------------------------------------------
#  Each layer is a family of parallel lines at one angle, kept where the
#  sphere is at least as dark as the layer's threshold. Light areas keep
#  no layer or one; the core shadow keeps all four, crossing.

LAYERS = [
    # (angle degrees, spacing mm, darkness threshold)
    (45,  4.0, 0.22),
    (135, 4.0, 0.45),
    (0,   4.0, 0.62),
    (90,  4.0, 0.78),
]

STEP = 0.5          # sampling step along a hatch line, mm
MIN_SPAN = 3.0      # discard spans shorter than this, mm


def hatch_layer(angle_deg, spacing, threshold):
    """Spans of one layer, as records (line, t0, t1, p0, p1).

    `line` is the index within the parallel family and `t0..t1` the
    extent along the hatch direction; the region grouping below uses
    them to find which spans belong to the same blob of tone.
    """

    a = math.radians(angle_deg)
    ux, uy = math.cos(a), math.sin(a)        # along the hatch line
    vx, vy = -uy, ux                          # across the family

    spans = []
    n = int(R / spacing) + 1

    for i in range(-n, n + 1):
        ox, oy = CX + vx * spacing * i, CY + vy * spacing * i

        run_t = None

        def point(t):
            return (ox + ux * t, oy + uy * t)

        def close(t_end):
            if run_t is not None and t_end - run_t >= MIN_SPAN:
                spans.append((i, run_t, t_end, point(run_t), point(t_end)))

        t = -R
        while t <= R:
            px, py = point(t)
            inside = in_sphere(px, py) and tone(px, py) >= threshold

            if inside and run_t is None:
                run_t = t
            elif not inside and run_t is not None:
                close(t - STEP)
                run_t = None

            t += STEP

        close(R)

    return spans


def group_regions(spans):
    """Split a layer's spans into connected blobs of tone.

    With the pen permanently down, the route between two spans is drawn.
    A layer traversed as one naive serpentine lays a connector straight
    across every light area that interrupts a hatch line. Grouping into
    blobs first means each connector only reaches the neighbouring hatch
    line inside the same blob, so it lands on the blob's own edge and
    reads as a contour rather than a stray slash.

    Two spans join when they sit on adjacent hatch lines and their
    extents along the hatch direction overlap.
    """

    parent = list(range(len(spans)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    by_line = {}
    for idx, (line, t0, t1, _, _) in enumerate(spans):
        by_line.setdefault(line, []).append(idx)

    for line, members in by_line.items():
        for idx in members:
            _, t0, t1, _, _ = spans[idx]
            for jdx in by_line.get(line + 1, ()):
                _, u0, u1, _, _ = spans[jdx]
                if min(t1, u1) - max(t0, u0) > 0.0:
                    union(idx, jdx)

    regions = {}
    for idx in range(len(spans)):
        regions.setdefault(find(idx), []).append(idx)

    return [[spans[i] for i in sorted(group, key=lambda k: spans[k][0])]
            for group in regions.values()]


def chain_region(region):
    """Serpentine one blob, alternating direction line by line."""

    pts = []
    flip = False
    current_line = None

    for line, _, _, p0, p1 in region:
        if current_line is not None and line != current_line:
            flip = not flip
        current_line = line

        if flip:
            p0, p1 = p1, p0

        pts.append(p0)
        pts.append(p1)

    return pts


def chain_layer(spans, start):
    """One layer as a point list: regions ordered nearest-first."""

    regions = [r for r in group_regions(spans) if len(r) >= 2]
    chains = [chain_region(r) for r in regions]

    pts = []
    here = start

    while chains:
        best, rev, best_d = None, False, None

        for i, c in enumerate(chains):
            for r in (False, True):
                head = c[-1] if r else c[0]
                d = math.hypot(head[0] - here[0], head[1] - here[1])
                if best_d is None or d < best_d:
                    best_d, best, rev = d, i, r

        c = chains.pop(best)
        if rev:
            c = list(reversed(c))

        pts += c
        here = c[-1]

    return pts


# ---------------------------------------------------------------------
# The whole drawing
# ---------------------------------------------------------------------

def circle(cx, cy, r, segments=36, start_angle=math.pi):
    """A closed polygon approximating a circle."""

    pts = []
    for i in range(segments + 1):
        a = start_angle + 2.0 * math.pi * i / segments
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def build_path():
    """The drawing as one list of absolute (x, y) points in mm."""

    # Lead in from the homed origin at the sphere's own height, so the
    # one unavoidable entry stroke arrives tangent to the outline instead
    # of slashing across the page.
    pts = [(0, 0), (0, CY), (CX - R, CY)]

    # The outline. It gives the shading an edge to belong to -- without
    # it the lit limb has no boundary and the form dissolves.
    pts += circle(CX, CY, R)

    # Hatch layers, lightest to darkest. Everything from here on stays
    # inside the outline, so every connector lands on sphere.
    for angle, spacing, threshold in LAYERS:
        layer = chain_layer(hatch_layer(angle, spacing, threshold), pts[-1])
        if layer:
            pts += layer

    return pts


# ---------------------------------------------------------------------
# G-code emission
# ---------------------------------------------------------------------

def to_gcode(points):
    """Integer, relative G1 moves, starting from the post-G28 origin."""

    ipts = [(int(round(x)), int(round(y))) for x, y in points]

    # Drop repeats: a zero-length move is accepted but wastes a whole
    # accel/settle cycle doing nothing.
    dedup = [ipts[0]]
    for p in ipts[1:]:
        if p != dedup[-1]:
            dedup.append(p)

    lines = [
        "; ME306 CoreXY plotter -- cross-hatch shaded sphere",
        "; Single continuous stroke: no pen lift, every move draws.",
        "; Shading is line density from four hatch layers at four angles.",
        "; X/Y are relative integers; F is clamped to 1200.",
        "G28",
    ]

    cx, cy = 0, 0
    first = True

    for x, y in dedup:
        dx, dy = x - cx, y - cy
        if dx == 0 and dy == 0:
            continue

        if first:
            lines.append("G1 X%d Y%d F%d" % (dx, dy, FEED))
            first = False
        else:
            lines.append("G1 X%d Y%d" % (dx, dy))

        cx, cy = x, y

    return "\n".join(lines) + "\n", dedup


# ---------------------------------------------------------------------
# Simulation -- re-parse the emitted file the way the firmware does
# ---------------------------------------------------------------------

def simulate(text):
    """Replay the G-code under the firmware's rules; return the path.

    Raises on anything the firmware would reject, so a file that
    simulates clean is a file the machine will accept line for line.
    """

    x, y = 0, 0
    have_f = False
    path = [(0, 0)]
    moves = 0

    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split(";")[0].strip()
        if not line:
            continue

        if line == "G28":
            x, y = 0, 0
            path = [(0, 0)]
            continue

        if not line.startswith("G1"):
            raise ValueError("line %d: unsupported command %r" % (n, line))

        dx = dy = None
        for tok in line.split()[1:]:
            letter, value = tok[0].upper(), tok[1:]

            if "." in value:
                raise ValueError("line %d: decimals are rejected by the "
                                 "parser (%r)" % (n, tok))

            v = int(value)
            if letter == "X":
                dx = v
            elif letter == "Y":
                dy = v
            elif letter == "F":
                if v <= 0:
                    raise ValueError("line %d: F must be positive" % n)
                have_f = True
            else:
                raise ValueError("line %d: bad token %r" % (n, tok))

        if dx is None or dy is None:
            raise ValueError("line %d: G1 needs both X and Y" % n)
        if not have_f:
            raise ValueError("line %d: F missing on the first move" % n)

        x, y = x + dx, y + dy

        if not (0 <= x <= MAX_X and 0 <= y <= MAX_Y):
            raise ValueError("line %d: leaves the workspace at (%d, %d)"
                             % (n, x, y))

        path.append((x, y))
        moves += 1

    return path, moves


# ---------------------------------------------------------------------
# Preview
# ---------------------------------------------------------------------

def render(path, out_png, px_per_mm=5, supersample=3, pen_mm=0.45):
    """Draw the simulated path so overlapping strokes build up tone."""

    s = px_per_mm * supersample
    w, h = int(MAX_X * s), int(MAX_Y * s)

    img = Image.new("L", (w, h), 255)
    d = ImageDraw.Draw(img)

    width = max(1, int(round(pen_mm * s)))

    def xf(p):
        # Flip Y: the machine's Y grows upward, an image's grows down.
        return (p[0] * s, (MAX_Y - p[1]) * s)

    for a, b in zip(path, path[1:]):
        d.line([xf(a), xf(b)], fill=0, width=width)
        d.ellipse([xf(a)[0] - width / 2, xf(a)[1] - width / 2,
                   xf(a)[0] + width / 2, xf(a)[1] + width / 2], fill=0)

    img = img.resize((int(MAX_X * px_per_mm), int(MAX_Y * px_per_mm)),
                     Image.LANCZOS)
    img.save(out_png)

    return img


# ---------------------------------------------------------------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    gcode_path = os.path.join(root, "gcode", "hatch_sphere.gcode")
    png_path = os.path.join(root, "gcode", "hatch_sphere_preview.png")

    text, pts = to_gcode(build_path())

    with open(gcode_path, "w", newline="\n") as f:
        f.write(text)

    path, moves = simulate(text)
    render(path, png_path)

    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(path, path[1:]))

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    print("moves          : %d" % moves)
    print("path length    : %.0f mm" % length)
    print("x range        : %d .. %d  (limit 0 .. %d)" % (min(xs), max(xs), MAX_X))
    print("y range        : %d .. %d  (limit 0 .. %d)" % (min(ys), max(ys), MAX_Y))
    print("gcode          : %s" % gcode_path)
    print("preview        : %s" % png_path)


if __name__ == "__main__":
    main()
