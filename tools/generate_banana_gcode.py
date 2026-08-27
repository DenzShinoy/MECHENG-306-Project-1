"""
Generate a cross-hatch shaded banana as a single continuous G1 path for
the ME306 CoreXY plotter.

Same rules as the sphere generator: no pen lift, so the whole drawing is
ONE polyline; integer relative X/Y; F clamped to 1200; the pen never
leaves 0..210 x 0..140.

The banana is modelled as a bent cylinder: its centreline is a circular
arc and its half-width tapers toward the tips. Tone is cylinder shading
across the width -- lit from the upper left, so the top edge stays bare
paper and the underside collects the crossing hatch layers -- plus dark
tips and a soft cast shadow underneath to ground it.
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
# The banana
# ---------------------------------------------------------------------
#  Centreline: an arc of the circle centred ABOVE the fruit, so the
#  banana bows downward like a smile. s runs 0..1 from the left tip to
#  the right tip.

ACX, ACY = 105.0, 135.0        # arc centre
ARC_R = 75.0                    # centreline radius
A0, A1 = 215.0, 325.0           # arc angular span, degrees

W_MAX = 12.0                    # half-width at the middle, mm
W_TIP = 2.0                     # half-width at the tips, mm

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


def half_width(s):
    """Half-width of the banana at fraction s along the centreline."""

    return W_TIP + (W_MAX - W_TIP) * math.sin(math.pi * s) ** 0.6


def banana_coords(x, y):
    """(s, q) tube coordinates of a point, or None when outside.

    s is the fraction along the centreline; q = r / w(s) is the offset
    across the width, -1 at the upper edge, +1 at the lower edge.
    """

    dx, dy = x - ACX, y - ACY
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return None

    a = math.degrees(math.atan2(dy, dx)) % 360.0
    if not (A0 <= a <= A1):
        return None

    s = (a - A0) / (A1 - A0)

    r = d - ARC_R
    w = half_width(s)
    if abs(r) > w:
        return None

    return s, r / w


def centre_point(s, r=0.0):
    """Absolute position at fraction s, radial offset r off the spine."""

    a = math.radians(A0 + (A1 - A0) * s)
    return (ACX + (ARC_R + r) * math.cos(a),
            ACY + (ARC_R + r) * math.sin(a))


def tone(x, y):
    """Darkness at (x, y), 0 = bare paper, 1 = solid."""

    tube = banana_coords(x, y)

    if tube is not None:
        s, q = tube

        # Cylinder shading: project onto the tube surface. The radial
        # unit vector points down-and-outward (away from the arc centre
        # above), so positive q is the underside.
        dx, dy = x - ACX, y - ACY
        d = math.hypot(dx, dy)
        ux, uy = dx / d, dy / d

        nz = math.sqrt(max(0.0, 1.0 - q * q))
        diffuse = max(0.0, q * ux * LX + q * uy * LY + nz * LZ)

        lit = 0.08 + 0.92 * (diffuse ** 0.85)

        dark = 1.0 - lit

        # Ripe dark tips.
        dark += 0.9 * _smoothstep(0.05, 0.0, s)
        dark += 0.9 * _smoothstep(0.95, 1.0, s)

        return max(0.0, min(1.0, dark))

    # Nothing but the banana marks the paper.
    return 0.0


# ---------------------------------------------------------------------
# Hatching
# ---------------------------------------------------------------------

LAYERS = [
    # (angle degrees, spacing mm, darkness threshold)
    (45,  3.5, 0.22),
    (135, 3.5, 0.45),
    (0,   3.0, 0.62),
    (90,  3.0, 0.78),
]

STEP = 0.5          # sampling step along a hatch line, mm
MIN_SPAN = 2.0      # discard spans shorter than this, mm

# Hatch field bounds: everything the tone function can mark.
HX0, HX1 = 20.0, 190.0
HY0, HY1 = 25.0, 110.0


def hatch_layer(angle_deg, spacing, threshold):
    """Spans of one layer, as records (line, t0, t1, p0, p1)."""

    a = math.radians(angle_deg)
    ux, uy = math.cos(a), math.sin(a)        # along the hatch line
    vx, vy = -uy, ux                          # across the family

    cx, cy = (HX0 + HX1) / 2.0, (HY0 + HY1) / 2.0
    half = math.hypot(HX1 - HX0, HY1 - HY0) / 2.0

    spans = []
    n = int(half / spacing) + 1

    for i in range(-n, n + 1):
        ox, oy = cx + vx * spacing * i, cy + vy * spacing * i

        run_t = None

        def point(t):
            return (ox + ux * t, oy + uy * t)

        def close(t_end):
            if run_t is not None and t_end - run_t >= MIN_SPAN:
                spans.append((i, run_t, t_end, point(run_t), point(t_end)))

        t = -half
        while t <= half:
            px, py = point(t)
            inside = tone(px, py) >= threshold

            if inside and run_t is None:
                run_t = t
            elif not inside and run_t is not None:
                close(t - STEP)
                run_t = None

            t += STEP

        close(half)

    return spans


def group_regions(spans):
    """Split a layer's spans into connected blobs of tone.

    With the pen permanently down, the route between two spans is drawn.
    Grouping into blobs first means each connector only reaches the
    neighbouring hatch line inside the same blob, so it lands on the
    blob's own edge and reads as a contour rather than a stray slash.
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
    """One blob as oriented span pairs, serpentining line by line."""

    out = []
    flip = False
    current_line = None

    for line, _, _, p0, p1 in region:
        if current_line is not None and line != current_line:
            flip = not flip
        current_line = line

        if flip:
            p0, p1 = p1, p0

        out.append((p0, p1))

    return out


def chain_layer(spans, start):
    """A layer as one ordered list of oriented spans, nearest-first."""

    regions = [r for r in group_regions(spans) if len(r) >= 2]
    chains = [chain_region(r) for r in regions]

    ordered = []
    here = start

    while chains:
        best, rev, best_d = None, False, None

        for i, c in enumerate(chains):
            for r in (False, True):
                head = c[-1][1] if r else c[0][0]
                d = math.hypot(head[0] - here[0], head[1] - here[1])
                if best_d is None or d < best_d:
                    best_d, best, rev = d, i, r

        c = chains.pop(best)
        if rev:
            c = [(b, a) for a, b in reversed(c)]

        ordered += c
        here = ordered[-1][1]

    return ordered


# ---------------------------------------------------------------------
# Routing jumps along the outline
# ---------------------------------------------------------------------
#  Retracing a line that is already inked costs machine time but adds no
#  new mark. The outline is drawn first and encircles every hatch region,
#  so a long jump between regions is taken by hopping to the nearest
#  outline point, riding the outline around, and hopping off near the
#  destination -- instead of slashing straight across the bare lit side.

JUMP_LIMIT = 10.0   # mm; a direct connector this short hides in the hatch

# Tone below this is bare paper: the first hatch layer's threshold. A
# connector through anything darker lands on ink and disappears; only
# the bare millimetres of a connector are visible.
BARE = 0.22


def _bare_length(a, b):
    """Millimetres of bare paper a straight stroke a->b would cross."""

    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(length))

    bare = 0.0
    for k in range(n + 1):
        t = k / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if tone(x, y) < BARE:
            bare += length / (n + 1)

    return bare


def _nearest_vertex(loop, p):
    """(index, distance) of the loop vertex closest to p."""

    best_i, best_d = 0, None
    for i, q in enumerate(loop):
        d = math.hypot(p[0] - q[0], p[1] - q[1])
        if best_d is None or d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


def connect(loop, here, target):
    """Points that carry the pen from `here` to `target`.

    The choice between going straight and riding the outline is made on
    VISIBLE ink -- the bare-paper millimetres each option would cross --
    not on distance. A long connector that stays inside the hatched dark
    side is free; a short one across the highlight is the one thing this
    drawing cannot absorb.
    """

    direct = math.hypot(target[0] - here[0], target[1] - here[1])
    if direct <= JUMP_LIMIT:
        return []

    direct_bare = _bare_length(here, target)
    if direct_bare <= 3.0:
        return []                      # the hatch swallows it

    ia, _ = _nearest_vertex(loop, here)
    ib, _ = _nearest_vertex(loop, target)

    detour_bare = (_bare_length(here, loop[ia]) +
                   _bare_length(loop[ib], target))

    if detour_bare >= direct_bare:
        return []

    # The outline is a closed loop: walk whichever way round is shorter.
    n = len(loop)
    fwd = (ib - ia) % n
    back = (ia - ib) % n

    if fwd <= back:
        idxs = [(ia + k) % n for k in range(fwd + 1)]
    else:
        idxs = [(ia - k) % n for k in range(back + 1)]

    return [loop[i] for i in idxs]


# ---------------------------------------------------------------------
# The whole drawing
# ---------------------------------------------------------------------

def outline(segments=48):
    """Closed outline: left tip, along the top edge, around the right
    tip, back along the underside."""

    pts = []

    for i in range(segments + 1):
        s = i / segments
        pts.append(centre_point(s, -half_width(s)))

    for i in range(segments, -1, -1):
        s = i / segments
        pts.append(centre_point(s, half_width(s)))

    return pts


def build_path():
    """The drawing as one list of absolute (x, y) points in mm."""

    # Lead in from the homed origin at the left tip's height, so the one
    # unavoidable entry stroke arrives at the tip instead of slashing
    # across the page.
    tip = centre_point(0.0, -half_width(0.0))

    pts = [(0, 0), (0, tip[1]), tip]

    # The outline gives the shading an edge to belong to, and doubles as
    # the road network long connectors ride around the lit side.
    loop = outline()
    pts += loop

    # Every span gets its own routed approach: short hops go direct and
    # vanish into the hatch, long ones ride the outline. This covers the
    # within-region case too -- two spans on the same hatch line separated
    # by a lit gap would otherwise be joined by a straight ride along the
    # hatch direction, right across the highlight.
    for angle, spacing, threshold in LAYERS:
        for p0, p1 in chain_layer(hatch_layer(angle, spacing, threshold),
                                  pts[-1]):
            pts += connect(loop, pts[-1], p0)
            pts += [p0, p1]

    return pts


# ---------------------------------------------------------------------
# G-code emission
# ---------------------------------------------------------------------

def to_gcode(points):
    """Integer, relative G1 moves, starting from the post-G28 origin."""

    ipts = [(int(round(x)), int(round(y))) for x, y in points]

    dedup = [ipts[0]]
    for p in ipts[1:]:
        if p != dedup[-1]:
            dedup.append(p)

    lines = [
        "; ME306 CoreXY plotter -- cross-hatch shaded banana",
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
    """Replay the G-code under the firmware's rules; return the path."""

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

    img = img.resize((int(MAX_X * px_per_mm), int(MAX_Y * px_per_mm)),
                     Image.LANCZOS)
    img.save(out_png)

    return img


# ---------------------------------------------------------------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    gcode_path = os.path.join(root, "gcode", "banana.gcode")
    png_path = os.path.join(root, "gcode", "banana_preview.png")

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
