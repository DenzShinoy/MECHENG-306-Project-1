"""
Cross-hatch a photograph into a single continuous G1 path for the ME306
CoreXY plotter.

Same machine constraints as the sphere generator: no pen lift, so the
whole drawing is ONE polyline; integer relative X/Y; F clamped to 1200;
the pen may never leave 0..210 x 0..140.

Tone here is sampled from an image instead of computed analytically.
Five hatch layers at five angles are each kept where the photo is darker
than the layer's threshold, so highlights stay bare paper and the
darkest areas (hair, beard) collect every layer crossing.
"""

import math
import os

from PIL import Image, ImageDraw, ImageFilter, ImageOps

# ---------------------------------------------------------------------
# Machine limits (mirror of Manager::max_x / max_y and cfg::MAX_FEED)
# ---------------------------------------------------------------------

MAX_X = 210
MAX_Y = 140
FEED = 1200

# ---------------------------------------------------------------------
# Image placement
# ---------------------------------------------------------------------
#  The photo is square, the workspace is not: use the full height minus
#  a margin and centre horizontally.

MARGIN = 10
SIZE = MAX_Y - 2 * MARGIN                 # 120 mm square

X0 = (MAX_X - SIZE) // 2                  # 45
X1 = X0 + SIZE                            # 165
Y0 = MARGIN                               # 10
Y1 = Y0 + SIZE                            # 130

# ---------------------------------------------------------------------
# Tone from the photograph
# ---------------------------------------------------------------------

_gray = None


def load_image(path):
    global _gray

    im = Image.open(path).convert("L")

    # Kill the photo's noise before thresholding turns it into speckle,
    # then stretch the histogram so the tones span the full hatch range.
    im = im.filter(ImageFilter.GaussianBlur(radius=1.2))
    im = ImageOps.autocontrast(im, cutoff=2)

    _gray = im


def tone(x, y):
    """Darkness of the photo at plot position (x, y), 0..1."""

    if not (X0 <= x <= X1 and Y0 <= y <= Y1):
        return 0.0

    w, h = _gray.size

    # Map plot mm to image pixels. Machine Y grows upward, image Y grows
    # downward, so the vertical axis flips.
    fx = (x - X0) / SIZE * (w - 1)
    fy = (1.0 - (y - Y0) / SIZE) * (h - 1)

    ix, iy = int(fx), int(fy)
    ix2, iy2 = min(ix + 1, w - 1), min(iy + 1, h - 1)
    tx, ty = fx - ix, fy - iy

    p = _gray.load()
    v = (p[ix, iy] * (1 - tx) * (1 - ty) + p[ix2, iy] * tx * (1 - ty) +
         p[ix, iy2] * (1 - tx) * ty + p[ix2, iy2] * tx * ty)

    return 1.0 - v / 255.0


def in_frame(x, y):
    return X0 <= x <= X1 and Y0 <= y <= Y1


# ---------------------------------------------------------------------
# Hatching
# ---------------------------------------------------------------------

LAYERS = [
    # (angle degrees, spacing mm, darkness threshold)
    (45,  2.6, 0.18),
    (135, 2.6, 0.38),
    (0,   2.2, 0.55),
    (90,  2.2, 0.70),
    (45,  1.8, 0.84),
]

STEP = 0.5          # sampling step along a hatch line, mm
MIN_SPAN = 1.5      # discard spans shorter than this, mm


def hatch_layer(angle_deg, spacing, threshold):
    """Spans of one layer, as records (line, t0, t1, p0, p1)."""

    a = math.radians(angle_deg)
    ux, uy = math.cos(a), math.sin(a)        # along the hatch line
    vx, vy = -uy, ux                          # across the family

    cx, cy = (X0 + X1) / 2.0, (Y0 + Y1) / 2.0
    half = math.hypot(X1 - X0, Y1 - Y0) / 2.0

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
            inside = in_frame(px, py) and tone(px, py) >= threshold

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

def build_path():
    """The drawing as one list of absolute (x, y) points in mm."""

    # Lead in from the homed origin along the paper edge, then draw the
    # frame around the portrait. The frame squares the picture and gives
    # the hatching a neutral place to start.
    pts = [(0, 0), (X0, 0), (X0, Y0)]
    pts += [(X1, Y0), (X1, Y1), (X0, Y1), (X0, Y0)]

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

    dedup = [ipts[0]]
    for p in ipts[1:]:
        if p != dedup[-1]:
            dedup.append(p)

    lines = [
        "; ME306 CoreXY plotter -- cross-hatched portrait",
        "; Single continuous stroke: no pen lift, every move draws.",
        "; Tone sampled from a photo; five hatch layers at five angles.",
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

def render(path, out_png, px_per_mm=6, supersample=3, pen_mm=0.4):
    """Draw the simulated path so overlapping strokes build up tone."""

    s = px_per_mm * supersample
    w, h = int(MAX_X * s), int(MAX_Y * s)

    img = Image.new("L", (w, h), 255)
    d = ImageDraw.Draw(img)

    width = max(1, int(round(pen_mm * s)))

    def xf(p):
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

    src = os.path.join(os.path.dirname(root), "thumbnail.png")
    gcode_path = os.path.join(root, "gcode", "portrait.gcode")
    png_path = os.path.join(root, "gcode", "portrait_preview.png")

    load_image(src)

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
