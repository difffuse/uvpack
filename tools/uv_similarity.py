#!/usr/bin/env python3
"""Matching UV islands, stacking them, spreading them apart and measuring them.

Standard library only, like uv_islands.py, so the same code runs inside Blender's
bundled interpreter (which has no numpy and no shapely) and standalone next to the
OBJ pipeline.

The island dictionaries are the ones uv_islands.extract_islands produces:

    {"id": int, "outer": [[u, v], ...], "holes": [[[u, v], ...], ...],
     "parts": [{"outer": [...], "holes": [...]}, ...],   # optional
     "faces": [face index, ...], "area_3d": float}       # optional

Two optional keys are read by this module and by nothing else:

    "verts":     the island's unique UV vertices, including the interior ones. The
                 rings only carry boundary vertices, so vertex-based modes and
                 "correct vertices" need this to see the whole island.
    "face_uvs":  the island's faces as lists of (u, v), needed by TOPOLOGY mode.

What each mode decides and how exact the transforms it hands back are is written
down in README.md, section "UV tools".
"""
import math
from collections import defaultdict

GRID = 1e-7  # UV quantization for vertex identity, the same grid uv_islands.py uses

# How two islands are compared. OUTLINE looks at the boundary alone, VERTICES
# additionally requires the two vertex sets to coincide, TOPOLOGY walks the faces.
OUTLINE = "outline"
VERTICES = "vertices"
TOPOLOGY = "topology"
MODES = (OUTLINE, VERTICES, TOPOLOGY)

# How an overlap is decided: TOUCHING counts any contact, IDENTICAL only two
# islands whose outlines coincide (a deliberate stack).
TOUCHING = "touching"
IDENTICAL = "identical"


# ---------------------------------------------------------------- affine transforms
class Transform:
    """A 2D affine map in uvpack_cli's own convention:

        (x, y) -> (a x + b y + c, d x + e y + f)

    which is what the CLI's nine-element `transform` carries, so a Transform can be
    handed straight to the packer or composed with what the packer sends back.
    """
    __slots__ = ("a", "b", "c", "d", "e", "f")

    def __init__(self, a=1.0, b=0.0, c=0.0, d=0.0, e=1.0, f=0.0):
        self.a, self.b, self.c = float(a), float(b), float(c)
        self.d, self.e, self.f = float(d), float(e), float(f)

    # -- construction
    @staticmethod
    def identity():
        return Transform()

    @staticmethod
    def translation(dx, dy):
        return Transform(1.0, 0.0, dx, 0.0, 1.0, dy)

    @staticmethod
    def rotation(angle, pivot=(0.0, 0.0)):
        return Transform.similarity(angle=angle, pivot=pivot)

    @staticmethod
    def similarity(angle=0.0, scale=1.0, flip=False, translation=(0.0, 0.0), pivot=(0.0, 0.0),
                   scale_y=None):
        """rotate(angle) . mirror-in-u (optional) . scale, about `pivot`, then translate.

        `scale_y` gives the non-uniform case: `scale` is then the u scale."""
        sx = float(scale)
        sy = float(scale if scale_y is None else scale_y)
        c, s = math.cos(angle), math.sin(angle)
        mx = -1.0 if flip else 1.0
        # linear = R(angle) . diag(mx, 1) . diag(sx, sy)
        a, b = c * mx * sx, -s * sy
        d, e = s * mx * sx, c * sy
        px, py = pivot
        tx = translation[0] + px - (a * px + b * py)
        ty = translation[1] + py - (d * px + e * py)
        return Transform(a, b, tx, d, e, ty)

    @staticmethod
    def from_cli(values):
        """A uvpack_cli `transform` (nine numbers, row major, last row 0 0 1)."""
        return Transform(values[0], values[1], values[2], values[3], values[4], values[5])

    def to_cli(self):
        return [self.a, self.b, self.c, self.d, self.e, self.f, 0.0, 0.0, 1.0]

    # -- use
    def apply(self, point):
        x, y = point
        return (self.a * x + self.b * y + self.c, self.d * x + self.e * y + self.f)

    def apply_ring(self, ring):
        return [list(self.apply(p)) for p in ring]

    def compose(self, other):
        """self after other: compose(other).apply(p) == self.apply(other.apply(p))."""
        return Transform(
            self.a * other.a + self.b * other.d,
            self.a * other.b + self.b * other.e,
            self.a * other.c + self.b * other.f + self.c,
            self.d * other.a + self.e * other.d,
            self.d * other.b + self.e * other.e,
            self.d * other.c + self.e * other.f + self.f)

    def inverse(self):
        det = self.determinant()
        if abs(det) < 1e-30:
            raise ValueError("singular transform")
        ia, ib = self.e / det, -self.b / det
        id_, ie = -self.d / det, self.a / det
        return Transform(ia, ib, -(ia * self.c + ib * self.f),
                         id_, ie, -(id_ * self.c + ie * self.f))

    # -- inspection
    def determinant(self):
        return self.a * self.e - self.b * self.d

    def mirrored(self):
        return self.determinant() < 0.0

    def scale(self):
        return math.sqrt(abs(self.determinant()))

    def angle(self):
        return math.atan2(self.d, self.a) if not self.mirrored() else math.atan2(-self.d, -self.a)

    def is_close(self, other, tol=1e-9):
        return all(abs(getattr(self, k) - getattr(other, k)) <= tol for k in self.__slots__)

    def __repr__(self):
        return ("Transform(angle={:.4f} scale={:.5f}{} t=({:.5f}, {:.5f}))"
                .format(self.angle(), self.scale(), " mirrored" if self.mirrored() else "",
                        self.c, self.f))


# ---------------------------------------------------------------- rings and islands
def ring_area(ring):
    a = 0.0
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return 0.5 * a


def ring_perimeter(ring):
    total = 0.0
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def island_parts(island):
    """(outer, holes) for the main polygon and for every extra part."""
    return ([(island["outer"], island.get("holes", []))] +
            [(p["outer"], p.get("holes", [])) for p in island.get("parts", [])])


def island_rings(island):
    return [r for outer, holes in island_parts(island) for r in [outer] + list(holes)]


def outer_ring(island):
    """The largest outer ring: the one an outline signature is built from."""
    outers = [o for o, _ in island_parts(island)]
    return max(outers, key=lambda r: abs(ring_area(r)))


def extra_rings(island):
    """Everything the outline signature leaves out: holes and the other parts.

    `check_holes` folds these into the comparison; without it only the ring above
    decides, which is what "check holes" off means."""
    main = outer_ring(island)
    return [r for r in island_rings(island) if r is not main]


def island_area(island):
    """Outer-ring area minus the holes, summed over the parts, always positive."""
    return sum(abs(ring_area(o)) - sum(abs(ring_area(h)) for h in holes)
               for o, holes in island_parts(island))


def island_bbox(island):
    xs = [p[0] for r in island_rings(island) for p in r]
    ys = [p[1] for r in island_rings(island) for p in r]
    return (min(xs), min(ys), max(xs), max(ys))


def island_vertices(island):
    """Unique UV vertices. "verts" when the caller supplied the interior vertices too,
    otherwise the boundary vertices, which is all the rings know about."""
    source = island.get("verts")
    if source is None:
        source = [p for r in island_rings(island) for p in r]
    seen, out = set(), []
    for p in source:
        key = (round(p[0] / GRID), round(p[1] / GRID))
        if key in seen:
            continue
        seen.add(key)
        out.append((float(p[0]), float(p[1])))
    return out


def island_size(island):
    """Characteristic length used to normalize residuals: the side of a square with
    the island's area, falling back to a quarter of the outline when the area is 0."""
    area = island_area(island)
    if area > 1e-18:
        return math.sqrt(area)
    perimeter = ring_perimeter(outer_ring(island))
    return max(perimeter / 4.0, 1e-12)


def transform_island(island, transform):
    """A copy of the island with every ring (and "verts", and "face_uvs") mapped."""
    out = dict(island)
    out["outer"] = transform.apply_ring(island["outer"])
    out["holes"] = [transform.apply_ring(h) for h in island.get("holes", [])]
    if island.get("parts"):
        out["parts"] = [{"outer": transform.apply_ring(p["outer"]),
                         "holes": [transform.apply_ring(h) for h in p.get("holes", [])]}
                        for p in island["parts"]]
    if island.get("verts") is not None:
        out["verts"] = [list(transform.apply(p)) for p in island["verts"]]
    if island.get("face_uvs") is not None:
        out["face_uvs"] = [transform.apply_ring(f) for f in island["face_uvs"]]
    return out


def translate_island(island, dx, dy):
    return transform_island(island, Transform.translation(dx, dy))


def attach_face_uvs(island, faces_uv):
    """Give an island from extract_islands the UV loops of its faces, which TOPOLOGY
    mode needs. `faces_uv` is the same list that was handed to extract_islands."""
    island = dict(island)
    island["face_uvs"] = [[tuple(uv) for uv in faces_uv[f]] for f in island["faces"]]
    return island


def attach_verts(island, faces_uv):
    """Give an island its full vertex set (interior vertices included)."""
    island = dict(island)
    island["verts"] = [list(uv) for f in island["faces"] for uv in faces_uv[f]]
    return island


# ---------------------------------------------------------------- overlap, ported from
# src/validator.cpp part_distance(): a part is an outer ring plus its holes; two parts
# are at distance 0 when either contains the other's first vertex or any two of their
# edges meet. Islands overlap when any of their parts do ("any part" detection).
def _point_in_ring(pt, ring):
    x, y = pt
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            if x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                inside = not inside
    return inside


def _contains_part(part, pt):
    outer, holes = part
    return _point_in_ring(pt, outer) and not any(_point_in_ring(pt, h) for h in holes)


def _point_segment_distance(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segment_distance(p1, p2, p3, p4):
    d1, d2 = _orient(p3, p4, p1), _orient(p3, p4, p2)
    d3, d4 = _orient(p1, p2, p3), _orient(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(_point_segment_distance(p1, p3, p4), _point_segment_distance(p2, p3, p4),
               _point_segment_distance(p3, p1, p2), _point_segment_distance(p4, p1, p2))


def part_distance(part_a, part_b, gap_needed=0.0):
    if _contains_part(part_a, part_b[0][0]) or _contains_part(part_b, part_a[0][0]):
        return 0.0
    best = float("inf")
    rings_a = [part_a[0]] + list(part_a[1])
    rings_b = [part_b[0]] + list(part_b[1])
    for ra in rings_a:
        n = len(ra)
        for i in range(n):
            a, b = ra[i], ra[(i + 1) % n]
            ax0, ax1 = min(a[0], b[0]) - gap_needed, max(a[0], b[0]) + gap_needed
            ay0, ay1 = min(a[1], b[1]) - gap_needed, max(a[1], b[1]) + gap_needed
            for rb in rings_b:
                m = len(rb)
                for j in range(m):
                    c, d = rb[j], rb[(j + 1) % m]
                    if (max(c[0], d[0]) < ax0 or min(c[0], d[0]) > ax1 or
                            max(c[1], d[1]) < ay0 or min(c[1], d[1]) > ay1):
                        continue
                    dist = segment_distance(a, b, c, d)
                    if dist < best:
                        best = dist
                        if best == 0.0:
                            return 0.0
    return best


def island_distance(island_a, island_b, gap_needed=0.0):
    best = float("inf")
    for pa in island_parts(island_a):
        for pb in island_parts(island_b):
            best = min(best, part_distance(pa, pb, gap_needed))
            if best == 0.0:
                return 0.0
    return best


def _boxes_apart(a, b, gap=0.0):
    return (a[2] + gap < b[0] or b[2] + gap < a[0] or
            a[3] + gap < b[1] or b[3] + gap < a[1])


def islands_overlap(island_a, island_b, detection_mode=TOUCHING, tolerance=1e-9):
    """TOUCHING: the islands touch or overlap anywhere.
    IDENTICAL: their outlines coincide, which is what uvpack_cli's `bind_overlapping:
    identical` merges -- islands deliberately stacked on top of each other."""
    if _boxes_apart(island_bbox(island_a), island_bbox(island_b)):
        return False
    if detection_mode == IDENTICAL:
        return _islands_coincide(island_a, island_b, tolerance)
    return island_distance(island_a, island_b) == 0.0


def _islands_coincide(island_a, island_b, tolerance=1e-6):
    va, vb = island_vertices(island_a), island_vertices(island_b)
    if len(va) != len(vb):
        return False
    remaining = list(vb)
    for p in va:
        hit = None
        for i, q in enumerate(remaining):
            if math.hypot(p[0] - q[0], p[1] - q[1]) <= tolerance:
                hit = i
                break
        if hit is None:
            return False
        remaining.pop(hit)
    return True


# ---------------------------------------------------------------- Procrustes / Kabsch
def fit_similarity(src, dst, allow_flip=False, adjust_scale=False,
                   non_uniform_scaling_tolerance=0.0):
    """The least-squares similarity transform taking the points `src` onto `dst`
    (Kabsch in 2D, closed form), and the RMS residual it leaves.

    adjust_scale off means the fit is rigid (rotation, optional reflection, translation);
    on, a uniform scale is fitted too. non_uniform_scaling_tolerance in (0, 1] additionally
    allows the two axis scales to differ by that fraction.
    """
    n = len(src)
    if n == 0 or n != len(dst):
        return None, float("inf")
    cx = sum(p[0] for p in src) / n
    cy = sum(p[1] for p in src) / n
    qx = sum(p[0] for p in dst) / n
    qy = sum(p[1] for p in dst) / n
    sxx = sxy = syx = syy = 0.0
    src_norm = dst_norm = 0.0
    for (px, py), (dx_, dy_) in zip(src, dst):
        px, py = px - cx, py - cy
        dx_, dy_ = dx_ - qx, dy_ - qy
        sxx += px * dx_
        sxy += px * dy_
        syx += py * dx_
        syy += py * dy_
        src_norm += px * px + py * py
        dst_norm += dx_ * dx_ + dy_ * dy_
    if src_norm <= 1e-30:
        return None, float("inf")

    best = (None, float("inf"))
    signs = (1.0, -1.0) if allow_flip else (1.0,)
    for mx in signs:
        # Mirroring in u first turns (px, py) into (mx*px, py), which flips the signs
        # of the two sums that carry px.
        a11, a12, a21, a22 = mx * sxx, mx * sxy, syx, syy
        angle = math.atan2(a12 - a21, a11 + a22)
        c, s = math.cos(angle), math.sin(angle)
        # trace of R^T . S, the quantity the rotation maximizes
        dot = c * (a11 + a22) + s * (a12 - a21)
        scale = (dot / src_norm) if adjust_scale else 1.0
        if adjust_scale and scale <= 0.0:
            continue
        sx = sy = scale
        if non_uniform_scaling_tolerance > 0.0:
            sx, sy = _axis_scales(src, dst, (cx, cy), (qx, qy), angle, mx, scale,
                                  non_uniform_scaling_tolerance, adjust_scale)
        transform = Transform.similarity(angle=angle, scale=sx, scale_y=sy, flip=(mx < 0))
        # translation so the centroids coincide
        mapped = transform.apply((cx, cy))
        transform = Transform(transform.a, transform.b, qx - mapped[0],
                              transform.d, transform.e, qy - mapped[1])
        residual = rms_residual(src, dst, transform)
        if residual < best[1]:
            best = (transform, residual)
    return best


def _axis_scales(src, dst, src_c, dst_c, angle, mx, uniform, tolerance, adjust_scale):
    """One refinement step for the anisotropic case. The model is R . diag(mx, 1) . D,
    so with the rotation fixed the two entries of D are independent least squares
    against the destination points rotated back by R, and the tolerance caps how far
    their ratio may stray from 1."""
    c, s = math.cos(angle), math.sin(angle)
    num_x = den_x = num_y = den_y = 0.0
    for (px, py), (dx_, dy_) in zip(src, dst):
        px, py = px - src_c[0], py - src_c[1]
        qx, qy = dx_ - dst_c[0], dy_ - dst_c[1]
        tx, ty = c * qx + s * qy, -s * qx + c * qy   # R^T q
        num_x += (mx * px) * tx
        den_x += px * px
        num_y += py * ty
        den_y += py * py
    sx = num_x / den_x if den_x > 1e-30 else uniform
    sy = num_y / den_y if den_y > 1e-30 else uniform
    if sx <= 0.0 or sy <= 0.0:
        return uniform, uniform
    mean = math.sqrt(sx * sy)
    if not adjust_scale:
        mean = 1.0
    ratio = sx / sy
    ratio = min(max(ratio, 1.0 / (1.0 + tolerance)), 1.0 + tolerance)
    root = math.sqrt(ratio)
    return mean * root, mean / root


def rms_residual(src, dst, transform):
    total = 0.0
    for p, q in zip(src, dst):
        mx, my = transform.apply(p)
        total += (mx - q[0]) ** 2 + (my - q[1]) ** 2
    return math.sqrt(total / len(src))


def max_residual(src, dst, transform):
    worst = 0.0
    for p, q in zip(src, dst):
        mx, my = transform.apply(p)
        worst = max(worst, math.hypot(mx - q[0], my - q[1]))
    return worst


# ---------------------------------------------------------------- outline signatures
def resample_ring(ring, count, offset=0.0, reverse=False):
    """`count` points spaced equally along the ring by arc length, starting `offset`
    (a fraction of the perimeter) after the first vertex. Cyclic shift and reflection
    invariance come from sweeping `offset` and from `reverse`."""
    pts = list(ring[::-1]) if reverse else list(ring)
    n = len(pts)
    lengths = []
    total = 0.0
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        seg = math.hypot(x1 - x0, y1 - y0)
        lengths.append(seg)
        total += seg
    if total <= 0.0:
        return [tuple(pts[0]) for _ in range(count)]
    out = []
    start = (offset % 1.0) * total
    edge, walked = 0, 0.0
    for k in range(count):
        target = start + total * k / count
        if target >= total:
            target -= total
        while walked + lengths[edge] < target:
            walked += lengths[edge]
            edge = (edge + 1) % n
            if edge == 0:
                walked = 0.0
        t = (target - walked) / lengths[edge] if lengths[edge] > 0 else 0.0
        x0, y0 = pts[edge]
        x1, y1 = pts[(edge + 1) % n]
        out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return out


def _centroid(points):
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def radial_profile(points):
    """Distance from the centroid at every sample, in units of the mean such distance:
    a descriptor invariant under rotation, translation and scale, used only to rank
    candidate cyclic shifts before the exact fits run."""
    cx, cy = _centroid(points)
    radii = [math.hypot(p[0] - cx, p[1] - cy) for p in points]
    mean = sum(radii) / len(radii)
    if mean <= 1e-18:
        return radii
    return [r / mean for r in radii]


def _shift_costs(profile_a, profile_b, limit):
    """The `limit` cyclic shifts of B that line its radial profile up with A's best."""
    n = len(profile_a)
    step = max(1, n // 256)  # a long ring is ranked on a subsample; the fit is exact anyway
    idx = range(0, n, step)
    scored = []
    for k in range(n):
        cost = 0.0
        for i in idx:
            cost += abs(profile_a[i] - profile_b[(i + k) % n])
        scored.append((cost, k))
    scored.sort()
    return [k for _, k in scored[:limit]]


class _RingFit:
    """The outcome of fitting one ring onto another."""
    __slots__ = ("transform", "residual")

    def __init__(self, transform, residual):
        self.transform = transform
        self.residual = residual


def _fit_by_vertex_correspondence(ring_src, ring_dst, allow_flip, adjust_scale, nu_tol,
                                  candidates=8):
    """Exact path: two rings with the same vertex count that are copies of each other
    correspond under a cyclic shift (plus a reversal when one is mirrored), so fitting
    the actual vertices lands on the transform to machine resolution. Returns every fit
    it tried, best first, because a symmetric outline has several equally good ones and
    the vertex-based modes need to test more than the first."""
    n = len(ring_src)
    if n != len(ring_dst) or n < 2:
        return []
    prof_dst = radial_profile(ring_dst)
    fits = []
    for reverse in ((False, True) if allow_flip else (False,)):
        src = list(ring_src[::-1]) if reverse else list(ring_src)
        prof_src = radial_profile(src)
        for k in _shift_costs(prof_dst, prof_src, candidates):
            pairs_src = [src[(i + k) % n] for i in range(n)]
            transform, residual = fit_similarity(pairs_src, ring_dst, allow_flip=allow_flip,
                                                 adjust_scale=adjust_scale,
                                                 non_uniform_scaling_tolerance=nu_tol)
            if transform is None:
                continue
            fits.append(_RingFit(transform, residual))
    fits.sort(key=lambda f: f.residual)
    return fits


def _fit_by_resampling(ring_src, ring_dst, allow_flip, adjust_scale, nu_tol, resolution):
    """General path: the two rings are sampled at the same number of equally spaced
    points and the best cyclic offset is found first on the sample grid, then refined
    continuously (a golden-section search on the offset, since the residual is smooth
    in it). This is what makes a match possible between two tessellations of the same
    shape, at the cost of the sampling error the refinement then removes."""
    count = max(16, min(int(resolution), 1024))
    dst = resample_ring(ring_dst, count)
    prof_dst = radial_profile(dst)
    best, best_offset, best_reverse = None, 0.0, False
    for reverse in ((False, True) if allow_flip else (False,)):
        src = resample_ring(ring_src, count, reverse=reverse)
        prof_src = radial_profile(src)
        for k in _shift_costs(prof_dst, prof_src, 6):
            offset = k / float(count)
            transform, residual = fit_similarity(
                resample_ring(ring_src, count, offset=offset, reverse=reverse), dst,
                allow_flip=allow_flip, adjust_scale=adjust_scale,
                non_uniform_scaling_tolerance=nu_tol)
            if transform is None:
                continue
            if best is None or residual < best.residual:
                best, best_offset, best_reverse = _RingFit(transform, residual), offset, reverse

    if best is None:
        return None

    def at(offset):
        transform, residual = fit_similarity(
            resample_ring(ring_src, count, offset=offset, reverse=best_reverse), dst,
            allow_flip=allow_flip, adjust_scale=adjust_scale,
            non_uniform_scaling_tolerance=nu_tol)
        return _RingFit(transform, residual if transform is not None else float("inf"))

    step = 1.0 / count
    lo, hi = best_offset - step, best_offset + step
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    x1, x2 = hi - phi * (hi - lo), lo + phi * (hi - lo)
    f1, f2 = at(x1), at(x2)
    for _ in range(40):
        if f1.residual <= f2.residual:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - phi * (hi - lo)
            f1 = at(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + phi * (hi - lo)
            f2 = at(x2)
        if hi - lo < 1e-12:
            break
    refined = f1 if f1.residual <= f2.residual else f2
    return refined if refined.residual < best.residual else best


def _extra_ring_residual(island_src, island_dst, transform):
    """How far the holes and extra parts land from their counterparts once the outer
    rings are fitted. Rings are paired by area, largest first; a different count is an
    immediate mismatch, which is what "Check Holes" is for."""
    src = sorted(extra_rings(island_src), key=lambda r: -abs(ring_area(r)))
    dst = sorted(extra_rings(island_dst), key=lambda r: -abs(ring_area(r)))
    if len(src) != len(dst):
        return float("inf")
    if not src:
        return 0.0
    worst = 0.0
    for a, b in zip(src, dst):
        ca = _centroid(transform.apply_ring(a))
        cb = _centroid(b)
        area_a = abs(ring_area(a)) * abs(transform.determinant())
        area_b = abs(ring_area(b))
        worst = max(worst, math.hypot(ca[0] - cb[0], ca[1] - cb[1]))
        # An area mismatch is a shape mismatch even when the centroids agree.
        worst = max(worst, math.sqrt(abs(area_a - area_b)))
    return worst


def outline_fits(island_src, island_dst, params, limit=6):
    """Candidate similarity transforms taking `island_src` onto `island_dst`, best
    first, each with the residual normalized by the destination island's size (so the
    number is comparable across islands and is what the threshold applies to).

    The exact vertex-correspondence path runs first; the resampled path is added when
    that one is missing (different vertex counts) or not already comfortably inside the
    threshold, which is the case that needs a match across two tessellations."""
    ring_src, ring_dst = outer_ring(island_src), outer_ring(island_dst)
    size = island_size(island_dst)
    allow_flip = params.allow_mirroring
    fits = _fit_by_vertex_correspondence(ring_src, ring_dst, allow_flip, params.adjust_scale,
                                         params.non_uniform_scaling_tolerance)
    if not fits or fits[0].residual > params.threshold * size * 0.5:
        sampled = _fit_by_resampling(ring_src, ring_dst, allow_flip, params.adjust_scale,
                                     params.non_uniform_scaling_tolerance, params.resolution)
        if sampled is not None:
            fits.append(sampled)
    if not fits:
        return []
    out = []
    for fit in sorted(fits, key=lambda f: f.residual)[:limit]:
        residual = fit.residual / size
        if params.check_holes:
            # Holes and extra parts have to land on their counterparts too; without
            # hole checking the outer ring decides on its own.
            residual = max(residual, _extra_ring_residual(island_src, island_dst,
                                                          fit.transform) / size)
        out.append(_RingFit(fit.transform, residual))
    out.sort(key=lambda f: f.residual)
    return out


def outline_fit(island_src, island_dst, params):
    """The best of outline_fits(), as (transform, normalized residual)."""
    fits = outline_fits(island_src, island_dst, params, limit=1)
    if not fits:
        return None, float("inf")
    return fits[0].transform, fits[0].residual


# ---------------------------------------------------------------- vertex and topology
def _match_vertex_sets(src, dst, transform, threshold):
    """Greedy bijection between two vertex sets under `transform`, and the largest
    distance it leaves. inf when the sets have different sizes or a vertex has no
    partner within the threshold -- the VERTICES test."""
    if len(src) != len(dst):
        return float("inf"), None
    buckets = defaultdict(list)
    cell = max(threshold, 1e-9)
    for i, q in enumerate(dst):
        buckets[(int(math.floor(q[0] / cell)), int(math.floor(q[1] / cell)))].append(i)
    taken = set()
    pairs = []
    worst = 0.0
    for p in src:
        mx, my = transform.apply(p)
        gx, gy = int(math.floor(mx / cell)), int(math.floor(my / cell))
        best, best_dist = None, float("inf")
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for i in buckets.get((gx + ox, gy + oy), ()):
                    if i in taken:
                        continue
                    dist = math.hypot(mx - dst[i][0], my - dst[i][1])
                    if dist < best_dist:
                        best, best_dist = i, dist
        if best is None or best_dist > threshold:
            return float("inf"), None
        taken.add(best)
        pairs.append((p, dst[best]))
        worst = max(worst, best_dist)
    return worst, pairs


def _face_topology(island):
    """(faces as tuples of vertex ids, vertex positions, undirected edge -> faces).
    Face windings are normalized counter-clockwise so two copies of the same island
    are traversed the same way, whichever winding their loops happened to have."""
    faces_uv = island.get("face_uvs")
    if not faces_uv:
        return None
    ids, positions = {}, []
    faces = []
    for face in faces_uv:
        ring = [tuple(uv) for uv in face]
        if ring_area(ring) < 0:
            ring = ring[::-1]
        corner_ids = []
        for uv in ring:
            key = (round(uv[0] / GRID), round(uv[1] / GRID))
            if key not in ids:
                ids[key] = len(positions)
                positions.append(uv)
            corner_ids.append(ids[key])
        # drop repeated corners, which a degenerate UV face can produce
        cleaned = [v for i, v in enumerate(corner_ids) if v != corner_ids[i - 1]]
        if len(cleaned) >= 3:
            faces.append(tuple(cleaned))
    edges = defaultdict(list)
    for fi, face in enumerate(faces):
        n = len(face)
        for i in range(n):
            a, b = face[i], face[(i + 1) % n]
            edges[(a, b) if a < b else (b, a)].append(fi)
    return faces, positions, edges


def _grow_correspondence(topo_a, topo_b, seed_face_b, seed_rotation, seed_reverse):
    """Breadth-first from one seeded face pair: walk both islands across shared edges at
    the same time and record which vertex of A is which vertex of B. Returns the vertex
    map, or None as soon as the two structures disagree.

    Every face is wound counter-clockwise, so a mirrored copy is traversed backwards:
    `seed_reverse` flips the direction the B side walks its corner lists in, and that
    direction then has to be kept for every neighbour reached from the seed."""
    faces_a, _, edges_a = topo_a
    faces_b, _, edges_b = topo_b
    face_a0 = faces_a[0]
    face_b0 = faces_b[seed_face_b]
    if len(face_a0) != len(face_b0):
        return None
    n = len(face_a0)
    step_b = -1 if seed_reverse else 1
    aligned = [face_b0[(seed_rotation + step_b * i) % n] for i in range(n)]
    vmap, used = {}, {}
    fmap = {0: seed_face_b}
    queue = [(0, seed_face_b, list(face_a0), aligned)]

    def bind(va, vb):
        if va in vmap:
            return vmap[va] == vb
        if vb in used:
            return False
        vmap[va] = vb
        used[vb] = va
        return True

    while queue:
        fa, fb, corners_a, corners_b = queue.pop()
        for va, vb in zip(corners_a, corners_b):
            if not bind(va, vb):
                return None
        m = len(corners_a)
        for i in range(m):
            a0, a1 = corners_a[i], corners_a[(i + 1) % m]
            b0, b1 = corners_b[i], corners_b[(i + 1) % m]
            key_a = (a0, a1) if a0 < a1 else (a1, a0)
            key_b = (b0, b1) if b0 < b1 else (b1, b0)
            next_a = [f for f in edges_a.get(key_a, ()) if f != fa]
            next_b = [f for f in edges_b.get(key_b, ()) if f != fb]
            if len(next_a) != len(next_b):
                return None
            if not next_a:
                continue
            na, nb = next_a[0], next_b[0]
            if na in fmap:
                if fmap[na] != nb:
                    return None
                continue
            face_na, face_nb = faces_a[na], faces_b[nb]
            k = len(face_na)
            if k != len(face_nb):
                return None
            # Line the neighbour up on the shared edge: a counter-clockwise neighbour
            # uses the shared edge in the opposite direction, so it holds (a1, a0) --
            # and (b1, b0) once the B side is walked in its own direction.
            ia = _edge_start(face_na, a1, a0, 1)
            ib = _edge_start(face_nb, b1, b0, step_b)
            if ia is None or ib is None:
                return None
            queue.append((na, nb, [face_na[(ia + j) % k] for j in range(k)],
                          [face_nb[(ib + step_b * j) % k] for j in range(k)]))
            fmap[na] = nb
    if len(fmap) != len(faces_a) or len(fmap) != len(faces_b):
        return None
    return vmap


def _edge_start(face, first, second, step):
    """The corner index i with face[i] == first and face[i + step] == second, or None."""
    k = len(face)
    for i in range(k):
        if face[i] == first and face[(i + step) % k] == second:
            return i
    return None


def topology_correspondence(island_src, island_dst, allow_flip=True):
    """A vertex correspondence between two islands with the same face topology, or
    None. Seeds are the first face of the source against every face of the destination
    with the same corner count, in every rotation (and, with mirroring, reversal)."""
    topo_src = _face_topology(island_src)
    topo_dst = _face_topology(island_dst)
    if topo_src is None or topo_dst is None:
        return None
    faces_src, pos_src, _ = topo_src
    faces_dst, pos_dst, _ = topo_dst
    if not faces_src or len(faces_src) != len(faces_dst) or len(pos_src) != len(pos_dst):
        return None
    if sorted(len(f) for f in faces_src) != sorted(len(f) for f in faces_dst):
        return None
    n = len(faces_src[0])
    for fb, face_b in enumerate(faces_dst):
        if len(face_b) != n:
            continue
        for rotation in range(n):
            for reverse in ((False, True) if allow_flip else (False,)):
                vmap = _grow_correspondence(topo_src, topo_dst, fb, rotation, reverse)
                if vmap is None:
                    continue
                return ([pos_src[a] for a in sorted(vmap)],
                        [pos_dst[vmap[a]] for a in sorted(vmap)])
    return None


# ---------------------------------------------------------------- parameters and matches
class MatchParams:
    """Everything the comparison takes: which mode decides, how far apart two
    islands may be and still match, and how much freedom the fit is given
    (mirroring, a uniform scale, a little anisotropy, vertex correction).
    """

    def __init__(self, mode=OUTLINE, threshold=0.1, resolution=500, check_holes=False,
                 allow_mirroring=False, adjust_scale=False, non_uniform_scaling_tolerance=0.0,
                 correct_vertices=False, vertex_threshold=0.01):
        if mode not in MODES:
            raise ValueError("unknown match mode: {!r}".format(mode))
        self.mode = mode
        self.threshold = float(threshold)
        self.resolution = int(resolution)
        self.check_holes = bool(check_holes)
        self.allow_mirroring = bool(allow_mirroring)
        self.adjust_scale = bool(adjust_scale)
        self.non_uniform_scaling_tolerance = float(non_uniform_scaling_tolerance)
        self.correct_vertices = bool(correct_vertices)
        self.vertex_threshold = float(vertex_threshold)

    def is_vertex_based(self):
        return self.mode in (TOPOLOGY, VERTICES)

    def copy(self, **overrides):
        values = {k: getattr(self, k) for k in
                  ("mode", "threshold", "resolution", "check_holes", "allow_mirroring",
                   "adjust_scale", "non_uniform_scaling_tolerance", "correct_vertices",
                   "vertex_threshold")}
        values.update(overrides)
        return MatchParams(**values)


class Match:
    """One island found to match a reference, with the transform that stacks it."""
    __slots__ = ("island", "index", "id", "reference_id", "transform", "residual", "mode",
                 "pairs")

    def __init__(self, island, index, transform, residual, mode, reference_id=None, pairs=None):
        self.island = island
        self.index = index
        self.id = island.get("id", index)
        self.reference_id = reference_id
        self.transform = transform
        self.residual = residual
        self.mode = mode
        self.pairs = pairs  # (source vertex, destination vertex) pairs when known

    def mirrored(self):
        return self.transform.mirrored()

    def __repr__(self):
        return "Match(id={} residual={:.6g} {})".format(self.id, self.residual, self.transform)


def compare_islands(island, reference, params=None, **kwargs):
    """Does `island` match `reference`, and under which transform? Returns a Match
    or None. The transform maps the island's UVs onto the reference's."""
    params = _params(params, kwargs)
    if params.mode == TOPOLOGY:
        correspondence = topology_correspondence(island, reference, params.allow_mirroring)
        if correspondence is None:
            return None
        src, dst = correspondence
        transform, _ = fit_similarity(src, dst, allow_flip=params.allow_mirroring,
                                      adjust_scale=params.adjust_scale,
                                      non_uniform_scaling_tolerance=params.non_uniform_scaling_tolerance)
        if transform is None:
            return None
        worst = max_residual(src, dst, transform)
        if worst > params.vertex_threshold:
            return None
        return Match(island, -1, transform, worst, params.mode,
                     reference_id=reference.get("id"), pairs=list(zip(src, dst)))

    if params.mode == VERTICES:
        # The border fit only supplies candidate transforms; the decision is whether the
        # two vertex sets coincide under one of them, within vertex_threshold. A
        # symmetric outline has several equally good fits, hence more than one candidate.
        src, dst = island_vertices(island), island_vertices(reference)
        best = None
        for fit in outline_fits(island, reference, params):
            worst, pairs = _match_vertex_sets(src, dst, fit.transform, params.vertex_threshold)
            if worst == float("inf"):
                continue
            if best is None or worst < best[0]:
                best = (worst, fit.transform, pairs)
        if best is None:
            return None
        return Match(island, -1, best[1], best[0], params.mode,
                     reference_id=reference.get("id"), pairs=best[2])

    transform, residual = outline_fit(island, reference, params)
    if transform is None or residual > params.threshold:
        return None
    return Match(island, -1, transform, residual, params.mode, reference_id=reference.get("id"))


def _params(params, kwargs):
    if params is None:
        return MatchParams(**kwargs)
    return params.copy(**kwargs) if kwargs else params


# ---------------------------------------------------------------- the operators
def find_matching(islands, reference, params=None, **kwargs):
    """Every island of `islands` that matches `reference`, with the transform that would
    stack it on top of the reference. `reference` may be an island dict or an index
    into `islands`; the reference itself is never returned."""
    params = _params(params, kwargs)
    if isinstance(reference, int):
        reference = islands[reference]
    matches = []
    for index, island in enumerate(islands):
        if island is reference:
            continue
        match = compare_islands(island, reference, params)
        if match is None:
            continue
        match.index = index
        match.id = island.get("id", index)
        matches.append(match)
    return matches


def select_matching(islands, references, params=None, **kwargs):
    """Select Matching Islands: the islands (normally the unselected ones) that match at
    least one of the references (normally the selected ones). Returns their indices."""
    params = _params(params, kwargs)
    found = []
    for index, island in enumerate(islands):
        for reference in references:
            if island is reference:
                continue
            if compare_islands(island, reference, params) is not None:
                found.append(index)
                break
    return found


class StackResult:
    """Per-island transforms, plus corrected vertices when asked for."""
    __slots__ = ("transforms", "vertices", "matches", "reference_id")

    def __init__(self, transforms, vertices, matches, reference_id):
        self.transforms = transforms        # island id -> Transform
        self.vertices = vertices            # island id -> [(u, v), ...] parallel to island_vertices
        self.matches = matches
        self.reference_id = reference_id


def stack_matching(islands, reference, matches=None, correct_vertices=False, params=None, **kwargs):
    """Stack Matching Islands: stack every island that matches `reference` exactly
    on top of it. Returns the transform per island id; with correct_vertices, also the
    UV each of the island's vertices should end up at, which is the reference's own
    vertex whenever one is within vertex_threshold (so the stack is bit-exact even when
    the islands differ slightly)."""
    params = _params(params, kwargs)
    if isinstance(reference, int):
        reference = islands[reference]
    if matches is None:
        matches = find_matching(islands, reference, params)
    transforms, vertices = {}, {}
    for match in matches:
        transforms[match.id] = match.transform
        if not (correct_vertices or params.correct_vertices):
            continue
        src = island_vertices(match.island)
        dst = island_vertices(reference)
        worst, pairs = _match_vertex_sets(src, dst, match.transform, params.vertex_threshold)
        if worst == float("inf"):
            # No bijection within the threshold: leave the transform alone rather than
            # dragging vertices onto whatever happened to be nearest.
            vertices[match.id] = [match.transform.apply(p) for p in src]
            continue
        lookup = {(round(p[0] / GRID), round(p[1] / GRID)): q for p, q in pairs}
        vertices[match.id] = [lookup.get((round(p[0] / GRID), round(p[1] / GRID)),
                                         match.transform.apply(p)) for p in src]
    return StackResult(transforms, vertices, matches, reference.get("id"))


def spread_overlapping(islands, gap=1.0, detection_mode=TOUCHING, max_tile_x=0, priorities=None):
    """Spread Overlapping Islands: move overlapping islands apart in whole steps
    along +U (one step is one UDIM tile by default), wrapping into +V after
    `max_tile_x` steps. Returns {island id: (du, dv)}.

    Repeatedly compute the overlap map,
    take the still-unplaced islands most entangled first and give each the lowest step
    index none of its overlapping neighbours already took, move it, and check again."""
    if not islands:
        return {}
    order = list(range(len(islands)))
    state = {}
    for i in order:
        state[i] = {"offset": [0, 0], "tile": 0, "processed": False, "local": None}
    if priorities is None:
        priorities = {}

    current = {i: islands[i] for i in order}
    to_process = list(order)
    processed = []
    guard = 0
    while to_process and guard < 1000:
        guard += 1
        to_check = to_process + processed
        overlap = {i: [] for i in to_check}
        for a_pos, i in enumerate(to_check):
            for j in to_check[a_pos + 1:]:
                if islands_overlap(current[i], current[j], detection_mode):
                    overlap[i].append(j)
                    overlap[j].append(i)
        pending = []
        for i in to_check:
            if not overlap[i]:
                if not state[i]["processed"]:
                    state[i]["processed"] = True
                    processed.append(i)
                continue
            if state[i]["processed"]:
                state[i]["local"] = 0
                continue
            state[i]["local"] = None
            pending.append(i)
        pending.sort(key=lambda i: (-priorities.get(islands[i].get("id", i), 0), len(overlap[i])))
        to_process = []
        for i in pending:
            taken = {state[j]["local"] for j in overlap[i] if state[j]["local"] is not None}
            step = 0
            while step in taken:
                step += 1
            state[i]["local"] = step
            du, dv = _tile_delta(state[i], step, max_tile_x)
            state[i]["offset"][0] += du
            state[i]["offset"][1] += dv
            if du or dv:
                current[i] = translate_island(current[i], du * gap, dv * gap)
            to_process.append(i)
        if not pending:
            break

    return {islands[i].get("id", i): (state[i]["offset"][0] * gap, state[i]["offset"][1] * gap)
            for i in order}


def _tile_delta(entry, step, max_tile_x):
    """The +X walk wraps into the next row of tiles once it would leave the allowed
    tile range; with max_tile_x 0 it walks along +X forever."""
    if max_tile_x <= 0:
        return (step, 0)
    x = entry["tile"] + step
    y = 0
    while x >= max_tile_x:
        x -= max_tile_x
        y += 1
    delta = x - entry["tile"]
    entry["tile"] = x
    return (delta, y)


def undo_spread(offsets):
    """Undo Spread: the inverse translations of a previous spread, which is all the
    stored spread offsets negated."""
    return {island_id: (-du, -dv) for island_id, (du, dv) in offsets.items()}


class BundleResult:
    __slots__ = ("transforms", "bind_groups", "bundles", "unmatched")

    def __init__(self, transforms, bind_groups, bundles, unmatched):
        self.transforms = transforms    # island id -> Transform stacking it on its leader
        self.bind_groups = bind_groups  # island id -> bind_group for the packer
        self.bundles = bundles          # [[island id, ...], ...], the leader first
        self.unmatched = unmatched      # ids that did not match their bundle's leader


def bundles(islands, bundle_ids, params=None, default_bundle=0, first_bind_group=1, **kwargs):
    """Bundles: within every bundle, stack the matching islands onto the bundle's
    first island and then bind the stack together so the packer treats it as one body.
    Returns the stacking transforms and the `bind_group` to send per island.

    `bundle_ids` maps island id (or index) to the bundle number; islands in
    `default_bundle` take part in nothing."""
    params = _params(params, kwargs)
    by_bundle = defaultdict(list)
    for index, island in enumerate(islands):
        island_id = island.get("id", index)
        bundle = bundle_ids.get(island_id, bundle_ids.get(index, default_bundle))
        if bundle == default_bundle:
            continue
        by_bundle[bundle].append(index)

    transforms, bind_groups, stacks, unmatched = {}, {}, [], []
    bind = first_bind_group
    for bundle in sorted(by_bundle):
        members = by_bundle[bundle]
        if len(members) < 2:
            continue
        leader = members[0]
        leader_id = islands[leader].get("id", leader)
        stack = [leader_id]
        transforms[leader_id] = Transform.identity()
        for index in members[1:]:
            island_id = islands[index].get("id", index)
            match = compare_islands(islands[index], islands[leader], params)
            if match is None:
                unmatched.append(island_id)
                continue
            transforms[island_id] = match.transform
            stack.append(island_id)
        if len(stack) < 2:
            transforms.pop(leader_id, None)
            continue
        for island_id in stack:
            bind_groups[island_id] = bind
        stacks.append(stack)
        bind += 1
    return BundleResult(transforms, bind_groups, stacks, unmatched)


class FollowMatch:
    __slots__ = ("follower", "follower_id", "leader_id", "transform", "residual")

    def __init__(self, follower, follower_id, leader_id, transform, residual):
        self.follower = follower
        self.follower_id = follower_id
        self.leader_id = leader_id
        self.transform = transform      # follower UV space -> leader UV space
        self.residual = residual

    def replay(self, pack_transform):
        """The transform the follower has to take so it follows what the packer did to
        its leader: M^-1 . P . M, the leader's motion expressed in the follower's frame."""
        return follower_transform(self.transform, pack_transform)


def follower_transform(match_transform, pack_transform):
    """follower.transform_inverse * leader_transform * follower.transform: express the
    leader's motion in the follower's own frame."""
    return match_transform.inverse().compose(pack_transform).compose(match_transform)


# Follow-group matching pairs islands that were never meant to be edited together, so
# it uses its own fixed, deliberately generous parameters rather than the ones in the UI.
FOLLOW_PARAMS = dict(threshold=0.55, resolution=5000, check_holes=True,
                     vertex_threshold=0.005)


def follow_groups(leaders, followers, mode=OUTLINE, leader_groups=None,
                  follower_groups=None, params=None, pack_transforms=None, **kwargs):
    """Follow Groups: every follower island is matched to a leader island of the same
    follow group, and then replays whatever the packer did to that leader.

    Returns a list of FollowMatch. With `pack_transforms` ({leader id: Transform},
    straight from the packer's output) the composition is done here and returned as
    {follower id: Transform}."""
    if params is None:
        defaults = dict(FOLLOW_PARAMS)
        defaults.update(kwargs)
        params = MatchParams(mode=mode, **defaults)
    else:
        params = _params(params, kwargs)

    matches = []
    for f_index, follower in enumerate(followers):
        follower_id = follower.get("id", f_index)
        group = None if follower_groups is None else follower_groups.get(follower_id, follower_groups.get(f_index))
        best = None
        for l_index, leader in enumerate(leaders):
            leader_id = leader.get("id", l_index)
            if group is not None and leader_groups is not None:
                l_group = leader_groups.get(leader_id, leader_groups.get(l_index))
                if l_group != group:
                    continue
            match = compare_islands(follower, leader, params)
            if match is None:
                continue
            if best is None or match.residual < best[0].residual:
                best = (match, leader_id)
        if best is not None:
            matches.append(FollowMatch(follower, follower_id, best[1], best[0].transform,
                                       best[0].residual))
    if pack_transforms is None:
        return matches
    return {m.follower_id: m.replay(pack_transforms[m.leader_id])
            for m in matches if m.leader_id in pack_transforms}


def island_metrics(islands, area_3d=None, tex_size=0, scale_length=1.0):
    """Island Metrics: the UV area, the 3D area and the texel density they imply:

        texel density [px per 3D unit] = tex_size * sqrt(uv_area / area_3d) / scale_length

    `area_3d` may be a list parallel to `islands` or left out, in which case each
    island's own "area_3d" (which uv_islands puts there) is used."""
    per_island, total_uv, total_3d = [], 0.0, 0.0
    for index, island in enumerate(islands):
        uv_area = island_area(island)
        if area_3d is None:
            three_d = float(island.get("area_3d", 0.0))
        elif isinstance(area_3d, dict):
            three_d = float(area_3d.get(island.get("id", index), 0.0))
        else:
            three_d = float(area_3d[index])
        density = None
        if tex_size and three_d > 1e-18 and uv_area > 1e-18:
            density = tex_size * math.sqrt(uv_area / three_d) / (scale_length or 1.0)
        per_island.append({"id": island.get("id", index), "uv_area": uv_area,
                           "area_3d": three_d, "texel_density": density})
        total_uv += uv_area
        total_3d += three_d
    total_density = None
    if tex_size and total_3d > 1e-18 and total_uv > 1e-18:
        total_density = tex_size * math.sqrt(total_uv / total_3d) / (scale_length or 1.0)
    return {"islands": per_island, "uv_area": total_uv, "area_3d": total_3d,
            "texel_density": total_density}


AXES = {"+X": (1.0, 0.0, 0.0), "-X": (-1.0, 0.0, 0.0),
        "+Y": (0.0, 1.0, 0.0), "-Y": (0.0, -1.0, 0.0),
        "+Z": (0.0, 0.0, 1.0), "-Z": (0.0, 0.0, -1.0)}


def _axis_vector(axis):
    if isinstance(axis, str):
        key = axis.upper()
        if len(key) == 1:
            key = "+" + key
        if key not in AXES:
            raise ValueError("unknown axis: {!r}".format(axis))
        return AXES[key]
    return tuple(float(v) for v in axis)


def align_to_world(uvs, positions, up_axis="+Z", uv_axis=(0.0, 1.0), pivot=None):
    """Align To World Axis, reduced to the primary axis: rotate the island so
    that the 3D `up_axis`, projected onto the island's own surface, points along
    `uv_axis` (+V by default) in UV space.

    `uvs` and `positions` are parallel lists over the island's loops. The surface
    direction is turned into a UV direction with the island's least-squares 3D-to-UV
    Jacobian, which is exact for a flat island and is the average for a curved one.
    Returns the rotation about `pivot` (the UV centroid unless given)."""
    if len(uvs) != len(positions) or len(uvs) < 3:
        raise ValueError("need at least three (uv, position) pairs")
    n = len(uvs)
    cu = sum(p[0] for p in uvs) / n
    cv = sum(p[1] for p in uvs) / n
    cx = sum(p[0] for p in positions) / n
    cy = sum(p[1] for p in positions) / n
    cz = sum(p[2] for p in positions) / n

    # Tangent plane: the two dominant directions of the centered 3D point cloud. The
    # covariance's smallest eigenvector is the normal; the other two span the plane.
    cov = [[0.0] * 3 for _ in range(3)]
    for p in positions:
        d = (p[0] - cx, p[1] - cy, p[2] - cz)
        for i in range(3):
            for j in range(3):
                cov[i][j] += d[i] * d[j]
    normal = _smallest_eigenvector(cov)
    t1 = _orthogonal_to(normal)
    t2 = _cross(normal, t1)

    # Least-squares 2x2 Jacobian from (t1, t2) coordinates to UV.
    m11 = m12 = m22 = 0.0
    bu1 = bu2 = bv1 = bv2 = 0.0
    for (u, v), p in zip(uvs, positions):
        d = (p[0] - cx, p[1] - cy, p[2] - cz)
        a = _dot(d, t1)
        b = _dot(d, t2)
        m11 += a * a
        m12 += a * b
        m22 += b * b
        bu1 += a * (u - cu)
        bu2 += b * (u - cu)
        bv1 += a * (v - cv)
        bv2 += b * (v - cv)
    det = m11 * m22 - m12 * m12
    if abs(det) < 1e-24:
        raise ValueError("island is degenerate in 3D")
    j11 = (bu1 * m22 - bu2 * m12) / det
    j12 = (bu2 * m11 - bu1 * m12) / det
    j21 = (bv1 * m22 - bv2 * m12) / det
    j22 = (bv2 * m11 - bv1 * m12) / det

    up = _axis_vector(up_axis)
    projected = _sub(up, _scale3(normal, _dot(up, normal)))
    length = math.sqrt(_dot(projected, projected))
    if length < 1e-12:
        raise ValueError("the up axis is perpendicular to the island's surface")
    projected = _scale3(projected, 1.0 / length)
    a, b = _dot(projected, t1), _dot(projected, t2)
    du, dv = j11 * a + j12 * b, j21 * a + j22 * b
    if math.hypot(du, dv) < 1e-15:
        raise ValueError("the up axis maps to nothing in UV space")

    target = math.atan2(uv_axis[1], uv_axis[0])
    angle = target - math.atan2(dv, du)
    return Transform.rotation(angle, pivot=(cu, cv) if pivot is None else pivot)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale3(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _orthogonal_to(v):
    axis = (1.0, 0.0, 0.0) if abs(v[0]) < 0.9 else (0.0, 1.0, 0.0)
    out = _cross(v, axis)
    length = math.sqrt(_dot(out, out))
    return _scale3(out, 1.0 / length)


def _smallest_eigenvector(matrix, iterations=200):
    """Inverse power iteration without a solver: iterate on (trace*I - M), whose largest
    eigenvector is the smallest of M. Three-by-three symmetric, so this converges fast
    and needs no linear algebra beyond a matrix-vector product."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    shifted = [[(trace if i == j else 0.0) - matrix[i][j] for j in range(3)] for i in range(3)]
    v = (0.5773502691896258, 0.5773502691896258, 0.5773502691896258)
    for _ in range(iterations):
        w = tuple(sum(shifted[i][j] * v[j] for j in range(3)) for i in range(3))
        length = math.sqrt(_dot(w, w))
        if length < 1e-300:
            return (0.0, 0.0, 1.0)
        new = _scale3(w, 1.0 / length)
        if abs(_dot(new, v)) > 1.0 - 1e-18:
            v = new
            break
        v = new
    return v
