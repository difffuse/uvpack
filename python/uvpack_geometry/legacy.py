#!/usr/bin/env python3
"""UV island extraction and write-back. Standard library only, so the same code
runs inside Blender (bmesh adapter) and standalone on OBJ files.

  python3 uv_islands.py extract mesh.obj > islands.json
  ./uvpack_cli islands.json > layout.json
  python3 uv_islands.py apply mesh.obj layout.json packed.obj

Islands are connected components of faces that share a UV vertex (Blender's
default) or, with --split-at-vertices, an edge used in opposite directions by two
faces. The outline is built from boundary edges and the loops are then nested by
containment: a loop at even depth is a part's outer ring, a loop at odd depth is
a hole of the part around it. An island whose faces only meet at a UV vertex has
several disjoint parts, which go out as uvpack_cli's "parts". Faces with fewer than
three distinct UV vertices are ignored. Self-overlapping islands produce
self-intersecting outlines, which uvpack_cli reports and rejects (or repairs).
"""
import json, math, sys
from collections import defaultdict

PRECISION = 1e-7


def _key(uv):
    return (round(uv[0] / PRECISION), round(uv[1] / PRECISION))


def _ring_area(ring):
    a = 0.0
    for i in range(len(ring)):
        x0, y0 = ring[i]; x1, y1 = ring[(i + 1) % len(ring)]
        a += x0 * y1 - x1 * y0
    return 0.5 * a


def _point_in_ring(pt, ring):
    x, y = pt
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]; x1, y1 = ring[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            if x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                inside = not inside
    return inside


def _ring_inside(child, parent):
    """Majority vote, so a vertex the two rings share (a pinch point) cannot decide
    containment on its own. Boundary loops of one face set never cross each other."""
    return sum(1 for p in child if _point_in_ring(p, parent)) * 2 > len(child)


def _nest_loops(loops):
    """Group boundary loops into (outer, holes) parts by containment depth: a loop at
    even depth starts a part, a loop at odd depth is a hole of the part holding it.

    Faces that meet at a single UV vertex are one island under Blender's rule but their
    boundary is two disjoint rings, not a ring with a hole. Taking the largest loop as
    the outer ring and every other loop as a hole would produce holes that lie outside
    their outer ring, which uvpack_cli rejects outright."""
    loops = [r for r in loops if abs(_ring_area(r)) > 0]
    order = sorted(range(len(loops)), key=lambda i: -abs(_ring_area(loops[i])))
    parent, depth = {}, {}
    for pos, i in enumerate(order):
        best = None
        for j in order[:pos]:  # only a larger loop can contain this one
            if _ring_inside(loops[i], loops[j]) and (best is None or abs(_ring_area(loops[j])) < abs(_ring_area(loops[best]))):
                best = j
        parent[i], depth[i] = best, 0 if best is None else depth[best] + 1
    parts = []
    for i in order:
        if depth[i] % 2:
            continue
        outer = loops[i]
        holes = [loops[j] for j in order if parent[j] == i and depth[j] == depth[i] + 1]
        # Outer counter-clockwise, holes clockwise (uvpack_cli normalizes anyway).
        if _ring_area(outer) < 0: outer = outer[::-1]
        holes = [h[::-1] if _ring_area(h) > 0 else h for h in holes]
        parts.append((outer, holes))
    return parts


class UnionFind:
    def __init__(self, n):
        self.p = list(range(n))
    def find(self, i):
        while self.p[i] != i:
            self.p[i] = self.p[self.p[i]]; i = self.p[i]
        return i
    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b: self.p[a] = b


def extract_islands(faces_uv, split_at_vertices=False, area_3d=None):
    """faces_uv: list of faces, each a list of (u, v). Returns a list of island dicts:
    {"id", "outer", "holes", "faces": [face indices], "area_3d" (optional)}."""
    n = len(faces_uv)
    uf = UnionFind(n)
    if split_at_vertices:
        # Faces are connected when they share an edge traversed in opposite directions.
        directed = {}
        for f, face in enumerate(faces_uv):
            keys = [_key(uv) for uv in face]
            for i in range(len(keys)):
                a, b = keys[i], keys[(i + 1) % len(keys)]
                if a == b: continue
                directed.setdefault((a, b), []).append(f)
        for (a, b), owners in directed.items():
            for g in directed.get((b, a), []):
                for f in owners: uf.union(f, g)
    else:
        by_vertex = defaultdict(list)
        for f, face in enumerate(faces_uv):
            for uv in face: by_vertex[_key(uv)].append(f)
        for owners in by_vertex.values():
            for g in owners[1:]: uf.union(owners[0], g)

    groups = defaultdict(list)
    for f in range(n):
        if len({_key(uv) for uv in faces_uv[f]}) < 3: continue  # degenerate in UV
        groups[uf.find(f)].append(f)

    islands = []
    for island_id, (_, members) in enumerate(sorted(groups.items(), key=lambda kv: min(kv[1]))):
        # Boundary edges: directed edges whose reverse is not used by a face of the island.
        # Faces with clockwise UV winding are flipped so all edges run counter-clockwise.
        directed = defaultdict(int)
        coords = {}
        for f in members:
            face = [uv for i, uv in enumerate(faces_uv[f]) if i == 0 or _key(uv) != _key(faces_uv[f][i - 1])]
            if len(face) >= 2 and _key(face[0]) == _key(face[-1]): face = face[:-1]
            if len(face) < 3: continue
            if _ring_area(face) < 0: face = face[::-1]
            keys = [_key(uv) for uv in face]
            for k, uv in zip(keys, face): coords[k] = uv
            for i in range(len(keys)):
                directed[(keys[i], keys[(i + 1) % len(keys)])] += 1
        boundary = defaultdict(list)
        for (a, b), count in directed.items():
            if directed.get((b, a), 0) == 0:
                boundary[a].append(b)
        loops = []
        used = set()
        for start in list(boundary.keys()):
            for first in boundary[start]:
                if (start, first) in used: continue
                loop = [start]; a, b = start, first
                used.add((a, b))
                guard = 0
                while b != start and guard < 100000:
                    loop.append(b)
                    nexts = [c for c in boundary.get(b, []) if (b, c) not in used]
                    if not nexts: break
                    # Prefer the tightest left turn to keep loops simple at pinch vertices.
                    # Faces are wound counter-clockwise above, so the interior lies to
                    # the left of every boundary edge; turning left at a vertex where two
                    # pieces touch stays on the piece the walk arrived from, while turning right
                    # crosses over and traces the two pieces as one figure-eight ring
                    # (which uvpack_cli then rejects as a self-intersecting outer ring).
                    if len(nexts) > 1:
                        ax, ay = coords[a]; bx, by = coords[b]
                        def angle(c):
                            cx, cy = coords[c]
                            return math.atan2((bx - ax) * (cy - by) - (by - ay) * (cx - bx), (bx - ax) * (cx - bx) + (by - ay) * (cy - by))
                        nexts.sort(key=angle, reverse=True)
                    c = nexts[0]
                    used.add((b, c)); a, b = b, c
                    guard += 1
                if b == start and len(loop) >= 3:
                    loops.append([list(coords[k]) for k in loop])
        if not loops:
            continue
        parts = _nest_loops(loops)
        if not parts:
            continue
        outer, holes = parts[0]
        island = {"id": island_id, "outer": outer, "holes": holes, "faces": members}
        if len(parts) > 1:
            # Disjoint pieces of one island: uvpack_cli packs them as one rigid body.
            island["parts"] = [{"outer": o, "holes": h} for o, h in parts[1:]]
        if area_3d is not None:
            island["area_3d"] = float(sum(area_3d[f] for f in members))
        islands.append(island)
    return islands


# ---------------------------------------------------------------- OBJ support
def load_obj(path):
    verts, uvs, faces = [], [], []
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if not parts: continue
            if parts[0] == "v": verts.append(tuple(float(x) for x in parts[1:4]))
            elif parts[0] == "vt": uvs.append((float(parts[1]), float(parts[2]) if len(parts) > 2 else 0.0))
            elif parts[0] == "f":
                corners = []
                for token in parts[1:]:
                    idx = token.split("/")
                    v = int(idx[0]); t = int(idx[1]) if len(idx) > 1 and idx[1] else 0
                    corners.append((v - 1 if v > 0 else len(verts) + v, t - 1 if t > 0 else (len(uvs) + t if t else None)))
                faces.append(corners)
    return verts, uvs, faces


def face_area_3d(verts, corners):
    pts = [verts[v] for v, _ in corners]
    area = 0.0
    for i in range(1, len(pts) - 1):
        ax, ay, az = (pts[i][k] - pts[0][k] for k in range(3))
        bx, by, bz = (pts[i + 1][k] - pts[0][k] for k in range(3))
        cx, cy, cz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
        area += 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz)
    return area


def extract_obj(path, split_at_vertices=False):
    verts, uvs, faces = load_obj(path)
    faces_uv, area3d, face_ids = [], [], []
    for f, corners in enumerate(faces):
        if any(t is None for _, t in corners): continue
        faces_uv.append([uvs[t] for _, t in corners])
        area3d.append(face_area_3d(verts, corners))
        face_ids.append(f)
    islands = extract_islands(faces_uv, split_at_vertices, area3d)
    for isl in islands: isl["faces"] = [face_ids[i] for i in isl["faces"]]
    return {"target": [0, 0, 1, 1], "islands": islands}


# ------------------------------------------------- texel snapping output
# The vertex snap modes ("outline", "all") move every boundary vertex on its own, so
# the layout carries per-island `vertices` arrays instead of relying on the transform
# alone. They follow the island's INPUT rings one for one, so an original UV can be
# looked up directly; anything not listed (an interior vertex, or a vertex the
# packer's input repair dropped) keeps the transform.
def texel_snapper(layout_item, snap_report):
    """The snap the client owes the packer, or None. Only mode "all" asks for one:
    the packer never saw the interior vertices, so it cannot snap them itself.

    `snap_report` is the layout's `snap_to_texels` block."""
    if not layout_item.get("snap_interior") or not snap_report:
        return None
    tex = float(snap_report.get("tex_size") or 0)
    if tex <= 0:
        return None
    # Texel centres sit at (k + 0.5)/tex; the nearest one has k = floor(c * tex).
    if snap_report.get("target") == "center":
        return lambda c: (math.floor(c * tex) + 0.5) / tex
    return lambda c: math.floor(c * tex + 0.5) / tex


def vertex_map(layout_item, island):
    """{original UV key: aligned UV} for one island, or None when the layout carries
    no per-vertex output for it."""
    verts = layout_item.get("vertices")
    if not verts:
        return None
    out = {}

    def pair(src_rings, dst_rings):
        for src, dst in zip(src_rings, dst_rings):
            for a, b in zip(src, dst):
                out[_key(a)] = b

    pair([island["outer"]] + list(island.get("holes", [])),
         [verts["outer"]] + list(verts.get("holes", [])))
    for src_part, dst_part in zip(island.get("parts", []), verts.get("parts", [])):
        pair([src_part["outer"]] + list(src_part.get("holes", [])),
             [dst_part["outer"]] + list(dst_part.get("holes", [])))
    return out


def map_uv(uv, transform, vmap=None, snap=None):
    """One UV through a layout entry: the aligned vertex when the packer listed it,
    otherwise the transform (snapped afterwards when the mode asks for it)."""
    if vmap is not None:
        hit = vmap.get(_key(uv))
        if hit is not None:
            return (hit[0], hit[1])
    u, v = uv
    q = (transform[0] * u + transform[1] * v + transform[2],
         transform[3] * u + transform[4] * v + transform[5])
    return (snap(q[0]), snap(q[1])) if snap is not None else q


def apply_layout(path, layout, out_path, islands_doc):
    """Write a copy of the OBJ with vt entries transformed per island. Each vt index is
    assigned to the island of the first face using it."""
    verts, uvs, faces = load_obj(path)
    face_to_island = {}
    for isl in islands_doc["islands"]:
        for f in isl["faces"]: face_to_island[f] = isl["id"]
    by_id = {isl["id"]: isl for isl in islands_doc["islands"]}
    snap_report = layout.get("snap_to_texels") or {}
    entries = {}
    for item in layout["islands"]:
        island = by_id.get(item["id"])
        entries[item["id"]] = (item["transform"],
                               vertex_map(item, island) if island else None,
                               texel_snapper(item, snap_report))
    vt_owner = {}
    for f, corners in enumerate(faces):
        if f not in face_to_island: continue
        for _, t in corners:
            if t is not None: vt_owner.setdefault(t, face_to_island[f])
    new_uvs = list(uvs)
    for t, island_id in vt_owner.items():
        entry = entries.get(island_id)
        if entry is None: continue
        new_uvs[t] = map_uv(uvs[t], *entry)
    with open(path) as src, open(out_path, "w") as dst:
        vt_index = 0
        for line in src:
            if line.startswith("vt "):
                u, v = new_uvs[vt_index]; vt_index += 1
                dst.write(f"vt {u:.8f} {v:.8f}\n")
            else:
                dst.write(line)


def main(argv):
    if len(argv) >= 2 and argv[0] == "extract":
        doc = extract_obj(argv[1], split_at_vertices="--split-at-vertices" in argv)
        json.dump(doc, sys.stdout); print(); return 0
    if len(argv) >= 4 and argv[0] == "apply":
        islands_doc = extract_obj(argv[1], split_at_vertices="--split-at-vertices" in argv)
        with open(argv[2]) as fh: layout = json.load(fh)
        apply_layout(argv[1], layout, argv[3], islands_doc)
        print("wrote", argv[3]); return 0
    print(__doc__); return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
