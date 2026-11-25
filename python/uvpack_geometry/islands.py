"""Topology-aware, loss-accounted mesh transport for the native packer.

This module owns numeric copies. It neither repairs invalid UVs nor creates hulls;
those decisions belong to the native input policy. Legacy coordinate-only callers
remain in :mod:`uvpack_geometry.legacy`.
"""
from collections import defaultdict
from dataclasses import dataclass, field
from fractions import Fraction
import math
import operator
import struct
import sys

from .legacy import UnionFind, _point_in_ring, texel_snapper


def _area(ring):
    if not ring:
        return 0.0
    x, y = ring[0]
    return math.fsum((a[0]-x)*(b[1]-y)-(b[0]-x)*(a[1]-y)
                     for a, b in zip(ring, ring[1:]+ring[:1])) / 2


def _collinear(points):
    if not points:
        return True
    origin = points[0]
    other = next((p for p in points if p != origin), None)
    if other is None:
        return True
    dx, dy = other[0]-origin[0], other[1]-origin[1]
    return all(dx*(p[1]-origin[1])-dy*(p[0]-origin[0]) == 0 for p in points)


def _orientation(a, b, c):
    left = (b[0]-a[0])*(c[1]-a[1])
    right = (b[1]-a[1])*(c[0]-a[0])
    determinant = left-right
    # The bound only selects the fast sign calculation. Near cancellation,
    # overflow or underflow, exact binary-rational arithmetic decides contact;
    # a small but real gap is never rounded into a boundary intersection.
    if (math.isfinite(determinant) and abs(determinant) >= sys.float_info.min
            and abs(determinant) > 8*sys.float_info.epsilon*(abs(left)+abs(right))):
        return 1 if determinant > 0 else -1
    ax, ay = map(Fraction, a); bx, by = map(Fraction, b); cx, cy = map(Fraction, c)
    determinant = (bx-ax)*(cy-ay)-(by-ay)*(cx-ax)
    return (determinant > 0)-(determinant < 0)


def _boundary_contacts(loops, coordinates):
    """Detect nonsimple boundaries within one connected source component.

    Node identity alone cannot detect overlapping collinear slit edges or a
    vertex touching the interior of an unrelated edge. Bounding-volume queries
    avoid comparing every pair on ordinary large outlines, including long runs
    of collinear subdivisions. Different source components remain rigid parts.
    """
    edges = [(coordinates[a], coordinates[b], group, i, len(loop))
             for group, loop in enumerate(loops)
             for i, (a, b) in enumerate(zip(loop, loop[1:]+loop[:1]))]
    if any(a == b for a, b, *_ in edges): return True
    if not edges: return False
    boxes = [(min(a[0],b[0]), min(a[1],b[1]), max(a[0],b[0]), max(a[1],b[1]))
             for a, b, *_ in edges]
    rational = {}

    def exact(point):
        if point not in rational: rational[point] = tuple(map(Fraction, point))
        return rational[point]

    def project(point, normal):
        x,y = exact(point)
        return x*normal[0]+y*normal[1]

    def build(indices):
        bound = (min(boxes[i][0] for i in indices), min(boxes[i][1] for i in indices),
                 max(boxes[i][2] for i in indices), max(boxes[i][3] for i in indices))
        if len(indices) <= 8: return bound, None, None, indices, None, None
        # XY boxes alone have quadratic behavior on valid diagonal strips:
        # parallel long segments all have overlapping boxes. Each internal node
        # also bounds projection onto a long edge's normal. Exact projections
        # make this a conservative broad phase without rounding away contacts.
        longest = max(indices, key=lambda i:math.hypot(edges[i][1][0]-edges[i][0][0],
                                                       edges[i][1][1]-edges[i][0][1]))
        a,b,*_ = edges[longest]
        ax,ay = exact(a); bx,by = exact(b); dx,dy = bx-ax,by-ay
        normal, interval, projected = None, None, None
        if dx and dy:  # An axis-aligned normal adds nothing to the XY bounds.
            unit = max(abs(dx),abs(dy)); normal = (-dy/unit,dx/unit)
            projected = {i:sorted((project(edges[i][0],normal),project(edges[i][1],normal))) for i in indices}
            interval = (min(p[0] for p in projected.values()),max(p[1] for p in projected.values()))
        centers = [(boxes[i][0]*.5+boxes[i][2]*.5, boxes[i][1]*.5+boxes[i][3]*.5) for i in indices]
        axis = max(range(2), key=lambda k:max(c[k] for c in centers)-min(c[k] for c in centers))
        spread = max(c[axis] for c in centers)-min(c[axis] for c in centers)
        if projected is not None and max(sum(p)/2 for p in projected.values())-min(sum(p)/2 for p in projected.values()) >= spread:
            indices.sort(key=lambda i:sum(projected[i])/2)
        else:
            indices.sort(key=lambda i:boxes[i][axis]*.5+boxes[i][axis+2]*.5)
        mid = len(indices)//2
        return bound, build(indices[:mid]), build(indices[mid:]), None, normal, interval

    def overlaps(a, b):
        return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]

    def on(a, b, c):
        return min(a[0],b[0]) <= c[0] <= max(a[0],b[0]) and min(a[1],b[1]) <= c[1] <= max(a[1],b[1])

    root = build(list(range(len(edges))))
    for i, (a,b,group,index,size) in enumerate(edges):
        ax,ay = exact(a); bx,by = exact(b); dx,dy = bx-ax,by-ay
        query_normal = (-dy,dx) if dx and dy else None
        offset = project(a,query_normal) if query_normal is not None else None
        pending = [root]
        while pending:
            bound, left, right, indices, normal, interval = pending.pop()
            if not overlaps(boxes[i], bound): continue
            if query_normal is not None:
                # The symmetric separating axis matters too: a diagonal query
                # spans many short vertical edges' XY boxes, although its line
                # crosses only a small part of their containing rectangle.
                nx,ny = query_normal
                low = (bound[0] if nx >= 0 else bound[2],bound[1] if ny >= 0 else bound[3])
                high = (bound[2] if nx >= 0 else bound[0],bound[3] if ny >= 0 else bound[1])
                if offset < project(low,query_normal) or offset > project(high,query_normal): continue
            if normal is not None:
                start,end = sorted((project(a,normal),project(b,normal)))
                if end < interval[0] or start > interval[1]: continue
            if indices is None:
                pending.extend((left,right)); continue
            for j in indices:
                if j <= i or not overlaps(boxes[i], boxes[j]): continue
                c,d,other,other_index,_ = edges[j]
                if group == other and abs(index-other_index) in (1,size-1):
                    # Adjacent edges may share their endpoint, but cannot double
                    # back over one another or introduce a zero-length segment.
                    common = set((a,b)).intersection((c,d))
                    if len(common) != 1: return True
                    pivot = next(iter(common))
                    p = b if a == pivot else a; q = d if c == pivot else c
                    if _orientation(pivot,p,q) == 0 and (on(pivot,p,q) or on(pivot,q,p)): return True
                    continue
                ab, ad, ca, cb = _orientation(a,b,c), _orientation(a,b,d), _orientation(c,d,a), _orientation(c,d,b)
                if (ab*ad < 0 and ca*cb < 0 or
                        ab == 0 and on(a,b,c) or ad == 0 and on(a,b,d) or
                        ca == 0 and on(c,d,a) or cb == 0 and on(c,d,b)): return True
    return False


def _ids(values, count, name, unique=False):
    try:
        result = [operator.index(v) for v in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain integers') from exc
    if len(result) != count or any(v < 0 or v > 2**53-1 for v in result):
        raise ValueError(f'{name}: incorrect length or unsafe ID')
    if unique and len(set(result)) != count:
        raise ValueError(f'{name}: duplicate ID')
    return result


def _points(values):
    try:
        result = [tuple(float(x) for x in uv) for uv in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('UVs must be finite pairs') from exc
    if any(len(uv) != 2 or not all(map(math.isfinite, uv)) for uv in result):
        raise ValueError('UVs must be finite pairs')
    return result


@dataclass
class Extraction:
    islands: list
    issues: list
    face_ids: tuple
    corner_ids: tuple
    _uvs: tuple = field(repr=False)
    _indices: dict = field(repr=False)
    _rings: dict = field(repr=False)


def _parts(loops, coordinates):
    """Nest rings while retaining source-node identity for snap writeback."""
    def points(loop): return [coordinates[k] for k in loop]
    loops = [r for r in loops if _area(points(r)) != 0]
    order = sorted(range(len(loops)), key=lambda i: -abs(_area(points(loops[i]))))
    parent, depth = {}, {}
    for pos, i in enumerate(order):
        owners = [j for j in order[:pos] if sum(_point_in_ring(p, points(loops[j]))
                  for p in points(loops[i])) * 2 > len(loops[i])]
        parent[i] = min(owners, key=lambda j: abs(_area(points(loops[j])))) if owners else None
        depth[i] = 0 if parent[i] is None else depth[parent[i]]+1
    result = []
    for i in order:
        if depth[i] % 2:
            continue
        outer = loops[i]
        if _area(points(outer)) < 0: outer = outer[::-1]
        holes = [loops[j] for j in order if parent[j] == i and depth[j] == depth[i]+1]
        holes = [h[::-1] if _area(points(h)) > 0 else h for h in holes]
        result.append([outer]+holes)
    return result


def extract_mesh_islands(uvs, face_starts, face_sizes, corner_verts, *,
                         chart_ids=None, face_ids=None, corner_ids=None,
                         face_area_uv=None, face_area_3d=None, face_area_3d_global=None):
    uvs = _points(uvs)
    count = len(face_sizes)
    starts = _ids(face_starts, count, 'face_starts')
    sizes = _ids(face_sizes, count, 'face_sizes')
    verts = _ids(corner_verts, len(uvs), 'corner_verts')
    faces = _ids(range(count) if face_ids is None else face_ids, count, 'face_ids', True)
    corners = _ids(range(len(uvs)) if corner_ids is None else corner_ids,
                   len(uvs), 'corner_ids', True)
    charts = None if chart_ids is None else _ids(chart_ids, count, 'chart_ids')
    offset = 0
    for start, size in zip(starts, sizes):
        if start != offset or size < 3:
            raise ValueError('faces must partition corners contiguously with at least 3 corners')
        offset += size
    if offset != len(uvs): raise ValueError('faces must account for every corner')
    indices = [list(range(s, s+n)) for s, n in zip(starts, sizes)]
    areas = {}
    for name, values in [('area_uv', face_area_uv), ('area_3d', face_area_3d),
                         ('area_3d_global', face_area_3d_global)]:
        if values is not None:
            values = [float(v) for v in values]
            if len(values) != count or any(not math.isfinite(v) or v < 0 for v in values):
                raise ValueError(f'{name}: expected finite nonnegative per-face values')
            areas[name] = values
    uf = UnionFind(count)
    edges = defaultdict(list)
    for f, cs in enumerate(indices):
        for a, b in zip(cs, cs[1:]+cs[:1]):
            key = tuple(sorted((verts[a], verts[b])))
            endpoints = (uvs[a], uvs[b]) if verts[a] < verts[b] else (uvs[b], uvs[a])
            edges[key].append((f, endpoints))
    if charts is None:
        for owners in edges.values():
            # Ambiguous nonmanifold edges are never used to invent connectivity.
            if len(owners) == 2 and owners[0][1] == owners[1][1]:
                uf.union(owners[0][0], owners[1][0])
    groups = defaultdict(list)
    for f in range(count): groups[charts[f] if charts is not None else uf.find(f)].append(f)
    islands, issues, index_map, ring_map = [], [], {}, {}
    for ordinal, (chart, members) in enumerate(sorted(groups.items(), key=lambda kv:min(kv[1]))):
        iid = chart if charts is not None else ordinal
        cs = [c for f in members for c in indices[f]]
        nodes = defaultdict(list)
        for c in cs: nodes[(verts[c], uvs[c])].append(c)
        coords = {k:k[1] for k in nodes}
        directed = defaultdict(int)
        for f in members:
            poly = indices[f]
            area = _area([uvs[c] for c in poly])
            if area == 0: continue
            if area < 0: poly = poly[::-1]
            keys = [(verts[c], uvs[c]) for c in poly]
            for a, b in zip(keys, keys[1:]+keys[:1]):
                if a != b: directed[a,b] += 1
        boundary = defaultdict(list)
        for (a,b), n in directed.items():
            if n > directed.get((b,a),0): boundary[a].append(b)
        used, loops = set(), []
        for start, nexts in boundary.items():
            for first in nexts:
                if (start,first) in used: continue
                loop, a, b = [start], start, first
                used.add((a,b))
                while b != start:
                    loop.append(b)
                    candidates = [c for c in boundary.get(b,[]) if (b,c) not in used]
                    if not candidates: break
                    def angle(c):
                        ax,ay=coords[a]; bx,by=coords[b]; cx,cy=coords[c]
                        return math.atan2((bx-ax)*(cy-by)-(by-ay)*(cx-bx),
                                          (bx-ax)*(cx-bx)+(by-ay)*(cy-by))
                    c=max(candidates,key=angle)
                    used.add((b,c)); a,b=b,c
                if b == start and len(loop)>=3: loops.append(loop)
        ambiguous = any(n > 1 for n in directed.values())
        # Nest only within a connected source component. Disconnected rigid parts
        # may overlap/nest in UV space; that does not make their surfaces holes.
        node_indices = {node: i for i, node in enumerate(nodes)}
        components = UnionFind(len(nodes))
        for a, b in directed:
            components.union(node_indices[a], node_indices[b])
        component_loops = defaultdict(list)
        for loop in loops:
            component_loops[components.find(node_indices[loop[0]])].append(loop)
        # A closed walk can revisit a source node where a hole touches the
        # exterior. Its face union may be valid, but that walk is not a simple
        # packing ring. Retain the full source envelope and report no_outline;
        # only explicit native recovery may choose a conservative footprint.
        # Starting at the contact node can split the same walk into two
        # individually simple rings. Shared source nodes between those rings
        # are equally unsuitable; classification must not depend on traversal.
        boundary_nodes = [node for loop in loops for node in loop]
        non_simple = (len(set(boundary_nodes)) != len(boundary_nodes) or
                      any(_boundary_contacts(rings, coords) for rings in component_loops.values()))
        parts = [] if ambiguous or non_simple else [part for rings in component_loops.values()
                                                     for part in _parts(rings, coords)]
        def encode(rings):
            return {'outer':[coords[k] for k in rings[0]],
                    'holes':[[coords[k] for k in ring] for ring in rings[1:]]}
        island = {'id':iid, 'faces':[faces[f] for f in members],
                  'corners':[corners[c] for c in cs], 'envelope_points':[uvs[c] for c in cs],
                  'outer':[], 'holes':[]}
        if parts:
            island.update(encode(parts[0]))
            if len(parts)>1: island['parts']=[encode(p) for p in parts[1:]]
        else:
            issues.append({'id':iid, 'reason':'ambiguous_boundary' if ambiguous else 'degenerate_chart' if all(
                _collinear([uvs[c] for c in indices[f]]) for f in members) else 'no_outline',
                'face_ids':island['faces'], 'corner_ids':island['corners']})
        for name, values in areas.items(): island[name]=math.fsum(values[f] for f in members)
        if 'area_uv' in areas:
            for name,suffix in [('area_3d',''),('area_3d_global','_global')]:
                if name in areas:
                    island['density_cross_sum'+suffix]=math.fsum(
                        math.sqrt(areas['area_uv'][f])*math.sqrt(areas[name][f]) for f in members)
        islands.append(island); index_map[iid]=cs
        ring_map[iid]=[[[tuple(nodes[k]) for k in ring] for ring in p] for p in parts]
    return Extraction(islands,issues,tuple(faces),tuple(corners),tuple(uvs),index_map,ring_map)


def apply_mesh_layout(original_uvs, extraction, layout, *, precision='float64'):
    original = _points(original_uvs)
    if tuple(original) != extraction._uvs: raise ValueError('UVs differ from extraction input')
    if precision not in ('float32','float64'): raise ValueError('unknown storage precision')
    entries = layout.get('islands',[]) if isinstance(layout,dict) else layout
    snap_report = layout.get('snap_to_texels',{}) if isinstance(layout,dict) else {}
    by_id = {}
    for item in entries:
        if not isinstance(item, dict) or 'id' not in item:
            raise ValueError('layout entries require an ID')
        iid = _ids([item['id']],1,'layout id')[0]
        if iid in by_id: raise ValueError('duplicate layout ID')
        by_id[iid]=item
    if set(by_id) != set(extraction._indices): raise ValueError('layout must account for every island')
    output = list(original)
    for iid, cs in extraction._indices.items():
        item = by_id[iid]
        try: t = [float(x) for x in item['transform']]
        except (KeyError,TypeError,ValueError) as exc: raise ValueError('missing/invalid transform') from exc
        if len(t)!=9 or not all(map(math.isfinite,t)) or t[6:] != [0.,0.,1.]:
            raise ValueError('expected finite affine 3x3 transform')
        snap = texel_snapper(item,snap_report)
        for c in cs:
            x,y=original[c]; q=(t[0]*x+t[1]*y+t[2],t[3]*x+t[4]*y+t[5])
            output[c] = tuple(snap(v) for v in q) if snap else q
        if 'vertices' in item:
            supplied = item['vertices']
            if not isinstance(supplied, dict) or 'outer' not in supplied:
                raise ValueError('snapped vertices require an outer ring')
            parts = [supplied]+list(supplied.get('parts',[]))
            expected = extraction._rings[iid]
            if len(parts)!=len(expected): raise ValueError('snapped part count mismatch')
            for p,rings in zip(parts,expected):
                if not isinstance(p, dict) or 'outer' not in p:
                    raise ValueError('snapped parts require an outer ring')
                incoming = [p['outer']]+list(p.get('holes',[]))
                if len(incoming)!=len(rings): raise ValueError('snapped ring count mismatch')
                for values, mapping in zip(incoming,rings):
                    values=_points(values)
                    if len(values)!=len(mapping): raise ValueError('snapped vertex count mismatch')
                    for uv, owners in zip(values,mapping):
                        for c in owners: output[c]=uv
    try:
        if precision=='float32': output=[struct.unpack('ff',struct.pack('ff',*p)) for p in output]
    except (OverflowError,struct.error) as exc: raise ValueError('UV storage overflow') from exc
    if not all(math.isfinite(v) for p in output for v in p): raise ValueError('nonfinite output')
    return output
