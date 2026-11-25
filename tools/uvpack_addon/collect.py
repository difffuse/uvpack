# One pass over the edit-mode meshes: every visible island, sorted into the ones to
# pack, the ones that only stand in the way (obstacles) and the ones held in place.
# Everything downstream -- the request builder, the UV tools, the result apply --
# works on what this module produces.
import bpy
import bmesh
from collections import Counter

from . import uv_islands, uv_similarity
from .props import (BUNDLE_ATTR, FOLLOW_ATTR, DEFAULT_BUNDLE, SimpleSettings,
                    MATCH_MODE_ITEMS, int_attribute)
from .sets import face_set_keys, sync_rule_sets, set_attribute


class Collected:
    """What one pass over the edit-mode meshes produced."""

    def __init__(self):
        self.owners = []      # global face index -> (BMFace, uv layer)
        self.meshes = []      # (object, bmesh)
        self.islands = []     # island dicts to pack, with "faces" in global indices
        self.original = []    # global face index -> the UV loops the island was read from
        self.obstacles = []   # polygons the engine must avoid
        self.frozen_islands = []  # islands left where they were (pinned or unselected)

    @property
    def frozen(self):
        return len(self.frozen_islands)

    def selected(self):
        return [i for i in self.islands if i.get("selected")]

    def unselected(self):
        return [i for i in self.islands if not i.get("selected")]

    def followers(self):
        """The islands the pack will not move and that can therefore follow one that it
        does: the unselected ones. A pinned island is held in place on purpose, so it
        never follows anything."""
        return [i for i in self.frozen_islands if not i.get("selected") and not i.get("pinned")]

    def update(self):
        for obj, _ in self.meshes:
            bmesh.update_edit_mesh(obj.data)


def island_polygons(isl):
    """Every rigid part of an island as an {"outer", "holes"} obstacle body."""
    out = [{"outer": isl["outer"], "holes": isl.get("holes", [])}]
    for part in isl.get("parts", ()):
        out.append({"outer": part["outer"], "holes": part.get("holes", [])})
    return out


def _bbox(isl):
    xs, ys = [], []
    for poly in island_polygons(isl):
        for x, y in poly["outer"]:
            xs.append(x)
            ys.append(y)
    return (min(xs), min(ys), max(xs), max(ys)) if xs else (0.0, 0.0, 0.0, 0.0)


def _boxes_overlap(a, b):
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


def collect(context, st, rule=None, with_geometry=False):
    """Extract every island of the edit-mode meshes and sort it into packed, obstacle
    and frozen. Island connectivity is evaluated per mesh, which is Blender's own rule.

    `st` is the scene settings, or a `SimpleSettings` when the operator was called with
    explicit properties; only the handful of attributes read here have to exist.

    `with_geometry` also attaches the per-loop UVs, the unique vertex list and the loop
    positions in 3D, which the vertex-based match modes and Align To World Axis need."""
    got = Collected()
    objects = [o for o in context.objects_in_mode_unique_data if o.type == "MESH"]
    objects_bms = []
    for obj in objects:
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        objects_bms.append((obj, bm))
        got.meshes.append((obj, bm))

    method = rule.method if rule is not None else None
    keys = face_set_keys(objects_bms, method) if method not in (None, "MANUAL") else {}
    key_to_num = sync_rule_sets(rule, keys) if keys else {}

    pinned_islands, other_islands, packed = [], [], []
    for obj, bm in objects_bms:
        uv_layer = bm.loops.layers.uv.verify()
        manual = set_attribute(bm) if method == "MANUAL" else None
        bundle_layer = int_attribute(bm, BUNDLE_ATTR, create=False)
        follow_layer = int_attribute(bm, FOLLOW_ATTR, create=False)
        active_face = bm.faces.active
        faces_uv, area_3d, local_faces, local_owner, positions = [], [], [], [], []
        for face in bm.faces:
            if face.hide:
                continue
            uvs = [tuple(loop[uv_layer].uv) for loop in face.loops]
            faces_uv.append(uvs)
            positions.append([tuple(loop.vert.co) for loop in face.loops])
            area_3d.append(face.calc_area())
            local_faces.append(face)
            local_owner.append(len(got.owners))
            got.owners.append((face, uv_layer))
            # A streamed run writes the UVs over and over, once per improved layout,
            # and every transform maps the ORIGINAL UVs -- so keep them.
            got.original.append(uvs)
        if not faces_uv:
            continue
        for isl in uv_islands.extract_islands(faces_uv,
                                              split_at_vertices=st.split_at_vertices,
                                              area_3d=area_3d):
            local = isl["faces"]
            isl["faces"] = [local_owner[f] for f in local]
            faces = [local_faces[f] for f in local]
            isl["pinned"] = any(loop[uv_layer].pin_uv for face in faces for loop in face.loops)
            # An island counts as selected when all of its faces are: a half-selected
            # island is not one the user asked to move, and the tools that look at the
            # unselected islands (follow groups, "avoid unselected") need the same rule.
            isl["selected"] = all(face.select for face in faces)
            isl["object"] = obj
            isl["bm"] = bm
            isl["active"] = active_face is not None and active_face in faces
            # An island parameter belongs to the island, so the first face decides; the
            # operators write the same value to every face of the island anyway.
            first = faces[0]
            isl["bundle"] = first[bundle_layer] if bundle_layer is not None else DEFAULT_BUNDLE
            isl["follow_group"] = first[follow_layer] if follow_layer is not None else DEFAULT_BUNDLE
            if with_geometry:
                isl["face_uvs"] = [list(faces_uv[f]) for f in local]
                isl["positions"] = [list(positions[f]) for f in local]
                isl["verts"] = [list(uv) for f in local for uv in faces_uv[f]]
            if method == "MANUAL":
                nums = Counter(face[manual] for face in faces) if manual is not None else Counter({0: 1})
                isl["group"] = int(nums.most_common(1)[0][0])
            elif keys:
                names = Counter(keys[(obj.name, face.index)][0] for face in faces)
                isl["group"] = int(key_to_num.get(names.most_common(1)[0][0], 0))
            else:
                isl["group"] = 0
            if st.keep_pinned and isl["pinned"]:
                pinned_islands.append(isl)
            elif st.only_selected and not isl["selected"]:
                other_islands.append(isl)
            else:
                packed.append(isl)

    # "Include Overlapping Unselected": unselected islands that overlap the selection
    # join the pack rather than standing in its way.
    if st.others_mode == "REPACK" and other_islands and packed:
        selected_boxes = [_bbox(i) for i in packed]
        keep = []
        for isl in other_islands:
            box = _bbox(isl)
            if any(_boxes_overlap(box, b) for b in selected_boxes):
                packed.append(isl)
            else:
                keep.append(isl)
        other_islands = keep

    for index, isl in enumerate(packed):
        isl["id"] = index
    got.islands = packed
    # The frozen islands are numbered after the packed ones: the engine never sees them,
    # but the follow groups have to name them, and the two sets must not collide.
    for index, isl in enumerate(pinned_islands + other_islands):
        isl["id"] = len(packed) + index

    if st.keep_pinned and st.pinned_as_obstacles:
        for isl in pinned_islands:
            got.obstacles.extend(island_polygons(isl))
    if st.others_mode in ("OBSTACLES", "REPACK"):
        for isl in other_islands:
            got.obstacles.extend(island_polygons(isl))
    got.frozen_islands = pinned_islands + other_islands
    return got


def gather_islands(context, only_selected=True, split_at_vertices=False):
    """(state, problem) for the meshes in edit mode: the islands to pack, the BMFace
    that owns each face's UV loops, and the UVs those loops started with. `state` is
    None when there is nothing to pack, and `problem` says why.

    A module-level function rather than an operator method so a script (and the
    headless test) can build the same state the operator works on without an
    operator instance to hang properties off."""
    st = SimpleSettings(only_selected=only_selected, split_at_vertices=split_at_vertices)
    got = collect(context, st)
    if not got.meshes:
        return None, "No mesh in edit mode"
    if not got.owners:
        return None, "No faces to pack"
    if not got.islands:
        return None, "No islands could be extracted from the selected faces"
    return state_from(got, requests=[], targets=[[0.0, 0.0, 1.0, 1.0]]), ""


def state_from(got, requests, targets, st=None, rule=None, used_sets=None):
    """The plain dict every later step works on. A dict rather than the Collected object
    so the streaming path can carry the requests and the bookkeeping in one place."""
    return {"owners": got.owners, "meshes": got.meshes, "islands": got.islands,
            "original": got.original, "obstacles": got.obstacles,
            "frozen_islands": got.frozen_islands, "frozen": got.frozen,
            "requests": list(requests), "request": requests[0] if requests else None,
            "targets": [list(t) for t in targets], "collected": got,
            "st": st, "rule": rule, "used_sets": used_sets,
            # Filled by the matching step: the alignment a stacked island is packed
            # with, the geometry that was actually sent, and the islands that follow one.
            "pre": {}, "sent": {}, "followers": [], "bundles": None, "followed": 0}


def gather(context, split_at_vertices=False, only_selected=True, with_geometry=False):
    """The islands of the edit-mode meshes as the UV tools want them.

    `only_selected` keeps the packer's rule: with it on, only the fully selected islands
    are handed back. With it off, every visible island is, each flagged `selected` --
    which is what Select Matching Islands and Follow Groups need, since they have to see
    the unselected ones."""
    st = SimpleSettings(only_selected=only_selected, split_at_vertices=split_at_vertices)
    return collect(context, st, with_geometry=with_geometry)


def apply_transform(got, island, transform):
    """Push a uv_similarity.Transform onto every loop of the island."""
    for f in island["faces"]:
        face, uv_layer = got.owners[f]
        for loop in face.loops:
            loop[uv_layer].uv = transform.apply(tuple(loop[uv_layer].uv))


def apply_vertices(got, island, corrected):
    """Write the per-vertex UVs "correct vertices" produced. `corrected` is parallel to
    uv_similarity.island_vertices(island), so the original UV is the key."""
    grid = uv_similarity.GRID
    lookup = {}
    for source, target in zip(uv_similarity.island_vertices(island), corrected):
        lookup[(round(source[0] / grid), round(source[1] / grid))] = target
    for f in island["faces"]:
        face, uv_layer = got.owners[f]
        for loop in face.loops:
            u, v = loop[uv_layer].uv
            target = lookup.get((round(u / grid), round(v / grid)))
            if target is not None:
                loop[uv_layer].uv = (target[0], target[1])


def reference_island(islands):
    """The island the stacking operators put the others onto: the active face's island
    when there is one, else the first."""
    for isl in islands:
        if isl.get("active"):
            return isl
    return islands[0] if islands else None


def select_faces(got, island_ids):
    """Select exactly the faces of these islands, for the 'show me what failed' path.
    With UV select sync on (Blender's default) this is the UV selection as well."""
    wanted = set(island_ids)
    for face, _ in got.owners:
        face.select = False
    for isl in got.islands:
        if isl["id"] in wanted:
            for f in isl["faces"]:
                got.owners[f][0].select = True
    for obj, bm in got.meshes:
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data)


class MatchMixin:
    """The match parameters shared by the pack operator and the matching tools."""

    match_mode: bpy.props.EnumProperty(name="Mode", items=MATCH_MODE_ITEMS, default="outline")
    match_threshold: bpy.props.FloatProperty(name="Threshold", default=0.1, min=0.0, precision=3)
    resolution: bpy.props.IntProperty(name="Resolution", default=500, min=10, max=10000)
    match_check_holes: bpy.props.BoolProperty(name="Check holes", default=False)
    match_adjust_scale: bpy.props.BoolProperty(name="Adjust scale", default=False)
    match_nonuniform_tolerance: bpy.props.FloatProperty(
        name="Non-uniform scaling tolerance", default=0.0, min=0.0, max=1.0)
    match_correct_vertices: bpy.props.BoolProperty(name="Correct vertices", default=False)
    match_vertex_threshold: bpy.props.FloatProperty(name="Vertex threshold", default=0.01,
                                                    min=0.0, max=0.05, precision=4)
    match_mirroring: bpy.props.BoolProperty(name="Allow mirroring", default=False)

    def match_params(self):
        return uv_similarity.MatchParams(
            mode=self.match_mode, threshold=self.match_threshold, resolution=self.resolution,
            check_holes=self.match_check_holes, allow_mirroring=self.match_mirroring,
            adjust_scale=self.match_adjust_scale,
            non_uniform_scaling_tolerance=self.match_nonuniform_tolerance,
            correct_vertices=self.match_correct_vertices,
            vertex_threshold=self.match_vertex_threshold)

    def needs_geometry(self):
        return self.match_mode != "outline" or self.match_correct_vertices
