# Island sets: the rule that turns faces into sets (by material, object, mesh part,
# collection, colour attribute, or by hand), the sets it produces, and the operators
# the sidebar drives them with.
import bpy
import bmesh

from .props import (SET_ATTR, settings, tag_redraw, int_attribute, SET_RULE_METHODS)


def set_attribute(bm, create=False):
    return int_attribute(bm, SET_ATTR, create=create)


def mesh_part_of_face(bm):
    """Face index -> connected-component index, connectivity through mesh vertices.
    This is the 3D 'mesh part', not the UV island: a cube unwrapped into six pieces
    is one mesh part."""
    bm.faces.index_update()
    parent = list(range(len(bm.faces)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for vert in bm.verts:
        linked = vert.link_faces
        if len(linked) < 2:
            continue
        root = find(linked[0].index)
        for face in linked[1:]:
            other = find(face.index)
            if other != root:
                parent[other] = root
    labels, out = {}, {}
    for face in bm.faces:
        root = find(face.index)
        if root not in labels:
            labels[root] = len(labels)
        out[face.index] = labels[root]
    return out


def color_layer(obj, bm):
    """(bmesh layer, domain) of the object's active colour attribute.

    Read through bmesh, not through mesh.color_attributes[].data: the mesh datablock
    is not kept in sync while the object is in edit mode, so its attribute values are
    whatever they were when edit mode was entered."""
    name = None
    attributes = getattr(obj.data, "color_attributes", None)
    if attributes is not None and attributes.active_color is not None:
        name = attributes.active_color.name
    candidates = [("CORNER", bm.loops.layers), ("POINT", bm.verts.layers), ("FACE", bm.faces.layers)]
    fallback = (None, None)
    for domain, seq in candidates:
        for kind in ("color", "float_color"):
            layers = getattr(seq, kind, None)
            if layers is None or not len(layers):
                continue
            keys = list(layers.keys())
            if name is not None and name in keys:
                return layers[name], domain
            if fallback[0] is None:
                fallback = (layers[keys[0]], domain)
    return fallback


def _face_color(face, layer, domain):
    """The colour of a face under an arbitrary domain, quantized to 1/255 so two float
    colours that paint the same pixel land in the same set."""
    if layer is None:
        return None
    try:
        if domain == "POINT":
            value = face.verts[0][layer]
        elif domain == "FACE":
            value = face[layer]
        else:  # CORNER
            value = face.loops[0][layer]
    except (IndexError, KeyError, TypeError):
        return None
    return tuple(max(0, min(255, int(round(c * 255.0)))) for c in tuple(value)[:3])


def face_set_keys(objects_bms, method, part_offset=None):
    """[(object, bmesh)] -> {(object name, face index): (key, colour or None)}."""
    keys = {}
    part_base = 0
    for obj, bm in objects_bms:
        bm.faces.index_update()
        parts = mesh_part_of_face(bm) if method == "MESH_PART" else None
        layer, domain = color_layer(obj, bm) if method == "VERTEX_COLOR" else (None, None)
        slots = [s.name if s.name else f"Slot {i}" for i, s in enumerate(obj.material_slots)]
        collections = [c.name for c in obj.users_collection]
        for face in bm.faces:
            color = None
            if method == "MATERIAL":
                key = slots[face.material_index] if 0 <= face.material_index < len(slots) else "(no material)"
            elif method == "OBJECT":
                key = obj.name
            elif method == "MESH_PART":
                key = f"Mesh part {part_base + parts[face.index]}"
            elif method == "COLLECTION":
                key = collections[0] if collections else "(no collection)"
            elif method == "VERTEX_COLOR":
                rgb = _face_color(face, layer, domain)
                if rgb is None:
                    key = "(no colour)"
                else:
                    key = "C %d.%d.%d" % rgb
                    color = tuple(c / 255.0 for c in rgb)
            else:  # MANUAL
                key = None
            keys[(obj.name, face.index)] = (key, color)
        if parts is not None:
            part_base += len(set(parts.values()))
    return keys


def sync_rule_sets(rule, keys):
    """Create a set for every key the mesh shows, keeping the ones already there.
    Returns {key: set number}."""
    order = []
    seen = set()
    for key, color in keys.values():
        if key is not None and key not in seen:
            seen.add(key)
            order.append((key, color))
    order.sort(key=lambda kc: kc[0])
    mapping = {}
    for key, color in order:
        entry = rule.set_by_key(key)
        if entry is None:
            entry = rule.add_set(name=key, key=key, color=color)
        elif color is not None:
            entry.color = color
        mapping[key] = entry.num
    return mapping


def arrange_set_regions(rule):
    """Lay out one region per tile of every set according to the layout mode."""
    if rule.layout_mode == "MANUAL":
        return
    tile_index = 0
    for row, entry in enumerate(rule.sets):
        tiles = []
        if rule.layout_mode == "AUTOMATIC":
            for _ in range(entry.tile_count):
                tiles.append((tile_index % rule.tiles_per_row, tile_index // rule.tiles_per_row))
                tile_index += 1
        elif rule.layout_mode == "HORIZONTAL":
            tiles = [(t, row) for t in range(entry.tile_count)]
        elif rule.layout_mode == "VERTICAL":
            tiles = [(row, t) for t in range(entry.tile_count)]
        else:  # GRID: every set gets the same grid
            tiles = [(x, y) for y in range(rule.tile_count_y) for x in range(rule.tile_count_x)]
        entry.regions.clear()
        for tx, ty in tiles or [(0, 0)]:
            entry.regions.add().set_tile(tx, ty)
        entry.active_region = 0


def active_set(rule):
    if rule is None or not len(rule.sets):
        return None
    return rule.sets[max(0, min(rule.active_set, len(rule.sets) - 1))]


# ---------------------------------------------------------------- operators
class UVPACK_OT_rule_add(bpy.types.Operator):
    """Add a set rule"""
    bl_idname = "uv.uvpack_rule_add"
    bl_label = "Add Set Rule"
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Name", default="Set Rule")
    method: bpy.props.EnumProperty(name="Method", items=SET_RULE_METHODS, default="MATERIAL")

    def execute(self, context):
        st = settings(context)
        rule = st.rules.add()
        rule.name = self.name or "Set Rule"
        rule.method = self.method
        rule.add_set(name="Default")
        st.active_rule = len(st.rules) - 1
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_rule_remove(bpy.types.Operator):
    """Remove the active set rule"""
    bl_idname = "uv.uvpack_rule_remove"
    bl_label = "Remove Set Rule"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        st = settings(context)
        if not len(st.rules):
            return {"CANCELLED"}
        st.rules.remove(max(0, min(st.active_rule, len(st.rules) - 1)))
        st.active_rule = max(0, min(st.active_rule, len(st.rules) - 1))
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_rule_rebuild(bpy.types.Operator):
    """Rebuild the active rule's sets from the mesh and lay their target regions out"""
    bl_idname = "uv.uvpack_rule_rebuild"
    bl_label = "Rebuild Sets"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        st = settings(context)
        rule = st.rule()
        if rule is None:
            self.report({"ERROR"}, "No set rule")
            return {"CANCELLED"}
        if rule.method == "MANUAL":
            arrange_set_regions(rule)
            self.report({"INFO"}, f"{len(rule.sets)} manual sets, regions laid out")
            tag_redraw()
            return {"FINISHED"}
        pairs = []
        for obj in context.objects_in_mode_unique_data:
            if obj.type != "MESH":
                continue
            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            pairs.append((obj, bm))
        keys = face_set_keys(pairs, rule.method)
        before = len(rule.sets)
        sync_rule_sets(rule, keys)
        arrange_set_regions(rule)
        self.report({"INFO"}, f"{len(rule.sets)} sets ({len(rule.sets) - before} new)")
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_set_add(bpy.types.Operator):
    """Add a set to the active rule"""
    bl_idname = "uv.uvpack_set_add"
    bl_label = "Add Set"
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Name", default="")

    def execute(self, context):
        st = settings(context)
        rule = st.rule()
        if rule is None:
            self.report({"ERROR"}, "No set rule")
            return {"CANCELLED"}
        rule.add_set(name=self.name or None)
        rule.active_set = len(rule.sets) - 1
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_set_remove(bpy.types.Operator):
    """Remove the active set"""
    bl_idname = "uv.uvpack_set_remove"
    bl_label = "Remove Set"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        st = settings(context)
        rule = st.rule()
        if rule is None or len(rule.sets) <= 1:
            self.report({"ERROR"}, "A set rule needs at least one set")
            return {"CANCELLED"}
        rule.sets.remove(max(0, min(rule.active_set, len(rule.sets) - 1)))
        rule.active_set = max(0, min(rule.active_set, len(rule.sets) - 1))
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_set_assign(bpy.types.Operator):
    """Assign the selected faces to the active set of the active rule"""
    bl_idname = "uv.uvpack_set_assign"
    bl_label = "Assign To Set"
    bl_options = {"REGISTER", "UNDO"}

    set_num: bpy.props.IntProperty(name="Set", default=-1, options={"SKIP_SAVE"},
                                   description="-1 uses the rule's active set")

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        st = settings(context)
        rule = st.rule()
        if rule is None or not len(rule.sets):
            self.report({"ERROR"}, "No set rule with sets")
            return {"CANCELLED"}
        num = self.set_num
        if num < 0:
            num = active_set(rule).num
        assigned = 0
        for obj in context.objects_in_mode_unique_data:
            if obj.type != "MESH":
                continue
            bm = bmesh.from_edit_mesh(obj.data)
            layer = set_attribute(bm, create=True)
            for face in bm.faces:
                if face.select and not face.hide:
                    face[layer] = num
                    assigned += 1
            bmesh.update_edit_mesh(obj.data)
        self.report({"INFO"}, f"{assigned} faces assigned to set {num}")
        return {"FINISHED"}


class UVPACK_OT_set_select(bpy.types.Operator):
    """Select the faces assigned to the active set"""
    bl_idname = "uv.uvpack_set_select"
    bl_label = "Select Set"
    bl_options = {"REGISTER", "UNDO"}

    deselect: bpy.props.BoolProperty(name="Deselect", default=False)

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        st = settings(context)
        rule = st.rule()
        if rule is None or not len(rule.sets):
            return {"CANCELLED"}
        num = active_set(rule).num
        touched = 0
        for obj in context.objects_in_mode_unique_data:
            if obj.type != "MESH":
                continue
            bm = bmesh.from_edit_mesh(obj.data)
            layer = set_attribute(bm)
            if layer is None:
                continue
            for face in bm.faces:
                if not face.hide and face[layer] == num:
                    face.select = not self.deselect
                    touched += 1
            bm.select_flush_mode()
            bmesh.update_edit_mesh(obj.data)
        self.report({"INFO"}, f"{touched} faces in set {num}")
        return {"FINISHED"}


class UVPACK_OT_set_regions(bpy.types.Operator):
    """Lay the target regions of every set out again"""
    bl_idname = "uv.uvpack_set_regions"
    bl_label = "Arrange Set Regions"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        rule = settings(context).rule()
        if rule is None:
            return {"CANCELLED"}
        arrange_set_regions(rule)
        tag_redraw()
        return {"FINISHED"}


classes = (UVPACK_OT_rule_add, UVPACK_OT_rule_remove, UVPACK_OT_rule_rebuild,
           UVPACK_OT_set_add, UVPACK_OT_set_remove, UVPACK_OT_set_assign,
           UVPACK_OT_set_select, UVPACK_OT_set_regions)
