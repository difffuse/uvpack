# The UV tools built on uv_similarity: stacking matching islands, selecting them,
# spreading overlapping ones apart again, writing the per-island bundle and follow
# numbers, making unselected islands follow a packed one, measuring islands, and
# turning an island so a world axis points a given way in UV space.
import bpy
import bmesh

from . import uv_similarity
from .props import (BUNDLE_ATTR, FOLLOW_ATTR, DEFAULT_BUNDLE, SPREAD_OFFSET_ATTRS,
                    int_attribute, ensure_int_attributes)
from .collect import (gather, apply_transform, apply_vertices, reference_island,
                      MatchMixin)


class UVPACK_OT_stack_matching(bpy.types.Operator, MatchMixin):
    """Stack every selected island that matches the active one exactly on top of it."""
    bl_idname = "uv.uvpack_stack_matching"
    bl_label = "Stack Matching Islands"
    bl_options = {"REGISTER", "UNDO"}

    use_bundles: bpy.props.BoolProperty(
        name="Use bundles", default=False,
        description="Stack within each bundle instead of onto the active island")

    last_stacked = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        got = gather(context, only_selected=True, with_geometry=True)
        islands = got.islands
        if len(islands) < 2:
            self.report({"WARNING"}, "need at least two selected islands")
            return {"CANCELLED"}
        params = self.match_params()
        stacked = 0
        by_id = {isl["id"]: isl for isl in islands}
        if self.use_bundles:
            bundle_ids = {isl["id"]: isl.get("bundle", DEFAULT_BUNDLE) for isl in islands}
            result = uv_similarity.bundles(islands, bundle_ids, params=params,
                                           default_bundle=DEFAULT_BUNDLE)
            for island_id, transform in result.transforms.items():
                apply_transform(got, by_id[island_id], transform)
                stacked += 1
            stacked -= len(result.bundles)   # the leaders did not move
        else:
            reference = reference_island(islands)
            result = uv_similarity.stack_matching(islands, reference, params=params,
                                                  correct_vertices=params.correct_vertices)
            for island_id, transform in result.transforms.items():
                if island_id in result.vertices:
                    apply_vertices(got, by_id[island_id], result.vertices[island_id])
                else:
                    apply_transform(got, by_id[island_id], transform)
                stacked += 1
        got.update()
        type(self).last_stacked = stacked
        self.report({"INFO"}, f"stacked {stacked} islands")
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_select_matching(bpy.types.Operator, MatchMixin):
    """Select every unselected island that matches at least one selected island."""
    bl_idname = "uv.uvpack_select_matching"
    bl_label = "Select Matching Islands"
    bl_options = {"REGISTER", "UNDO"}

    last_found = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        got = gather(context, only_selected=False, with_geometry=True)
        references = got.selected()
        candidates = got.unselected()
        if not references:
            self.report({"WARNING"}, "no island is selected")
            return {"CANCELLED"}
        params = self.match_params()
        found = uv_similarity.select_matching(candidates, references, params=params)
        for index in found:
            for f in candidates[index]["faces"]:
                face, _ = got.owners[f]
                face.select_set(True)
        for _, bm in got.meshes:
            bm.select_flush(True)
        got.update()
        type(self).last_found = len(found)
        self.report({"INFO"}, f"selected {len(found)} matching islands")
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_spread(bpy.types.Operator):
    """Move overlapping islands apart along +U, in whole tiles."""
    bl_idname = "uv.uvpack_spread"
    bl_label = "Spread Overlapping Islands"
    bl_options = {"REGISTER", "UNDO"}

    step: bpy.props.FloatProperty(name="Step (UV)", default=1.0, min=0.0001,
                                  description="How far one spread step moves an island. "
                                              "1.0 is one UDIM tile")
    max_tile_x: bpy.props.IntProperty(name="Max tile (X)", default=0, min=0,
                                      description="Wrap into the next row of tiles after "
                                                  "this many steps; 0 walks along +U forever")
    detection_mode: bpy.props.EnumProperty(
        name="Detection", items=[("touching", "Touching", ""), ("identical", "Identical", "")],
        default="touching")

    last_moved = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        ensure_int_attributes(context, SPREAD_OFFSET_ATTRS)
        got = gather(context, only_selected=True)
        if not got.islands:
            self.report({"WARNING"}, "no islands selected")
            return {"CANCELLED"}
        offsets = uv_similarity.spread_overlapping(got.islands, gap=self.step,
                                                   detection_mode=self.detection_mode,
                                                   max_tile_x=self.max_tile_x)
        moved = 0
        for isl in got.islands:
            du, dv = offsets.get(isl["id"], (0.0, 0.0))
            if du == 0.0 and dv == 0.0:
                continue
            apply_transform(got, isl, uv_similarity.Transform.translation(du, dv))
            _store_spread_offset(got, isl, du / self.step, dv / self.step)
            moved += 1
        got.update()
        type(self).last_moved = moved
        self.report({"INFO"}, f"moved {moved} islands")
        return {"FINISHED"}


def _store_spread_offset(got, island, du, dv):
    """Remember the spread so Undo Spread can put the island back. Attributes are
    per-mesh, so the island's own bmesh has to supply them."""
    layers = [int_attribute(island["bm"], name) for name in SPREAD_OFFSET_ATTRS]
    values = (int(round(du)), int(round(dv)))
    for f in island["faces"]:
        face, _ = got.owners[f]
        for layer, value in zip(layers, values):
            face[layer] = face[layer] + value


class UVPACK_OT_undo_spread(bpy.types.Operator):
    """Move every selected island back by the spread offset stored on it."""
    bl_idname = "uv.uvpack_undo_spread"
    bl_label = "Undo Spread"
    bl_options = {"REGISTER", "UNDO"}

    step: bpy.props.FloatProperty(name="Step (UV)", default=1.0, min=0.0001)

    last_moved = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        got = gather(context, only_selected=True)
        moved = 0
        for isl in got.islands:
            layers = [int_attribute(isl["bm"], name, create=False) for name in SPREAD_OFFSET_ATTRS]
            if any(layer is None for layer in layers):
                continue
            face, _ = got.owners[isl["faces"][0]]
            du, dv = face[layers[0]], face[layers[1]]
            if du == 0 and dv == 0:
                continue
            apply_transform(got, isl, uv_similarity.Transform.translation(-du * self.step,
                                                                         -dv * self.step))
            for f in isl["faces"]:
                owner, _ = got.owners[f]
                owner[layers[0]] = 0
                owner[layers[1]] = 0
            moved += 1
        got.update()
        type(self).last_moved = moved
        self.report({"INFO"}, f"moved {moved} islands back")
        return {"FINISHED"}


class UVPACK_OT_set_bundle(bpy.types.Operator):
    """Write a per-island number (bundle, follow group) onto the selected faces."""
    bl_idname = "uv.uvpack_set_bundle"
    bl_label = "Set Bundle"
    bl_options = {"REGISTER", "UNDO"}

    param: bpy.props.EnumProperty(
        name="Parameter",
        items=[("bundle", "Bundle", "Islands sharing a bundle are stacked and packed as one"),
               ("follow_group", "Follow group",
                "Unselected islands follow the selected island of the same number")],
        default="bundle")
    value: bpy.props.IntProperty(name="Number", default=1, min=0, max=1000)

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        name = BUNDLE_ATTR if self.param == "bundle" else FOLLOW_ATTR
        faces = 0
        objects = [o for o in context.objects_in_mode_unique_data if o.type == "MESH"]
        for obj in objects:
            bm = bmesh.from_edit_mesh(obj.data)
            layer = int_attribute(bm, name)
            touched = False
            for face in bm.faces:
                if face.hide or not face.select:
                    continue
                face[layer] = self.value
                faces += 1
                touched = True
            if touched:
                bmesh.update_edit_mesh(obj.data)
        self.report({"INFO"}, f"{name} = {self.value} on {faces} faces")
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_follow(bpy.types.Operator):
    """Pack the selected islands and make the unselected followers follow their leaders."""
    bl_idname = "uv.uvpack_follow"
    bl_label = "Follow Groups"
    bl_options = {"REGISTER", "UNDO"}

    margin: bpy.props.FloatProperty(name="Margin (UV)", default=0.003, min=0.0, precision=4)
    time_budget: bpy.props.FloatProperty(name="Search time (s)", default=2.0, min=0.0)
    rotation_step: bpy.props.IntProperty(name="Rotation step", default=90, min=0, max=180)
    cli_path: bpy.props.StringProperty(name="uvpack_cli path (override)", subtype="FILE_PATH",
                                       default="", options={"SKIP_SAVE"})

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        # Not the modal path: this is a one-shot that has to be finished by the time it
        # returns, because the caller reads the result off the pack operator.
        return bpy.ops.uv.uvpack_pack("EXEC_DEFAULT", margin=self.margin,
                                      time_budget=self.time_budget,
                                      rotation_step=self.rotation_step,
                                      follow_groups=True, realtime=False,
                                      cli_path=self.cli_path)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_island_metrics(bpy.types.Operator):
    """Report the UV area, the 3D area and the texel density of the selected islands."""
    bl_idname = "uv.uvpack_island_metrics"
    bl_label = "Island Metrics"
    bl_options = {"REGISTER"}

    tex_size: bpy.props.IntProperty(name="Texture size", default=1024, min=0)
    per_island: bpy.props.BoolProperty(name="Report per island", default=False)

    last_report = None

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        got = gather(context, only_selected=True)
        if not got.islands:
            self.report({"WARNING"}, "no islands selected")
            return {"CANCELLED"}
        scale_length = context.scene.unit_settings.scale_length or 1.0
        report = uv_similarity.island_metrics(got.islands, tex_size=self.tex_size,
                                              scale_length=scale_length)
        type(self).last_report = report
        density = report["texel_density"]
        self.report({"INFO"}, "{} islands: UV area {:.6f}, 3D area {:.6f}{}".format(
            len(report["islands"]), report["uv_area"], report["area_3d"],
            ", texel density {:.1f} px/unit".format(density) if density else ""))
        if self.per_island:
            for entry in report["islands"]:
                print("island {}: uv {:.6f} 3d {:.6f} td {}".format(
                    entry["id"], entry["uv_area"], entry["area_3d"], entry["texel_density"]))
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_align_world(bpy.types.Operator):
    """Rotate each selected island so a world axis, projected onto its surface, points
    a given way in UV space."""
    bl_idname = "uv.uvpack_align_world"
    bl_label = "Align To World Axis"
    bl_options = {"REGISTER", "UNDO"}

    axis: bpy.props.EnumProperty(name="3D axis", default="+Z",
                                 items=[(a, a, "") for a in ("+X", "-X", "+Y", "-Y", "+Z", "-Z")])
    uv_axis: bpy.props.EnumProperty(name="UV axis", default="+V",
                                    items=[("+U", "+U", ""), ("-U", "-U", ""),
                                           ("+V", "+V", ""), ("-V", "-V", "")])
    space: bpy.props.EnumProperty(name="Coordinate space", default="local",
                                  items=[("local", "Local", ""), ("global", "Global", "")])

    last_aligned = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        got = gather(context, only_selected=True, with_geometry=True)
        target = {"+U": (1.0, 0.0), "-U": (-1.0, 0.0), "+V": (0.0, 1.0), "-V": (0.0, -1.0)}[self.uv_axis]
        aligned, skipped = 0, 0
        for isl in got.islands:
            uvs = [uv for face in isl["face_uvs"] for uv in face]
            positions = [p for face in isl["positions"] for p in face]
            if self.space == "global":
                matrix = isl["object"].matrix_world
                positions = [tuple(matrix @ _vector(p)) for p in positions]
            try:
                transform = uv_similarity.align_to_world(uvs, positions, up_axis=self.axis,
                                                         uv_axis=target)
            except ValueError:
                skipped += 1
                continue
            apply_transform(got, isl, transform)
            aligned += 1
        got.update()
        type(self).last_aligned = aligned
        message = f"aligned {aligned} islands"
        if skipped:
            message += f" ({skipped} skipped: the axis is perpendicular to their surface)"
        self.report({"INFO"}, message)
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


def _vector(triple):
    from mathutils import Vector  # noqa: PLC0415  (only needed for the global space option)
    return Vector(triple)


classes = (UVPACK_OT_stack_matching, UVPACK_OT_select_matching, UVPACK_OT_spread,
           UVPACK_OT_undo_spread, UVPACK_OT_set_bundle, UVPACK_OT_follow,
           UVPACK_OT_island_metrics, UVPACK_OT_align_world)
