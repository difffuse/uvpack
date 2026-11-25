# Target regions: the rectangles in UV space a pack is allowed to fill, the operators
# that set them (by number, from a UDIM tile, by dragging one in the UV editor) and the
# overlay that draws them.
import math

import bpy

from .props import (settings, tag_redraw, addon_prefs, uv_editor_image, tile_xy,
                    SET_REGION_MODES)
from .sets import active_set


def udim_tiles(context):
    """(x, y) of every tile of the image shown in the UV editor; [(0, 0)] if there is none."""
    image = uv_editor_image(context)
    tiles = [tile_xy(t.number) for t in getattr(image, "tiles", ())] if image is not None else []
    return tiles or [(0, 0)]


def build_targets(st, context, rule=None, used_sets=None):
    """(targets, groups) for the current pack mode.

    `targets` is the flat list the engine takes; `groups` maps a set number to the
    indices of the targets it may use. `used_sets` restricts the set regions to the
    sets that actually carry islands, so an unused set cannot claim a tile."""
    base = st.custom_region.as_list() if st.use_custom_region else [0.0, 0.0, 1.0, 1.0]
    mode = st.pack_mode

    if mode == "ONE_TILE":
        return [base], {}

    if mode in ("TILE_GRID", "SETS_ADJACENT"):
        width, height = base[2] - base[0], base[3] - base[1]
        if st.tiles_from_udim:
            return [[base[0] + x * width, base[1] + y * height,
                     base[0] + (x + 1) * width, base[1] + (y + 1) * height]
                    for x, y in udim_tiles(context)], {}
        return [[base[0] + x * width, base[1] + y * height,
                 base[0] + (x + 1) * width, base[1] + (y + 1) * height]
                for y in range(st.tile_count_y) for x in range(st.tile_count_x)], {}

    # SETS_TO_TILES / SETS_OVERLAID: the flattened per-set regions, each set pinned to
    # its own. The engine honors the pinning inside an overlaid run too, so the same
    # region list serves both paths and the overlay draws it either way.
    targets, groups = [], {}
    if rule is None:
        return [base], {}
    for entry in rule.sets:
        if used_sets is not None and entry.num not in used_sets:
            continue
        indices = []
        for region in entry.regions:
            if not region.is_valid():
                continue
            indices.append(len(targets))
            targets.append(region.as_list())
        if indices:
            groups[str(entry.num)] = {"tiles": indices}
    if not targets:
        return [base], {}
    return targets, groups


# ---------------------------------------------------------------- region operators
def _resolve_region(context, which, create=False):
    st = settings(context)
    if which == "CUSTOM":
        return st.custom_region
    entry = active_set(st.rule())
    if entry is None:
        return None
    if not len(entry.regions):
        if not create:
            return None
        entry.ensure_region()
    return entry.regions[max(0, min(entry.active_region, len(entry.regions) - 1))]


_REGION_TARGET = [("CUSTOM", "Target Region", ""), ("SET", "Active Set Region", "")]


class UVPACK_OT_region_set(bpy.types.Operator):
    """Set a target region from numbers"""
    bl_idname = "uv.uvpack_region_set"
    bl_label = "Set Region"
    bl_options = {"REGISTER", "UNDO"}

    which: bpy.props.EnumProperty(name="Region", items=_REGION_TARGET, default="CUSTOM")
    min_x: bpy.props.FloatProperty(name="Min X", default=0.0, precision=4)
    min_y: bpy.props.FloatProperty(name="Min Y", default=0.0, precision=4)
    max_x: bpy.props.FloatProperty(name="Max X", default=1.0, precision=4)
    max_y: bpy.props.FloatProperty(name="Max Y", default=1.0, precision=4)

    def execute(self, context):
        region = _resolve_region(context, self.which, create=True)
        if region is None:
            self.report({"ERROR"}, "No region to set")
            return {"CANCELLED"}
        region.set_region(self.min_x, self.min_y, self.max_x, self.max_y)
        if not region.is_valid():
            self.report({"WARNING"}, "The region is smaller than 0.0001 UV units")
        tag_redraw()
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_region_from_tile(bpy.types.Operator):
    """Set a target region to one UDIM tile"""
    bl_idname = "uv.uvpack_region_from_tile"
    bl_label = "Region From Tile"
    bl_options = {"REGISTER", "UNDO"}

    which: bpy.props.EnumProperty(name="Region", items=_REGION_TARGET, default="CUSTOM")
    from_active: bpy.props.BoolProperty(
        name="From Active Tile", default=True,
        description="Use the active tile of the image shown in the UV editor")
    tile_x: bpy.props.IntProperty(name="Tile (X)", default=0, min=0, max=100)
    tile_y: bpy.props.IntProperty(name="Tile (Y)", default=0, min=0, max=100)

    def execute(self, context):
        region = _resolve_region(context, self.which, create=True)
        if region is None:
            self.report({"ERROR"}, "No region to set")
            return {"CANCELLED"}
        tx, ty = self.tile_x, self.tile_y
        if self.from_active:
            image = uv_editor_image(context)
            tiles = getattr(image, "tiles", None)
            active = getattr(tiles, "active", None) if tiles is not None else None
            if active is not None:
                tx, ty = tile_xy(active.number)
        region.set_tile(tx, ty)
        if self.which == "CUSTOM":
            settings(context).use_custom_region = True
        tag_redraw()
        self.report({"INFO"}, f"region set to tile {tx}:{ty}")
        return {"FINISHED"}


class UVPACK_OT_region_nudge(bpy.types.Operator):
    """Move a target region by its own size, one tile at a time"""
    bl_idname = "uv.uvpack_region_nudge"
    bl_label = "Nudge Region"
    bl_options = {"REGISTER", "UNDO"}

    which: bpy.props.EnumProperty(name="Region", items=_REGION_TARGET, default="CUSTOM")
    dir_x: bpy.props.IntProperty(name="X", default=0, min=-1, max=1)
    dir_y: bpy.props.IntProperty(name="Y", default=0, min=-1, max=1)

    def execute(self, context):
        region = _resolve_region(context, self.which)
        if region is None:
            return {"CANCELLED"}
        x0, y0, x1, y1 = region.as_list()
        dx, dy = self.dir_x * (x1 - x0), self.dir_y * (y1 - y0)
        region.set_region(x0 + dx, y0 + dy, x1 + dx, y1 + dy)
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_region_reset(bpy.types.Operator):
    """Reset a target region to the 0-1 tile"""
    bl_idname = "uv.uvpack_region_reset"
    bl_label = "Reset Region"
    bl_options = {"REGISTER", "UNDO"}

    which: bpy.props.EnumProperty(name="Region", items=_REGION_TARGET, default="CUSTOM")

    def execute(self, context):
        region = _resolve_region(context, self.which)
        if region is None:
            return {"CANCELLED"}
        region.set_region(0.0, 0.0, 1.0, 1.0)
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_set_region_add(bpy.types.Operator):
    """Add a target region to the active set"""
    bl_idname = "uv.uvpack_set_region_add"
    bl_label = "Add Region"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        entry = active_set(settings(context).rule())
        if entry is None:
            return {"CANCELLED"}
        # Start one tile to the right of the last region, which is where a free tile
        # usually is; the user moves it from there.
        if len(entry.regions):
            x0, y0, x1, y1 = entry.regions[-1].as_list()
            entry.regions.add().set_region(x0 + (x1 - x0), y0, x1 + (x1 - x0), y1)
        else:
            entry.regions.add().set_tile(0, 0)
        entry.active_region = len(entry.regions) - 1
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_set_region_remove(bpy.types.Operator):
    """Remove the active target region of the active set"""
    bl_idname = "uv.uvpack_set_region_remove"
    bl_label = "Remove Region"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        entry = active_set(settings(context).rule())
        if entry is None:
            return {"CANCELLED"}
        if len(entry.regions) <= 1:
            self.report({"ERROR"}, "A set has to have at least one target region")
            return {"CANCELLED"}
        entry.regions.remove(max(0, min(entry.active_region, len(entry.regions) - 1)))
        entry.active_region = max(0, min(entry.active_region, len(entry.regions) - 1))
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_region_draw(bpy.types.Operator):
    """Draw a target region in the UV editor: drag with the left mouse button, Esc finishes"""
    bl_idname = "uv.uvpack_region_draw"
    bl_label = "Draw Region"
    bl_options = {"REGISTER", "UNDO"}

    which: bpy.props.EnumProperty(name="Region", items=_REGION_TARGET, default="CUSTOM")
    snap: bpy.props.BoolProperty(name="Snap To Tiles", default=False,
                                 description="Hold Ctrl while dragging to snap to whole tiles")

    _start = None
    _region = None

    @classmethod
    def poll(cls, context):
        space = getattr(context, "space_data", None)
        return space is not None and space.type == "IMAGE_EDITOR"

    def _view(self, context, event):
        region = context.region
        x, y = region.view2d.region_to_view(event.mouse_region_x - region.x,
                                            event.mouse_region_y - region.y)
        return x, y

    def modal(self, context, event):
        context.area.tag_redraw()
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            _set_region_draw(None)
            return {"CANCELLED"}
        if event.type == "LEFTMOUSE":
            if event.value == "PRESS":
                self._start = self._view(context, event)
                return {"RUNNING_MODAL"}
            if event.value == "RELEASE" and self._start is not None:
                x0, y0 = self._start
                x1, y1 = self._view(context, event)
                if event.ctrl or self.snap:
                    x0, y0 = math.floor(x0), math.floor(y0)
                    x1, y1 = math.ceil(x1), math.ceil(y1)
                self._region.set_region(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
                self._start = None
                if not self._region.is_valid():
                    self.report({"WARNING"}, "the region was too small, drag a larger one")
                _set_region_draw(None)
                return {"FINISHED"}
        if event.type == "MOUSEMOVE" and self._start is not None:
            x0, y0 = self._start
            x1, y1 = self._view(context, event)
            self._region.set_region(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
            return {"RUNNING_MODAL"}
        return {"PASS_THROUGH"}

    def invoke(self, context, event):
        self._region = _resolve_region(context, self.which, create=True)
        if self._region is None:
            self.report({"ERROR"}, "No region to draw")
            return {"CANCELLED"}
        if self.which == "CUSTOM":
            settings(context).use_custom_region = True
        self._start = None
        _set_region_draw(self.which)
        enable_overlay()
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        # Non-interactive call (headless, or a script): nothing to drag, so just make
        # sure the region exists and let the numeric fields do the work.
        region = _resolve_region(context, self.which, create=True)
        return {"FINISHED"} if region is not None else {"CANCELLED"}


# ---------------------------------------------------------------- overlay
# Drawn with the gpu module in the UV editor. Everything here is optional and must
# survive a background Blender, which has no SpaceImage type at all and refuses to
# build a shader, so the handler is only ever installed when the type exists and the
# shader is built lazily inside the draw callback.
_draw_handle = None
_overlay_regions = []       # [(region as [x0,y0,x1,y1], (r, g, b), label)]
_region_draw_which = None


def overlay_supported():
    return hasattr(bpy.types, "SpaceImage")


def overlay_active():
    return _draw_handle is not None


def _set_region_draw(which):
    global _region_draw_which
    _region_draw_which = which
    tag_redraw()


def set_overlay_targets(targets, st, rule=None, used_sets=None):
    """Remember what the last pack aimed at, so the overlay shows the real regions."""
    global _overlay_regions
    regions = []
    if rule is not None and st.pack_mode in SET_REGION_MODES:
        for entry in rule.sets:
            if used_sets is not None and entry.num not in used_sets:
                continue
            for region in entry.regions:
                regions.append((region.as_list(), tuple(entry.color),
                                f"{entry.name}: {region.label()}"))
    else:
        for i, target in enumerate(targets):
            regions.append((list(target), (1.0, 0.85, 0.1), f"Tile {i}"))
    _overlay_regions = regions
    tag_redraw()


def current_overlay_regions(context):
    """The regions to draw right now: the live settings, so the user sees them before
    packing, falling back to whatever the last pack used."""
    st = settings(context)
    if st is None:
        return _overlay_regions
    rule = st.rule() if st.pack_mode in SET_REGION_MODES else None
    if rule is not None:
        return [(r.as_list(), tuple(s.color), f"{s.name}: {r.label()}")
                for s in rule.sets for r in s.regions]
    targets, _ = build_targets(st, context)
    return [(list(t), (1.0, 0.85, 0.1), f"Tile {i}") for i, t in enumerate(targets)]


def _draw_overlay():
    context = bpy.context
    st = settings(context)
    if st is None or not st.overlay_enable:
        return
    try:
        import gpu  # noqa: PLC0415
        from gpu_extras.batch import batch_for_shader  # noqa: PLC0415
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    except Exception:  # noqa: BLE001
        return
    prefs = addon_prefs(context)
    width = prefs.region_line_width if prefs is not None else 2.0
    try:
        gpu.state.blend_set("ALPHA")
        gpu.state.line_width_set(width)
        for region, color, _label in current_overlay_regions(context):
            x0, y0, x1, y1 = region
            corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            segments = []
            for i in range(4):
                segments.append(corners[i])
                segments.append(corners[(i + 1) % 4])
            batch = batch_for_shader(shader, "LINES", {"pos": segments})
            shader.uniform_float("color", (color[0], color[1], color[2], 0.9))
            batch.draw(shader)
    finally:
        try:
            gpu.state.line_width_set(1.0)
            gpu.state.blend_set("NONE")
        except Exception:  # noqa: BLE001
            pass


def enable_overlay():
    global _draw_handle
    if _draw_handle is not None or not overlay_supported():
        return False
    try:
        _draw_handle = bpy.types.SpaceImage.draw_handler_add(_draw_overlay, (), "WINDOW", "POST_VIEW")
    except Exception:  # noqa: BLE001
        _draw_handle = None
        return False
    tag_redraw()
    return True


def disable_overlay():
    global _draw_handle
    if _draw_handle is None:
        return
    try:
        bpy.types.SpaceImage.draw_handler_remove(_draw_handle, "WINDOW")
    except Exception:  # noqa: BLE001
        pass
    _draw_handle = None
    tag_redraw()


def overlay_sync(st):
    if st.overlay_enable:
        enable_overlay()
    else:
        disable_overlay()


class UVPACK_OT_overlay_toggle(bpy.types.Operator):
    """Show or hide the target regions in the UV editor"""
    bl_idname = "uv.uvpack_overlay_toggle"
    bl_label = "Toggle Region Overlay"
    bl_options = {"REGISTER"}

    def execute(self, context):
        st = settings(context)
        st.overlay_enable = not st.overlay_enable
        if st.overlay_enable and not overlay_supported():
            self.report({"WARNING"}, "no UV editor to draw in (background Blender)")
        return {"FINISHED"}


classes = (UVPACK_OT_region_set, UVPACK_OT_region_from_tile, UVPACK_OT_region_nudge,
           UVPACK_OT_region_reset, UVPACK_OT_set_region_add, UVPACK_OT_set_region_remove,
           UVPACK_OT_region_draw, UVPACK_OT_overlay_toggle)
