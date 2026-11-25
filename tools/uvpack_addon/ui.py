# The UV editor sidebar: one panel per group of options, plus the two lists the island
# sets and their target regions are edited in.
import bpy

from .props import (settings, resolve_tex_size, resolve_texture_aspect,
                    requested_margin, SET_MODES)
from .sets import active_set
from .regions import current_overlay_regions
from .presets import BUILTIN_PRESETS, preset_dir, preset_files
from .engine import UVPACK_OT_pack


class UVPACK_UL_sets(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_prop, index):
        row = layout.row(align=True)
        row.prop(item, "color", text="")
        row.prop(item, "name", text="", emboss=False)
        row.label(text=f"#{item.num}")
        row.label(text=f"{len(item.regions)} region" + ("s" if len(item.regions) != 1 else ""))


class UVPACK_UL_regions(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_prop, index):
        layout.label(text=f"Region {index}: {item.label()}")


class _Panel:
    bl_space_type = "IMAGE_EDITOR"
    bl_region_type = "UI"
    bl_category = "UVPack"

    @classmethod
    def poll(cls, context):
        space = context.space_data
        return space is not None and getattr(space, "mode", "UV") == "UV" and settings(context) is not None


class UVPACK_PT_main(_Panel, bpy.types.Panel):
    bl_label = "UVPack"
    bl_idname = "UVPACK_PT_main"

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("uv.uvpack_pack_scene", text="Pack", icon="MOD_UVPROJECT")
        row = layout.row(align=True)
        row.operator("uv.uvpack_pack_scene", text="Avoid Unselected").others_mode = "OBSTACLES"
        row.operator("uv.uvpack_pack_scene", text="Include Overlapping").others_mode = "REPACK"
        layout.prop(st, "pack_mode")
        col = layout.column(align=True)
        col.prop(st, "only_selected")
        col.prop(st, "others_mode")
        col.prop(st, "split_at_vertices")
        box = layout.box()
        box.label(text="Pinned UVs:")
        box.prop(st, "keep_pinned")
        sub = box.row()
        sub.enabled = st.keep_pinned
        sub.prop(st, "pinned_as_obstacles")
        if UVPACK_OT_pack.last_layout is not None:
            layout.label(text=f"Last scale: {UVPACK_OT_pack.last_scale:.4f}")


class UVPACK_PT_tiles(_Panel, bpy.types.Panel):
    bl_label = "Tiles"
    bl_parent_id = "UVPACK_PT_main"

    @classmethod
    def poll(cls, context):
        st = settings(context)
        return _Panel.poll(context) and st.pack_mode in ("TILE_GRID", "SETS_ADJACENT", "SETS_OVERLAID")

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "tiles_from_udim")
        col = layout.column(align=True)
        col.enabled = not st.tiles_from_udim
        col.prop(st, "tile_count_x")
        col.prop(st, "tile_count_y")
        row = layout.row()
        row.enabled = st.pack_mode != "SETS_ADJACENT"
        row.prop(st, "tile_fill")


class UVPACK_PT_layout(_Panel, bpy.types.Panel):
    bl_label = "Layout"
    bl_parent_id = "UVPACK_PT_main"

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "scale_mode")
        sub = layout.column(align=True)
        sub.enabled = st.scale_mode != "max"
        sub.prop(st, "fixed_scale")
        sub.prop(st, "allow_partial")
        row = sub.row()
        row.enabled = st.allow_partial
        row.prop(st, "shelve_unplaced")
        layout.prop(st, "bind_overlapping")
        layout.separator()
        layout.prop(st, "fill")
        layout.prop(st, "origin")


class UVPACK_PT_density(_Panel, bpy.types.Panel):
    bl_label = "Texel Density"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "equalize_density")
        if st.equalize_density and st.scale_mode != "max":
            layout.label(text="Islands packed at a fixed scale are not equalized", icon="INFO")
        layout.prop(st, "density_space")
        layout.prop(st, "texel_density")
        if st.texel_density > 0:
            layout.label(text=f"Pre-scaled to {st.texel_density:.0f} px/unit on a "
                              f"{resolve_tex_size(st, context)} px texture, then packed at scale 1",
                         icon="INFO")
        layout.prop(st, "scale_policy")
        layout.prop(st, "shrink_spanning")
        if st.shrink_spanning and st.search_time <= 0:
            layout.label(text="Shrinking spanning islands needs a search time above 0", icon="ERROR")


class UVPACK_PT_margins(_Panel, bpy.types.Panel):
    bl_label = "Margins"
    bl_parent_id = "UVPACK_PT_main"

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "use_pixel_margin")
        if st.use_pixel_margin:
            col = layout.column(align=True)
            col.prop(st, "pixel_margin")
            col.prop(st, "tex_size_from_image")
            sub = col.row()
            sub.enabled = not st.tex_size_from_image
            sub.prop(st, "tex_size")
            layout.label(text=f"Effective size: {resolve_tex_size(st, context)} px, "
                              f"gap {requested_margin(st, context):.5f} UV")
        else:
            layout.prop(st, "margin")
        layout.prop(st, "use_border_margin")
        sub = layout.row()
        sub.enabled = st.use_border_margin
        sub.prop(st, "pixel_border_margin" if st.use_pixel_margin else "border_margin")
        layout.separator()
        layout.prop(st, "snap_to_texels")
        col = layout.column(align=True)
        col.enabled = st.snap_to_texels
        col.prop(st, "snap_mode")
        col.prop(st, "snap_target")
        if st.snap_to_texels and not st.use_pixel_margin:
            layout.label(text=f"Snaps to a {resolve_tex_size(st, context)} px grid", icon="INFO")


class UVPACK_PT_rotation(_Panel, bpy.types.Panel):
    bl_label = "Rotation"
    bl_parent_id = "UVPACK_PT_main"

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "allow_rotation")
        col = layout.column(align=True)
        col.enabled = st.allow_rotation
        col.prop(st, "rotation_step")
        col.prop(st, "prealign")
        layout.prop(st, "allow_mirroring")


class UVPACK_PT_search(_Panel, bpy.types.Panel):
    bl_label = "Search"
    bl_parent_id = "UVPACK_PT_main"

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "resolution")
        layout.label(text=f"Stripe height: {1.0 / max(1, st.resolution):.5f}")
        layout.prop(st, "search_time")
        sub = layout.row()
        sub.enabled = st.search_time > 0
        sub.prop(st, "stall_timeout")
        layout.prop(st, "trials")
        layout.prop(st, "threads")
        layout.separator()
        layout.prop(st, "compaction_time")
        sub = layout.row()
        sub.enabled = st.compaction_time > 0
        sub.prop(st, "compaction_rotation")
        layout.separator()
        layout.prop(st, "stream")
        layout.label(text="ESC keeps the best layout found so far", icon="INFO")


class UVPACK_PT_aspect(_Panel, bpy.types.Panel):
    bl_label = "Texture Aspect"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "use_texture_aspect")
        col = layout.column(align=True)
        col.enabled = st.use_texture_aspect
        col.prop(st, "aspect_from_image")
        sub = col.row()
        sub.enabled = not st.aspect_from_image
        sub.prop(st, "texture_aspect")
        if st.use_texture_aspect:
            layout.label(text=f"Effective aspect: {resolve_texture_aspect(st, context):.4f}")


class UVPACK_PT_region(_Panel, bpy.types.Panel):
    bl_label = "Target Region"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return _Panel.poll(context) and settings(context).pack_mode != "SETS_TO_TILES"

    def draw_header(self, context):
        self.layout.prop(settings(context), "use_custom_region", text="")

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.enabled = st.use_custom_region
        col = layout.column(align=True)
        col.prop(st.custom_region, "min_x")
        col.prop(st.custom_region, "min_y")
        col.prop(st.custom_region, "max_x")
        col.prop(st.custom_region, "max_y")
        layout.label(text=st.custom_region.label())
        row = layout.row(align=True)
        row.operator("uv.uvpack_region_draw", text="Draw In Editor", icon="GREASEPENCIL").which = "CUSTOM"
        row.operator("uv.uvpack_region_reset", text="Reset").which = "CUSTOM"
        op = layout.operator("uv.uvpack_region_from_tile", text="Set To Active UDIM Tile", icon="UV_DATA")
        op.which = "CUSTOM"
        op.from_active = True
        grid = layout.grid_flow(columns=3, align=True)
        for dy, dx in ((1, -1), (1, 0), (1, 1), (0, -1), (0, 0), (0, 1), (-1, -1), (-1, 0), (-1, 1)):
            if dx == 0 and dy == 0:
                grid.label(text="")
                continue
            op = grid.operator("uv.uvpack_region_nudge", text=f"{dx:+d}{dy:+d}")
            op.which, op.dir_x, op.dir_y = "CUSTOM", dx, dy


class UVPACK_PT_sets(_Panel, bpy.types.Panel):
    bl_label = "Island Sets"
    bl_parent_id = "UVPACK_PT_main"

    @classmethod
    def poll(cls, context):
        st = settings(context)
        return _Panel.poll(context) and (
            st.pack_mode in SET_MODES or st.sets_adjacent or st.sets_overlaid)

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        row = layout.row(align=True)
        row.template_list("UI_UL_list", "uvpack_rules", st, "rules", st, "active_rule", rows=2)
        col = row.column(align=True)
        col.operator("uv.uvpack_rule_add", text="", icon="ADD")
        col.operator("uv.uvpack_rule_remove", text="", icon="REMOVE")

        rule = st.rule()
        if rule is None:
            layout.label(text="Add a set rule to start", icon="INFO")
            return
        layout.prop(rule, "name")
        layout.prop(rule, "method")
        layout.operator("uv.uvpack_rule_rebuild", icon="FILE_REFRESH")

        if st.pack_mode in ("SETS_TO_TILES", "SETS_ADJACENT"):
            layout.prop(st, "sets_adjacent")
            sub = layout.row()
            sub.enabled = st.sets_adjacent
            sub.prop(st, "set_cohesion")
        if st.pack_mode != "SETS_OVERLAID":
            layout.prop(st, "sets_overlaid")
        overlaid = st.pack_mode == "SETS_OVERLAID" or st.sets_overlaid
        if overlaid and st.scale_mode != "max":
            layout.label(text="Sets Overlaid needs the Max Scale mode", icon="ERROR")

        row = layout.row(align=True)
        row.template_list("UVPACK_UL_sets", "", rule, "sets", rule, "active_set", rows=4)
        col = row.column(align=True)
        col.operator("uv.uvpack_set_add", text="", icon="ADD")
        col.operator("uv.uvpack_set_remove", text="", icon="REMOVE")

        if rule.method == "MANUAL":
            row = layout.row(align=True)
            row.operator("uv.uvpack_set_assign", icon="GROUP_VERTEX")
            row.operator("uv.uvpack_set_select", text="Select").deselect = False
            row.operator("uv.uvpack_set_select", text="Deselect").deselect = True

        if not len(rule.sets):
            return
        entry = active_set(rule)

        box = layout.box()
        box.label(text="Set Layout:")
        box.prop(rule, "layout_mode")
        if rule.layout_mode == "AUTOMATIC":
            box.prop(rule, "tiles_per_row")
        if rule.layout_mode == "GRID":
            row = box.row(align=True)
            row.prop(rule, "tile_count_x")
            row.prop(rule, "tile_count_y")
        if rule.layout_mode in ("AUTOMATIC", "HORIZONTAL", "VERTICAL"):
            box.prop(entry, "tile_count")
        box.operator("uv.uvpack_set_regions", icon="GRID")

        box = layout.box()
        box.label(text="Target Regions:")
        row = box.row(align=True)
        row.template_list("UVPACK_UL_regions", "", entry, "regions", entry, "active_region", rows=2)
        col = row.column(align=True)
        col.operator("uv.uvpack_set_region_add", text="", icon="ADD")
        col.operator("uv.uvpack_set_region_remove", text="", icon="REMOVE")
        if len(entry.regions):
            active = entry.regions[max(0, min(entry.active_region, len(entry.regions) - 1))]
            grid = box.grid_flow(columns=2, align=True)
            grid.prop(active, "min_x")
            grid.prop(active, "min_y")
            grid.prop(active, "max_x")
            grid.prop(active, "max_y")
            row = box.row(align=True)
            row.operator("uv.uvpack_region_draw", text="Draw In Editor").which = "SET"
            op = row.operator("uv.uvpack_region_from_tile", text="To Active Tile")
            op.which, op.from_active = "SET", True

        box = layout.box()
        box.prop(entry, "use_overrides")
        col = box.column(align=True)
        col.enabled = entry.use_overrides
        col.prop(entry, "scale_locked")
        row = col.row(align=True)
        row.prop(entry, "override_rotation", text="")
        sub = row.row(align=True)
        sub.enabled = entry.override_rotation
        sub.prop(entry, "allow_rotation")
        sub.prop(entry, "rotation_step")
        row = col.row(align=True)
        row.prop(entry, "override_prealign", text="")
        sub = row.row()
        sub.enabled = entry.override_prealign
        sub.prop(entry, "prealign")
        row = col.row(align=True)
        row.prop(entry, "override_scale", text="")
        sub = row.row()
        sub.enabled = entry.override_scale
        sub.prop(entry, "scale_mode", text="")
        row = col.row(align=True)
        row.prop(entry, "override_pixel_margin", text="")
        sub = row.row()
        sub.enabled = entry.override_pixel_margin
        sub.prop(entry, "pixel_margin")
        if entry.use_overrides and (entry.override_scale or entry.override_pixel_margin
                                    or entry.override_prealign):
            if overlaid:
                col.label(text="One engine run per set: no realtime stream.", icon="INFO")
            else:
                col.label(text="Scale mode, pixel margin and prealign are engine-wide:",
                          icon="ERROR")
                col.label(text="they only take effect in Sets Overlaid.")


class UVPACK_PT_tools(_Panel, bpy.types.Panel):
    bl_label = "UV Tools"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        col = layout.column(align=True)
        col.prop(st, "match_mode")
        col.prop(st, "match_threshold")
        layout.label(text="Resolution comes from the Search panel", icon="INFO")
        layout.separator()
        box = layout.box()
        box.label(text="Island parameters:")
        box.prop(st, "bundles")
        box.prop(st, "follow_groups")
        box.operator("uv.uvpack_set_bundle", text="Set Island Parameter",
                     icon="GROUP_VERTEX")
        layout.separator()
        col = layout.column(align=True)
        col.operator("uv.uvpack_stack_matching", text="Stack Matching Islands")
        col.operator("uv.uvpack_select_matching", text="Select Matching Islands")
        col.operator("uv.uvpack_spread", text="Spread Overlapping Islands")
        col.operator("uv.uvpack_undo_spread", text="Undo Spread")
        col.operator("uv.uvpack_follow", text="Follow Groups")
        col.operator("uv.uvpack_island_metrics", text="Island Metrics")
        col.operator("uv.uvpack_align_world", text="Align To World Axis")


class UVPACK_PT_overlay(_Panel, bpy.types.Panel):
    bl_label = "Overlay"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.prop(st, "overlay_enable")
        layout.prop(st, "overlay_select_invalid")
        layout.label(text=f"{len(current_overlay_regions(context))} target regions")


class UVPACK_PT_presets(_Panel, bpy.types.Panel):
    bl_label = "Presets"
    bl_parent_id = "UVPACK_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        st = settings(context)
        layout = self.layout
        layout.label(text="Built in:")
        grid = layout.grid_flow(columns=2, align=True)
        for name in BUILTIN_PRESETS:
            op = grid.operator("uv.uvpack_preset_load", text=name)
            op.name, op.builtin = name, True
        layout.separator()
        layout.prop(st, "preset_name")
        row = layout.row(align=True)
        row.operator("uv.uvpack_preset_save", icon="FILE_TICK").name = ""
        row.operator("uv.uvpack_preset_delete", text="", icon="X").name = ""
        saved = preset_files()
        if saved:
            layout.label(text="Saved:")
            col = layout.column(align=True)
            for name in saved:
                row = col.row(align=True)
                op = row.operator("uv.uvpack_preset_load", text=name)
                op.name, op.builtin = name, False
                row.operator("uv.uvpack_preset_delete", text="", icon="X").name = name
        else:
            layout.label(text=f"No saved presets in {preset_dir() or '(unknown)'}")


classes = (UVPACK_UL_sets, UVPACK_UL_regions,
           UVPACK_PT_main, UVPACK_PT_tiles, UVPACK_PT_layout, UVPACK_PT_density,
           UVPACK_PT_margins, UVPACK_PT_rotation, UVPACK_PT_search, UVPACK_PT_aspect,
           UVPACK_PT_region, UVPACK_PT_sets, UVPACK_PT_tools, UVPACK_PT_overlay,
           UVPACK_PT_presets)
