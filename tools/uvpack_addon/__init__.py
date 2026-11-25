# Blender add-on: pack the selected UV islands of the edit-mode mesh objects with
# uvpack_cli, plus the UV tools built on tools/uv_similarity.py (stack/select matching
# islands, spread overlapping ones, bundles, follow groups, island metrics, align to a
# world axis).
#
# Install this folder, tools/uv_islands.py, tools/uv_similarity.py and the
# python/uvpack_geometry package together in Blender's scripts/addons directory.
# Enable "UVPack Islands" and set the path to uvpack_cli in its preferences.
# Use: UV Editor > sidebar (N) > UVPack, or UV Editor > UV > UVPack ...
#
# The sidebar exposes the whole engine: scale mode, UV or pixel margins, rotation,
# resolution, search and compaction budgets, binding overlapping islands, texture
# aspect, fill modes and the origin, texel density (equalize, set, policies, shrinking
# spanning islands), set rules with per-set target regions and overrides, a target
# region, pinned-island handling, the realtime solution stream and presets. The menu
# entry keeps the small property dialog of the pack operator, which is also what the
# operator does when it is called from a script without `use_scene_settings`.
#
# Packing runs asynchronously: `uvpack_cli --stream` is driven by a modal operator that
# draws the latest layout while the search runs, and ESC keeps the best one so far.
#
# Layout of the package:
#   props.py     enum tables, per-face attribute names, the scene property groups
#   collect.py   one pass over the edit-mode meshes -> islands, obstacles, frozen
#   sets.py      set rules (material/object/mesh part/collection/colour/manual) and sets
#   regions.py   target regions, the region operators and the UV editor overlay
#   engine.py    request building, applying a result, packing
#   transport.py bounded process I/O, cancel-file lifetime, strict stream completion
#   tools.py     the UV tools around uv_similarity
#   presets.py   preset save, load and delete
#   ui.py        the sidebar panels

bl_info = {
    "name": "UVPack Islands",
    "author": "uvpack",
    "version": (0, 4, 0),
    "blender": (3, 6, 0),
    "location": "UV Editor > Sidebar (N) > UVPack, UV Editor > UV > UVPack Islands",
    "description": "Pack UV islands with the uvpack_cli engine, and the matching tools around it",
    "category": "UV",
}

import importlib.util
import os
import sys

import bpy


# ---------------------------------------------------------------- module import
# uv_islands.py and uv_similarity.py are plain standard-library modules shared with the
# OBJ pipeline, so they are loaded by file location rather than imported by name. That
# works both for an installed add-on (the two files sit inside this folder, or next to
# it in the add-ons directory) and when the package is loaded straight from the
# repository with importlib, where sys.path may not contain the tools folder at all.
def _load_sibling(filename, attribute):
    here = os.path.dirname(os.path.abspath(__file__))
    for folder in (here, os.path.dirname(here)):
        path = os.path.join(folder, filename)
        if not os.path.exists(path):
            continue
        name = __name__ + "._" + attribute
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module
    if here not in sys.path:
        sys.path.insert(0, here)
    return importlib.import_module(attribute)


uv_islands = _load_sibling("uv_islands.py", "uv_islands")
uv_similarity = _load_sibling("uv_similarity.py", "uv_similarity")


# ---------------------------------------------------------------- preferences
class UVPackPreferences(bpy.types.AddonPreferences):
    # Must equal the module name Blender registered the add-on under, which is
    # exactly __name__ both for a plain add-on and for a bl_ext extension.
    bl_idname = __name__

    cli_path: bpy.props.StringProperty(
        name="uvpack_cli path", subtype="FILE_PATH", default="uvpack_cli",
        description="Path to the uvpack_cli executable")
    region_line_width: bpy.props.FloatProperty(
        name="Region Line Width", default=2.0, min=1.0, max=10.0,
        description="Line width of the target regions drawn in the UV editor")

    def draw(self, context):
        self.layout.prop(self, "cli_path")
        self.layout.prop(self, "region_line_width")


# The submodules import `uv_islands` and `uv_similarity` from this package, so they can
# only be imported once the two names above exist.
from . import props      # noqa: E402
from . import collect    # noqa: E402,F401  (reachable as a package attribute)
from . import sets       # noqa: E402
from . import regions    # noqa: E402
from . import engine     # noqa: E402
from . import tools      # noqa: E402
from . import presets    # noqa: E402
from . import ui         # noqa: E402


# ---------------------------------------------------------------- re-exports
# What a script (and the headless test) reaches for, without having to know which
# submodule it ended up in.
from .props import (settings, set_cli_path, get_cli_path,  # noqa: E402,F401
                    PACK_MODES, LAYOUT_MODES, SET_RULE_METHODS, MATCH_MODE_ITEMS,
                    SNAP_MODES, FILL_MODES, UVPACK_PG_settings)
from .collect import collect as collect_islands  # noqa: E402,F401
from .collect import gather, gather_islands, state_from  # noqa: E402,F401
from .sets import face_set_keys, sync_rule_sets, arrange_set_regions  # noqa: E402,F401
from .regions import (build_targets, udim_tiles, overlay_supported,  # noqa: E402,F401
                      overlay_active, enable_overlay, disable_overlay)
from .engine import (UVPACK_OT_pack, engine_options, island_payload, parse_stream_lines,  # noqa: E402,F401
                     run_cli, StreamSession)
from .tools import (UVPACK_OT_stack_matching, UVPACK_OT_select_matching,  # noqa: E402,F401
                    UVPACK_OT_spread, UVPACK_OT_undo_spread, UVPACK_OT_set_bundle,
                    UVPACK_OT_follow, UVPACK_OT_island_metrics, UVPACK_OT_align_world)
from .presets import (PRESET_VERSION, BUILTIN_PRESETS, preset_dir,  # noqa: E402,F401
                      preset_files, settings_to_dict, dict_to_settings)


def menu_func(self, context):
    layout = self.layout
    layout.separator()
    layout.operator(engine.UVPACK_OT_pack.bl_idname)
    layout.operator(tools.UVPACK_OT_stack_matching.bl_idname)
    layout.operator(tools.UVPACK_OT_select_matching.bl_idname)
    layout.operator(tools.UVPACK_OT_spread.bl_idname)
    layout.operator(tools.UVPACK_OT_undo_spread.bl_idname)
    layout.operator(tools.UVPACK_OT_set_bundle.bl_idname)
    layout.operator(tools.UVPACK_OT_follow.bl_idname)
    layout.operator(tools.UVPACK_OT_island_metrics.bl_idname)
    layout.operator(tools.UVPACK_OT_align_world.bl_idname)


classes = ((UVPackPreferences,) + props.classes + engine.classes + tools.classes
           + sets.classes + regions.classes + presets.classes + ui.classes)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.uvpack = bpy.props.PointerProperty(type=props.UVPACK_PG_settings)
    bpy.types.IMAGE_MT_uvs.append(menu_func)


def unregister():
    regions.disable_overlay()
    bpy.types.IMAGE_MT_uvs.remove(menu_func)
    if hasattr(bpy.types.Scene, "uvpack"):
        del bpy.types.Scene.uvpack
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
