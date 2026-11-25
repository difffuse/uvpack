# Presets: the scalar options plus the target region, written as versioned JSON under
# the user's config directory. Set rules are per-scene data, not options, so they stay
# out of a preset.
import json
import os

import bpy

from .props import PRESET_DIR, settings, tag_redraw

#: The preset file format. Bumped when the property names in a preset change; a file
#: written under any other version is refused rather than guessed at.
PRESET_VERSION = 1

BUILTIN_PRESETS = {
    "Default": {},
    "Fast": {"resolution": 200, "search_time": 0.5, "trials": 4, "compaction_time": 0.0},
    "High Quality": {"resolution": 2000, "search_time": 10.0, "trials": 16,
                     "compaction_time": 5.0, "compaction_rotation": True},
    "Pixel Perfect 1024": {"use_pixel_margin": True, "tex_size": 1024, "tex_size_from_image": False,
                           "pixel_margin": 4, "use_border_margin": True, "pixel_border_margin": 2,
                           "snap_to_texels": True, "snap_mode": "outline"},
    "Texel Density": {"equalize_density": True, "scale_mode": "max"},
}

_PRESET_PROP_SKIP = {"rna_type", "name", "rules", "active_rule", "custom_region",
                     "preset_name", "overlay_enable"}


def preset_dir(create=False):
    return bpy.utils.user_resource("CONFIG", path=PRESET_DIR, create=create)


def preset_files():
    folder = preset_dir()
    if not folder or not os.path.isdir(folder):
        return []
    return sorted(f[:-5] for f in os.listdir(folder) if f.endswith(".json"))


def settings_to_dict(st):
    """Every scalar engine option, plus the target region."""
    out = {}
    for prop in st.bl_rna.properties:
        if prop.identifier in _PRESET_PROP_SKIP or prop.is_readonly:
            continue
        if prop.type in {"COLLECTION", "POINTER"}:
            continue
        out[prop.identifier] = getattr(st, prop.identifier)
    out["custom_region"] = st.custom_region.as_list()
    return out


def dict_to_settings(st, data):
    """Apply a preset. Unknown or unsettable keys are skipped and counted."""
    skipped = 0
    for key, value in data.items():
        if key == "custom_region":
            if isinstance(value, (list, tuple)) and len(value) == 4:
                st.custom_region.set_region(*[float(v) for v in value])
            continue
        if key in _PRESET_PROP_SKIP or not hasattr(st, key):
            skipped += 1
            continue
        try:
            setattr(st, key, value)
        except (TypeError, ValueError, AttributeError):
            skipped += 1
    return skipped


class UVPACK_OT_preset_save(bpy.types.Operator):
    """Save the current options as a named preset"""
    bl_idname = "uv.uvpack_preset_save"
    bl_label = "Save Preset"
    bl_options = {"REGISTER"}

    name: bpy.props.StringProperty(name="Name", default="")

    def execute(self, context):
        st = settings(context)
        name = (self.name or st.preset_name or "").strip()
        if not name:
            self.report({"ERROR"}, "A preset needs a name")
            return {"CANCELLED"}
        name = bpy.path.clean_name(name)
        folder = preset_dir(create=True)
        path = os.path.join(folder, name + ".json")
        payload = {"version": PRESET_VERSION, "properties": settings_to_dict(st)}
        try:
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=1, sort_keys=True)
        except OSError as ex:
            self.report({"ERROR"}, f"could not write {path}: {ex}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"preset saved to {path}")
        return {"FINISHED"}


class UVPACK_OT_preset_load(bpy.types.Operator):
    """Load a preset, built in or saved"""
    bl_idname = "uv.uvpack_preset_load"
    bl_label = "Load Preset"
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Name", default="")
    builtin: bpy.props.BoolProperty(name="Built In", default=False)

    def execute(self, context):
        st = settings(context)
        name = (self.name or st.preset_name or "").strip()
        if self.builtin or name in BUILTIN_PRESETS:
            if name not in BUILTIN_PRESETS:
                self.report({"ERROR"}, f"no built-in preset named '{name}'")
                return {"CANCELLED"}
            # Every built-in starts from the property defaults, then its own deltas
            # go on top -- which makes "Default" a plain reset.
            for prop in st.bl_rna.properties:
                if prop.identifier in _PRESET_PROP_SKIP or prop.is_readonly or \
                        prop.type in {"COLLECTION", "POINTER"}:
                    continue
                try:
                    st.property_unset(prop.identifier)
                except (TypeError, RuntimeError):
                    pass
            dict_to_settings(st, BUILTIN_PRESETS[name])
            self.report({"INFO"}, f"built-in preset '{name}' loaded")
            tag_redraw()
            return {"FINISHED"}

        path = os.path.join(preset_dir(create=True), bpy.path.clean_name(name) + ".json")
        if not os.path.exists(path):
            self.report({"ERROR"}, f"no preset at {path}")
            return {"CANCELLED"}
        try:
            with open(path, encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, ValueError) as ex:
            self.report({"ERROR"}, f"could not read {path}: {ex}")
            return {"CANCELLED"}
        version = int(payload.get("version", 0))
        if version != PRESET_VERSION:
            self.report({"ERROR"}, f"preset '{name}' is format version {version}; "
                                   f"this add-on reads version {PRESET_VERSION}")
            return {"CANCELLED"}
        skipped = dict_to_settings(st, payload.get("properties", {}))
        if skipped:
            self.report({"WARNING"}, f"preset '{name}' loaded, {skipped} options skipped")
        else:
            self.report({"INFO"}, f"preset '{name}' loaded")
        tag_redraw()
        return {"FINISHED"}


class UVPACK_OT_preset_delete(bpy.types.Operator):
    """Delete a saved preset"""
    bl_idname = "uv.uvpack_preset_delete"
    bl_label = "Delete Preset"
    bl_options = {"REGISTER"}

    name: bpy.props.StringProperty(name="Name", default="")

    def execute(self, context):
        name = (self.name or settings(context).preset_name or "").strip()
        path = os.path.join(preset_dir(create=True), bpy.path.clean_name(name) + ".json")
        if not os.path.exists(path):
            self.report({"ERROR"}, f"no preset at {path}")
            return {"CANCELLED"}
        os.remove(path)
        self.report({"INFO"}, f"preset '{name}' deleted")
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)


classes = (UVPACK_OT_preset_save, UVPACK_OT_preset_load, UVPACK_OT_preset_delete)
