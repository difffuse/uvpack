# The add-on's vocabulary in one place: the enum tables every operator and panel picks
# its items from, the per-face attribute names, the scene property groups, and the
# add-on preference the rest of the package reads the uvpack_cli path out of.
#
# This module imports nothing from the package, so every other module can import it.
import bpy
import bmesh


# ---------------------------------------------------------------- per-face attributes
# Island parameters live in face-domain integer attributes: bmesh exposes them
# directly in edit mode, one value per face, and an island takes the value of its
# first face (the operators write the same value to every face of a selection).
SET_ATTR = "uvpack_set"                 # manual set number
BUNDLE_ATTR = "uvpack_bundle"           # islands sharing a bundle are stacked and bound
FOLLOW_ATTR = "uvpack_follow_group"     # followers replay the leader of the same number
SPREAD_OFFSET_ATTRS = ("uvpack_spread_offset_x", "uvpack_spread_offset_y")
DEFAULT_SET = 0
DEFAULT_BUNDLE = 0

PRESET_DIR = "uvpack_presets"           # under bpy.utils.user_resource("CONFIG")


# ---------------------------------------------------------------- enum tables
MATCH_MODE_ITEMS = [
    ("outline", "Outline", "Islands match when their outlines coincide after a rigid transform"),
    ("vertices", "Vertex Positions", "Islands match when their vertex sets coincide after a transform"),
    ("topology", "Topology", "Islands match when their face topology and their layout match"),
]

# Cycled by set number so two neighbouring sets never share a colour.
SET_COLORS = [
    (0.25, 0.45, 1.0), (1.0, 0.85, 0.1), (0.2, 0.85, 0.85), (0.3, 0.85, 0.3),
    (1.0, 0.55, 0.1), (0.85, 0.3, 0.85), (1.0, 0.35, 0.35), (0.6, 0.6, 1.0),
    (0.55, 1.0, 0.75), (0.9, 0.9, 0.9), (0.7, 0.45, 0.2), (0.45, 0.7, 0.2),
    (0.8, 0.2, 0.5),
]

SCALE_MODES = [
    ("max", "Max Scale", "Pack at the largest uniform scale that fits"),
    ("fixed", "Fixed Scale", "Keep the island scale and place what fits"),
    ("max_margin", "Fixed Scale, Max Margin", "Keep the scale and find the largest gap"),
]

SET_RULE_METHODS = [
    ("MATERIAL", "Material", "One set per material slot"),
    ("OBJECT", "Object", "One set per object"),
    ("MESH_PART", "Mesh Part", "One set per connected mesh part (3D connectivity component)"),
    ("COLLECTION", "Collection", "One set per collection the object belongs to"),
    ("VERTEX_COLOR", "Vertex Color", "One set per distinct colour of the active colour attribute"),
    ("MANUAL", "Manual", "Sets assigned by hand with 'Assign To Set'"),
]

PACK_MODES = [
    ("ONE_TILE", "One Tile", "Pack into one target region"),
    ("TILE_GRID", "Tile Grid", "Pack into a grid of tiles"),
    ("SETS_TO_TILES", "Sets To Tiles", "Every set gets its own target regions"),
    ("SETS_ADJACENT", "Sets Adjacent", "One layout, set members kept next to each other"),
    ("SETS_OVERLAID", "Sets Overlaid", "Each set packed on its own, at its own scale"),
]

# The pack modes that hand the engine one target region per set.
SET_REGION_MODES = ("SETS_TO_TILES", "SETS_OVERLAID")
# The pack modes that build sets at all.
SET_MODES = ("SETS_TO_TILES", "SETS_ADJACENT", "SETS_OVERLAID")

LAYOUT_MODES = [
    ("AUTOMATIC", "Automatic", "Consecutive tiles, wrapping at 'Tiles Per Row'"),
    ("HORIZONTAL", "Automatic (Horizontal)", "One row of tiles per set"),
    ("VERTICAL", "Automatic (Vertical)", "One column of tiles per set"),
    ("GRID", "Tile Grid", "Every set gets the same tile grid"),
    ("MANUAL", "Manual", "Regions are edited by hand and never regenerated"),
]

# How the layout grows inside its target (the engine's `fill`).
FILL_MODES = [
    ("free", "Free", "No bias: the best layout the budget can find"),
    ("width_first", "Width First", "Fill the target's width, grow upward"),
    ("height_first", "Height First", "Fill the target's height, grow rightward"),
    ("proportional", "Proportional", "Grow towards the target's own shape"),
]

ORIGINS = [
    ("bottom_left", "Bottom Left", "The engine's own corner (a no-op)"),
    ("bottom_right", "Bottom Right", "Mirror the finished layout horizontally inside the target"),
    ("top_right", "Top Right", "Mirror the finished layout in both axes"),
    ("top_left", "Top Left", "Mirror the finished layout vertically inside the target"),
]

DENSITY_SPACES = [
    ("local", "Local", "Measure against each island's object-space 3D area"),
    ("global", "Global", "Measure against the world-space 3D area (object scale folded in)"),
]

SCALE_POLICIES = [
    ("shared", "Shared", "One scale for the whole run"),
    ("per_group", "Per Set", "Every island keeps its own texel density"),
    ("custom", "Custom", "One scale per set (the set's scale cluster)"),
    ("by_tiles", "By Tiles", "Let the engine cluster the islands by density"),
]

SNAP_MODES = [
    ("bbox_min", "Bounding Box Corner", "Move the island so its bounding-box minimum lands on the texel grid"),
    ("bbox", "Bounding Box", "Land both bounding-box edges on the grid (slight per-island non-uniform scale)"),
    ("outline", "Outline Vertices", "Snap every boundary vertex, hole outlines included"),
    ("all", "All Vertices", "Snap every vertex, interior ones included"),
]

SNAP_TARGETS = [
    ("corner", "Texel Corner", "Multiples of 1 / texture size"),
    ("center", "Texel Center", "Texel centres, (k + 0.5) / texture size"),
]

BIND_OVERLAPPING = [
    ("off", "Off", "Overlapping islands are packed separately"),
    ("touching", "Touching", "Islands that overlap anywhere move as one"),
    ("identical", "Identical", "Only identical stacked islands move as one"),
]

TILE_FILL = [
    ("sequential", "Sequential", "Fill a tile before moving on"),
    ("any", "Any", "Let the placement cost pick the tile"),
]

OTHERS_MODES = [
    ("IGNORE", "Ignore", "Unselected islands are left alone and may be covered"),
    ("OBSTACLES", "Avoid Unselected", "Unselected islands become obstacles the pack avoids"),
    ("REPACK", "Include Overlapping Unselected",
     "Unselected islands that overlap the selection join the pack, the rest become obstacles"),
]


# ---------------------------------------------------------------- preferences access
# The add-on preference only exists when the package is registered as an add-on
# (installed and enabled). When it is loaded with importlib for scripting or testing,
# bpy.context.preferences.addons has no entry for it, so fall back to a module-level
# path that set_cli_path() writes.
_FALLBACK_CLI_PATH = "uvpack_cli"

# The name Blender registered the add-on under: the package, whatever it was imported as.
ADDON_PACKAGE = __package__ or __name__


def addon_prefs(context=None):
    """The registered AddonPreferences, or None when loaded outside the add-on system."""
    context = context or bpy.context
    entry = context.preferences.addons.get(ADDON_PACKAGE)
    return entry.preferences if entry is not None else None


def set_cli_path(path, context=None):
    """Set the uvpack_cli path programmatically, registered as an add-on or not."""
    global _FALLBACK_CLI_PATH
    _FALLBACK_CLI_PATH = path
    prefs = addon_prefs(context)
    if prefs is not None:
        prefs.cli_path = path


def get_cli_path(context=None):
    prefs = addon_prefs(context)
    raw = prefs.cli_path if prefs is not None else _FALLBACK_CLI_PATH
    # bpy.path.abspath resolves Blender's "//" relative paths; plain names such as
    # "uvpack_cli" are returned unchanged and left to the PATH lookup.
    return bpy.path.abspath(raw) if raw else "uvpack_cli"


# ---------------------------------------------------------------- small helpers
def settings(context=None):
    context = context or bpy.context
    return getattr(context.scene, "uvpack", None)


def tag_redraw():
    screen = getattr(bpy.context, "screen", None)
    for area in getattr(screen, "areas", ()) or ():
        if area.type == "IMAGE_EDITOR":
            area.tag_redraw()


def _redraw(self, context):
    """Property update callback: repaint the UV editors so the overlay follows."""
    tag_redraw()


def _overlay_toggled(self, context):
    # regions imports this module, so bind it late rather than at import time.
    from . import regions  # noqa: PLC0415
    regions.overlay_sync(self)


def uv_editor_image(context):
    """The image shown in a UV editor, if there is one (None in background mode)."""
    screen = getattr(context, "screen", None)
    for area in getattr(screen, "areas", ()) or ():
        if area.type == "IMAGE_EDITOR":
            image = area.spaces.active.image
            if image is not None:
                return image
    # No UI (headless, or no UV editor open): fall back to the file's own image, if
    # there is exactly one to choose. Blender always carries a Render Result and a
    # Viewer Node image, which are never what a UV layout is packed for.
    images = [i for i in bpy.data.images if i.type not in {"RENDER_RESULT", "COMPOSITING"}]
    if len(images) == 1:
        return images[0]
    tiled = [i for i in images if i.source == "TILED"]
    return tiled[0] if len(tiled) == 1 else None


def tile_xy(number):
    """UDIM 1001 -> (0, 0), 1002 -> (1, 0), 1011 -> (0, 1)."""
    n = int(number) - 1001
    return n % 10, n // 10


def resolve_tex_size(st, context):
    if st.tex_size_from_image:
        image = uv_editor_image(context)
        if image is not None and image.size[0] > 0:
            return int(image.size[0])
    return int(st.tex_size)


def resolve_texture_aspect(st, context):
    if not st.use_texture_aspect:
        return 1.0
    if st.aspect_from_image:
        image = uv_editor_image(context)
        if image is not None and image.size[0] > 0 and image.size[1] > 0:
            return float(image.size[0]) / float(image.size[1])
    return float(st.texture_aspect)


def requested_margin(st, context):
    """The island gap the engine will honor, in UV units -- what a caller checks against."""
    if st.use_pixel_margin:
        tex = resolve_tex_size(st, context)
        return st.pixel_margin / float(tex) if tex else st.margin
    return st.margin


def int_attribute(bm, name, create=True):
    layer = bm.faces.layers.int.get(name)
    if layer is None and create:
        layer = bm.faces.layers.int.new(name)
    return layer


def ensure_int_attributes(context, names):
    """Create the face attributes before anything takes hold of a BMFace: adding a
    custom data layer reallocates the mesh, and every BMFace grabbed before that raises
    "BMesh data of type BMFace has been removed" the moment it is used again."""
    for obj in context.objects_in_mode_unique_data:
        if obj.type != "MESH":
            continue
        bm = bmesh.from_edit_mesh(obj.data)
        for name in names:
            int_attribute(bm, name)


# ---------------------------------------------------------------- property groups
class UVPACK_PG_region(bpy.types.PropertyGroup):
    """A target region in UV space. 1.0 is one UDIM tile; values may leave [0, 1]."""
    min_x: bpy.props.FloatProperty(name="Min X", default=0.0, precision=4, update=_redraw)
    min_y: bpy.props.FloatProperty(name="Min Y", default=0.0, precision=4, update=_redraw)
    max_x: bpy.props.FloatProperty(name="Max X", default=1.0, precision=4, update=_redraw)
    max_y: bpy.props.FloatProperty(name="Max Y", default=1.0, precision=4, update=_redraw)

    def as_list(self):
        """[minx, miny, maxx, maxy], corners normalized so min <= max."""
        return [min(self.min_x, self.max_x), min(self.min_y, self.max_y),
                max(self.min_x, self.max_x), max(self.min_y, self.max_y)]

    def set_region(self, x0, y0, x1, y1):
        self.min_x, self.min_y, self.max_x, self.max_y = x0, y0, x1, y1

    def set_tile(self, tx, ty):
        self.set_region(float(tx), float(ty), float(tx + 1), float(ty + 1))

    def label(self):
        x0, y0, x1, y1 = self.as_list()
        if all(abs(v - round(v)) < 1e-6 for v in (x0, y0, x1, y1)) and \
                abs((x1 - x0) - 1) < 1e-6 and abs((y1 - y0) - 1) < 1e-6:
            return f"Tile {int(round(x0))}:{int(round(y0))}"
        return f"[{x0:.2f}, {y0:.2f}] - [{x1:.2f}, {y1:.2f}]"

    def is_valid(self):
        x0, y0, x1, y1 = self.as_list()
        return (x1 - x0) > 1e-4 and (y1 - y0) > 1e-4


class UVPACK_PG_set(bpy.types.PropertyGroup):
    """One set of a set rule: a number, target regions and optional overrides."""
    name: bpy.props.StringProperty(name="Name", default="Set")
    num: bpy.props.IntProperty(name="Number", default=0, min=0, max=1000)
    color: bpy.props.FloatVectorProperty(name="Color", subtype="COLOR", size=3,
                                         min=0.0, max=1.0, default=(1.0, 0.85, 0.1), update=_redraw)
    key: bpy.props.StringProperty(name="Key", default="",
                                  description="The material/object/collection/colour this set was built from")
    tile_count: bpy.props.IntProperty(name="Tile Count", default=1, min=1, max=100, update=_redraw)
    regions: bpy.props.CollectionProperty(type=UVPACK_PG_region)
    active_region: bpy.props.IntProperty(name="Active Region", default=0)

    use_overrides: bpy.props.BoolProperty(
        name="Override Global Options", default=False,
        description="Use this set's own values instead of the global ones")
    override_rotation: bpy.props.BoolProperty(name="Override Rotation", default=False)
    allow_rotation: bpy.props.BoolProperty(name="Allow Rotation", default=True)
    rotation_step: bpy.props.IntProperty(name="Rotation Step (deg)", default=90, min=1, max=180)
    override_prealign: bpy.props.BoolProperty(name="Override Prealign", default=False)
    prealign: bpy.props.BoolProperty(name="Prealign", default=True)
    override_scale: bpy.props.BoolProperty(name="Override Scale Mode", default=False)
    scale_mode: bpy.props.EnumProperty(name="Scale Mode", items=SCALE_MODES, default="max")
    scale_locked: bpy.props.BoolProperty(
        name="Lock Island Scale", default=False,
        description="Keep this set's islands at scale 1 (a per-island engine flag, honored in every mode)")
    override_pixel_margin: bpy.props.BoolProperty(name="Override Pixel Margin", default=False)
    pixel_margin: bpy.props.IntProperty(name="Margin (px)", default=5, min=0, max=256)

    def ensure_region(self):
        if not len(self.regions):
            self.regions.add()
        return self.regions[0]


class UVPACK_PG_rule(bpy.types.PropertyGroup):
    """A named set rule: how islands become sets, and where each set packs."""
    name: bpy.props.StringProperty(name="Name", default="Set Rule")
    method: bpy.props.EnumProperty(name="Method", items=SET_RULE_METHODS,
                                   default="MATERIAL", update=_redraw)
    sets: bpy.props.CollectionProperty(type=UVPACK_PG_set)
    active_set: bpy.props.IntProperty(name="Active Set", default=0)
    layout_mode: bpy.props.EnumProperty(name="Set Layout Mode", items=LAYOUT_MODES,
                                        default="AUTOMATIC", update=_redraw)
    tiles_per_row: bpy.props.IntProperty(name="Tiles Per Row", default=10, min=1, max=100, update=_redraw)
    tile_count_x: bpy.props.IntProperty(name="Tile Count (X)", default=1, min=1, max=100, update=_redraw)
    tile_count_y: bpy.props.IntProperty(name="Tile Count (Y)", default=1, min=1, max=100, update=_redraw)

    def next_num(self):
        return max((s.num for s in self.sets), default=-1) + 1

    def set_by_num(self, num):
        for s in self.sets:
            if s.num == num:
                return s
        return None

    def set_by_key(self, key):
        for s in self.sets:
            if s.key == key:
                return s
        return None

    def add_set(self, name=None, key="", color=None):
        # The number has to be taken before the entry exists, or the fresh entry's
        # own default of 0 would push every set one number up.
        num = self.next_num()
        entry = self.sets.add()
        entry.num = num
        entry.name = name if name else f"S{entry.num}"
        entry.key = key
        entry.color = color if color is not None else SET_COLORS[entry.num % len(SET_COLORS)]
        entry.ensure_region().set_tile(entry.num % self.tiles_per_row, entry.num // self.tiles_per_row)
        return entry


class UVPACK_PG_settings(bpy.types.PropertyGroup):
    """Everything the engine takes, stored on the scene."""

    # -- scale
    scale_mode: bpy.props.EnumProperty(name="Scale Mode", items=SCALE_MODES, default="max")
    fixed_scale: bpy.props.FloatProperty(name="Scale", default=1.0, min=0.0001, precision=4)
    allow_partial: bpy.props.BoolProperty(
        name="Allow Partial", default=False,
        description="Fixed-scale modes place what fits and report the rest as unplaced")
    shelve_unplaced: bpy.props.BoolProperty(
        name="Shelve Unplaced", default=False,
        description="Lay the islands that did not fit in rows above the target instead of "
                    "leaving them where they were (needs Allow Partial)")
    equalize_density: bpy.props.BoolProperty(
        name="Equalize Density", default=False,
        description="Pre-scale islands so the UV area per 3D area is uniform (texel density)")

    # -- margins
    margin: bpy.props.FloatProperty(name="Margin", default=0.003, min=0.0, max=0.2, precision=4)
    use_border_margin: bpy.props.BoolProperty(name="Enable Border Margin", default=False)
    border_margin: bpy.props.FloatProperty(name="Border Margin", default=0.003, min=0.0, max=0.2, precision=4)
    use_pixel_margin: bpy.props.BoolProperty(
        name="Enable Pixel Margin", default=False,
        description="Express the margins in texels of a texture instead of UV units")
    pixel_margin: bpy.props.IntProperty(name="Margin (px)", default=5, min=0, max=256)
    pixel_border_margin: bpy.props.IntProperty(name="Border Margin (px)", default=1, min=0, max=256)
    tex_size: bpy.props.IntProperty(name="Texture Size", default=1024, min=16, max=32768)
    tex_size_from_image: bpy.props.BoolProperty(
        name="Size From Active Image", default=True,
        description="Take the texture size from the image shown in the UV editor")
    snap_to_texels: bpy.props.BoolProperty(
        name="Snap To Texels", default=False,
        description="Snap every island to the texel grid after packing")
    snap_mode: bpy.props.EnumProperty(
        name="Vertex Snap", items=SNAP_MODES, default="bbox_min",
        description="Which vertices the snap moves")
    snap_target: bpy.props.EnumProperty(
        name="Snap To", items=SNAP_TARGETS, default="corner")

    # -- rotation
    allow_rotation: bpy.props.BoolProperty(name="Allow Rotation", default=True)
    rotation_step: bpy.props.IntProperty(name="Rotation Step (deg)", default=90, min=1, max=180)
    prealign: bpy.props.BoolProperty(
        name="Prealign", default=True,
        description="Align every island to its own minimum-area rectangle before the search")
    allow_mirroring: bpy.props.BoolProperty(
        name="Allow Mirroring", default=False,
        description="Allow mirrored orientations (only for symmetric textures)")

    # -- search
    resolution: bpy.props.IntProperty(
        name="Resolution", default=500, min=10, max=10000,
        description="Occupancy resolution; the engine's stripe height is 1 / resolution")
    search_time: bpy.props.FloatProperty(name="Search Time (s)", default=2.0, min=0.0, max=3600.0)
    stall_timeout: bpy.props.FloatProperty(
        name="Stall Timeout (s)", default=0.0, min=0.0, max=3600.0,
        description="Stop the improvement phase after this many seconds without an "
                    "improvement instead of spending the rest of the search time. 0 disables it")
    trials: bpy.props.IntProperty(name="Trials", default=8, min=1, max=64)
    threads: bpy.props.IntProperty(name="Thread Count", default=0, min=0, max=256,
                                   description="0 uses every core")
    compaction_time: bpy.props.FloatProperty(name="Compaction Time (s)", default=0.0, min=0.0, max=3600.0)
    compaction_rotation: bpy.props.BoolProperty(name="Compaction Rotation", default=True)
    fill: bpy.props.EnumProperty(name="Fill", items=FILL_MODES, default="free")
    origin: bpy.props.EnumProperty(name="Origin", items=ORIGINS, default="bottom_left")
    stream: bpy.props.BoolProperty(
        name="Realtime Solution", default=True,
        description="Run the packer asynchronously and draw the latest layout the search finds "
                    "as it finds it; ESC stops the search and keeps the best one so far")

    # -- binding and sets
    bind_overlapping: bpy.props.EnumProperty(
        name="Bind Overlapping", default="off", items=BIND_OVERLAPPING)
    sets_adjacent: bpy.props.BoolProperty(name="Sets Adjacent", default=False)
    set_cohesion: bpy.props.FloatProperty(name="Set Cohesion", default=0.0, min=0.0, max=1.0)
    sets_overlaid: bpy.props.BoolProperty(
        name="Sets Overlaid", default=False,
        description="Run the whole search once per set, each over the full target at its "
                    "own scale, and merge. The sets overlap by design")

    # -- matching (the UV tools, and the pack options built on them)
    match_mode: bpy.props.EnumProperty(name="Match Mode", items=MATCH_MODE_ITEMS,
                                       default="outline")
    match_threshold: bpy.props.FloatProperty(name="Match Threshold", default=0.1,
                                             min=0.0, max=10.0, precision=3)
    bundles: bpy.props.BoolProperty(
        name="Bundles", default=False,
        description="Stack the islands sharing a bundle onto each other and pack the "
                    "stack as a single bound body")
    follow_groups: bpy.props.BoolProperty(
        name="Follow Groups", default=False,
        description="Unselected islands carrying a follow group follow the packing "
                    "transform of the matching selected island")

    # -- texel density
    texel_density: bpy.props.FloatProperty(
        name="Texel Density (px/unit)", default=0.0, min=0.0, max=100000.0,
        description="Pre-scale every island with a 3D area to this texel density and pack "
                    "at a fixed scale of 1. 0 disables it; needs a texture size")
    density_space: bpy.props.EnumProperty(name="Density Space", items=DENSITY_SPACES, default="local")
    scale_policy: bpy.props.EnumProperty(name="Scale Policy", items=SCALE_POLICIES, default="shared")
    shrink_spanning: bpy.props.BoolProperty(
        name="Shrink Spanning Islands", default=False,
        description="During the improvement phase, shrink islands that nearly span the "
                    "target so one of them stops capping the scale of the rest")

    # -- texture aspect
    use_texture_aspect: bpy.props.BoolProperty(
        name="Use Texture Aspect", default=False,
        description="Pack for a non-square texture (width / height)")
    aspect_from_image: bpy.props.BoolProperty(name="Aspect From Active Image", default=True)
    texture_aspect: bpy.props.FloatProperty(name="Texture Aspect", default=1.0, min=0.01,
                                            max=100.0, precision=4)

    # -- selection
    only_selected: bpy.props.BoolProperty(name="Only Selected Faces", default=True)
    split_at_vertices: bpy.props.BoolProperty(
        name="Split At Vertices", default=False,
        description="Split islands whose faces meet only at a UV vertex")
    others_mode: bpy.props.EnumProperty(
        name="Unselected Islands", default="IGNORE", items=OTHERS_MODES)
    keep_pinned: bpy.props.BoolProperty(
        name="Keep Pinned Islands", default=True,
        description="Islands with a pinned UV are never moved")
    pinned_as_obstacles: bpy.props.BoolProperty(
        name="Pinned As Obstacles", default=True,
        description="Send kept pinned islands to the engine as obstacles so the pack avoids them")

    # -- tiles and target regions
    pack_mode: bpy.props.EnumProperty(name="Packing Mode", items=PACK_MODES,
                                      default="ONE_TILE", update=_redraw)
    tile_count_x: bpy.props.IntProperty(name="Tile Count (X)", default=1, min=1, max=100, update=_redraw)
    tile_count_y: bpy.props.IntProperty(name="Tile Count (Y)", default=1, min=1, max=100, update=_redraw)
    tiles_from_udim: bpy.props.BoolProperty(
        name="Use Blender UDIM Grid", default=False,
        description="Take the tile grid from the tiles of the image shown in the UV editor",
        update=_redraw)
    tile_fill: bpy.props.EnumProperty(name="Tile Fill", default="sequential", items=TILE_FILL)
    use_custom_region: bpy.props.BoolProperty(name="Enable Target Region", default=False, update=_redraw)
    custom_region: bpy.props.PointerProperty(type=UVPACK_PG_region)

    # -- set rules
    rules: bpy.props.CollectionProperty(type=UVPACK_PG_rule)
    active_rule: bpy.props.IntProperty(name="Active Set Rule", default=0, update=_redraw)

    # -- overlay
    overlay_enable: bpy.props.BoolProperty(
        name="Show Target Regions", default=False,
        description="Draw the target regions in the UV editor", update=_overlay_toggled)
    overlay_select_invalid: bpy.props.BoolProperty(
        name="Select Invalid Islands", default=True,
        description="After packing, select the faces of islands the engine rejected or could not place")

    # -- presets
    preset_name: bpy.props.StringProperty(name="Preset Name", default="My Preset")

    def rule(self):
        if not len(self.rules):
            return None
        idx = max(0, min(self.active_rule, len(self.rules) - 1))
        return self.rules[idx]


class SimpleSettings:
    """The subset of the scene settings the collector reads, filled from the operator's
    own properties instead of from the scene. It is what lets the explicit-property path
    (the UV menu's dialog, and every script that passes arguments) share one extraction
    with the sidebar path rather than keeping a second copy of it."""

    def __init__(self, only_selected=True, split_at_vertices=False):
        self.only_selected = only_selected
        self.split_at_vertices = split_at_vertices
        self.keep_pinned = False
        self.pinned_as_obstacles = False
        self.others_mode = "IGNORE"


classes = (UVPACK_PG_region, UVPACK_PG_set, UVPACK_PG_rule, UVPACK_PG_settings)
