# Talking to uvpack_cli: turning the settings into a request, running the engine
# (blocking or as a realtime stream), and writing the layout it sends back onto the
# mesh. The pack operators live here because they are the two ends of that round trip.
from .transport import StreamSession, parse_stream_lines, run_cli

import bpy
import bmesh

from . import uv_islands, uv_similarity
from .props import (settings, get_cli_path, resolve_tex_size, resolve_texture_aspect,
                    SimpleSettings, DEFAULT_BUNDLE, SET_MODES, OTHERS_MODES)
from .collect import (collect, state_from, select_faces, MatchMixin)
from .regions import build_targets, set_overlay_targets


# ---------------------------------------------------------------- engine options
def engine_options(st, context, entry=None):
    """The uvpack_cli "options" object for these settings, with a set's overrides
    folded in. Only the settings the engine takes globally are folded here; the
    per-island ones (rotation lock, step, scale lock) ride on the islands instead."""
    o = {
        "scale_mode": st.scale_mode,
        "scale": st.fixed_scale,
        "allow_partial": st.allow_partial,
        "equalize_density": st.equalize_density,
        "density_space": st.density_space,
        "allow_rotation": st.allow_rotation,
        "rotation_step": st.rotation_step,
        "prealign": st.prealign,
        "allow_mirroring": st.allow_mirroring,
        "stripe_height": 1.0 / max(1, st.resolution),
        "time_budget": st.search_time,
        "trials": st.trials,
        "threads": st.threads,
        "compaction_time": st.compaction_time,
        "compaction_rotation": st.compaction_rotation,
        "bind_overlapping": st.bind_overlapping,
        "tile_fill": st.tile_fill,
        "fill": st.fill,
        "origin": st.origin,
    }
    if st.stall_timeout > 0:
        o["stall_timeout"] = st.stall_timeout
    if st.allow_partial and st.shelve_unplaced:
        o["shelve_unplaced"] = True
    if st.shrink_spanning:
        o["shrink_spanning"] = True
    if st.scale_policy != "shared":
        o["scale_policy"] = st.scale_policy
    if entry is not None and entry.use_overrides:
        if entry.override_rotation:
            o["allow_rotation"] = entry.allow_rotation
            o["rotation_step"] = entry.rotation_step
        if entry.override_prealign:
            o["prealign"] = entry.prealign
        if entry.override_scale:
            o["scale_mode"] = entry.scale_mode

    pixel_margin = st.pixel_margin
    if entry is not None and entry.use_overrides and entry.override_pixel_margin:
        pixel_margin = entry.pixel_margin
    if st.use_pixel_margin:
        o["tex_size"] = resolve_tex_size(st, context)
        o["pixel_margin"] = pixel_margin
        if st.use_border_margin:
            o["pixel_border_margin"] = st.pixel_border_margin
    else:
        o["margin"] = st.margin
        if st.use_border_margin:
            o["border_margin"] = st.border_margin
    if st.snap_to_texels:
        # Snapping is ignored by the engine without a texture size, so make sure one is
        # sent even when the margins are in UV units.
        o["tex_size"] = o.get("tex_size") or resolve_tex_size(st, context)
        o["snap_to_texels"] = True
        o["snap_mode"] = st.snap_mode
        o["snap_target"] = st.snap_target
    if st.texel_density > 0:
        # Setting a density pre-scales the islands, so the pack has to keep that scale.
        # The engine only switches to a fixed scale of 1 by itself when no scale_mode is
        # sent at all, and this always sends one, so do the switch here.
        o["tex_size"] = o.get("tex_size") or resolve_tex_size(st, context)
        o["texel_density"] = st.texel_density
        if o["scale_mode"] == "max":
            o["scale_mode"], o["scale"] = "fixed", 1.0

    aspect = resolve_texture_aspect(st, context)
    if abs(aspect - 1.0) > 1e-9:
        o["target_aspect"] = aspect
    if st.sets_adjacent:
        o["group_adjacent"] = True
        o["group_cohesion"] = st.set_cohesion
    return o


def simple_options(op):
    """The options of the pack operator's own dialog: what a script that passes explicit
    properties (and every test written against that operator) expects to see."""
    options = {"margin": op.margin, "rotation_step": op.rotation_step,
               "allow_rotation": op.rotation_step > 0, "allow_mirroring": op.mirroring,
               "scale_mode": op.scale_mode, "equalize_density": op.equalize_density,
               "bind_overlapping": op.bind_overlapping, "time_budget": op.time_budget,
               "trials": 8}
    if op.tex_size > 0:
        options.update({"tex_size": op.tex_size, "pixel_margin": op.pixel_margin,
                        "snap_to_texels": op.snap_to_texels,
                        "snap_mode": op.snap_mode,
                        "snap_target": op.snap_target})
    return options


def per_set_engine_overrides(rule, used_sets=None):
    """True when a set overrides an option the engine only takes globally (scale mode,
    pixel margin, prealign). Those can only be honored by running the engine once per
    set, which is what makes the difference between the two Sets Overlaid paths."""
    if rule is None:
        return False
    for entry in rule.sets:
        if used_sets is not None and entry.num not in used_sets:
            continue
        if not entry.use_overrides:
            continue
        if entry.override_scale or entry.override_pixel_margin or entry.override_prealign:
            return True
    return False


# ---------------------------------------------------------------- request building
def prepare_matching(state, params, bundles=False, follow_groups=False):
    """Fold the bundles and the follow groups into `state` before the request is built.

    A bundle stacks first and binds second: uvpack_cli merges islands that share a
    `bind_group` into one rigid body, so the geometry it receives has to be the stacked
    geometry -- the bundle is only a stack once the alignment was applied. Follow groups
    need nothing before the pack, only the match, which is replayed afterwards."""
    got, islands = state["collected"], state["islands"]
    if bundles:
        bundle_ids = {i["id"]: i.get("bundle", DEFAULT_BUNDLE) for i in islands}
        result = uv_similarity.bundles(islands, bundle_ids, params=params,
                                       default_bundle=DEFAULT_BUNDLE)
        state["bundles"] = result
        state["pre"] = dict(result.transforms)
        state["bind_groups"] = dict(result.bind_groups)
    if follow_groups:
        leaders = [i for i in islands if i.get("follow_group", DEFAULT_BUNDLE) != DEFAULT_BUNDLE]
        candidates = [i for i in got.followers()
                      if i.get("follow_group", DEFAULT_BUNDLE) != DEFAULT_BUNDLE]
        state["followers"] = uv_similarity.follow_groups(
            leaders, candidates, mode=params.mode,
            leader_groups={i["id"]: i["follow_group"] for i in leaders},
            follower_groups={i["id"]: i["follow_group"] for i in candidates})
    return state


def request_island(state, isl, entry=None):
    """One island as the engine takes it, stacked onto its bundle first when there is
    one. The geometry actually sent is kept in `state["sent"]`: a per-vertex match
    result pairs with the rings the packer *saw*, not with the ones on the mesh."""
    pre = state["pre"].get(isl["id"])
    source = uv_similarity.transform_island(isl, pre) if pre is not None else isl
    state["sent"][isl["id"]] = source
    out = island_payload(source, entry)
    bind = (state.get("bind_groups") or {}).get(isl["id"])
    if bind is not None:
        out["bind_group"] = bind
    return out


def island_payload(isl, entry=None):
    """One island as the engine takes it, with the set's per-island overrides."""
    out = {k: v for k, v in isl.items()
           if k in ("id", "outer", "holes", "parts", "area_3d", "group")}
    if entry is not None and entry.use_overrides:
        if entry.override_rotation:
            out["rotation"] = entry.allow_rotation
            out["rotation_step"] = entry.rotation_step
        if entry.scale_locked:
            out["scale_locked"] = True
    return out


# ---------------------------------------------------------------- pack operator
class UVPACK_OT_pack(bpy.types.Operator, MatchMixin):
    bl_idname = "uv.uvpack_pack"
    bl_label = "UVPack Islands"
    bl_options = {"REGISTER", "UNDO"}

    # The operator's own properties. They are what the UV menu's property dialog edits
    # and what a script that does not want the scene settings passes in.
    margin: bpy.props.FloatProperty(name="Margin (UV)", default=0.003, min=0.0, precision=4)
    tex_size: bpy.props.IntProperty(name="Texture size (0 = use UV margin)", default=0, min=0)
    pixel_margin: bpy.props.IntProperty(name="Pixel margin", default=4, min=0)
    rotation_step: bpy.props.IntProperty(name="Rotation step", default=90, min=0, max=180)
    mirroring: bpy.props.BoolProperty(name="Allow mirroring", default=False)
    scale_mode: bpy.props.EnumProperty(
        name="Scale",
        items=[("max", "Max scale", ""), ("fixed", "Fixed scale", ""),
               ("max_margin", "Fixed scale, max margin", "")], default="max")
    equalize_density: bpy.props.BoolProperty(name="Equalize texel density", default=False)
    bind_overlapping: bpy.props.EnumProperty(
        name="Bind overlapping",
        items=[("off", "Off", ""), ("touching", "Touching", ""), ("identical", "Identical", "")],
        default="off")
    snap_to_texels: bpy.props.BoolProperty(name="Snap to texels", default=False)
    snap_mode: bpy.props.EnumProperty(
        name="Vertex snap",
        items=[("bbox_min", "Bounding box corner", ""), ("bbox", "Bounding box", ""),
               ("outline", "Outline vertices", ""), ("all", "All vertices", "")],
        default="bbox_min")
    snap_target: bpy.props.EnumProperty(
        name="Snap to", items=[("corner", "Texel corner", ""), ("center", "Texel center", "")],
        default="corner")
    split_at_vertices: bpy.props.BoolProperty(name="Split at vertices", default=False)
    time_budget: bpy.props.FloatProperty(name="Search time (s)", default=2.0, min=0.0)
    only_selected: bpy.props.BoolProperty(name="Only selected faces", default=True)
    realtime: bpy.props.BoolProperty(
        name="Realtime solution", default=True,
        description="Show the latest layout while the search runs, and let ESC stop "
                    "the search and keep the best one so far")
    bundles: bpy.props.BoolProperty(
        name="Bundles", default=False,
        description="Stack the islands sharing a bundle onto each other and pack the "
                    "stack as a single bound body")
    follow_groups: bpy.props.BoolProperty(
        name="Follow groups", default=False,
        description="Unselected islands carrying a follow group follow the packing "
                    "transform of the matching selected island")
    cli_path: bpy.props.StringProperty(
        name="uvpack_cli path (override)", subtype="FILE_PATH", default="",
        description="Leave empty to use the add-on preference", options={"SKIP_SAVE"})
    use_scene_settings: bpy.props.BoolProperty(
        name="Use Scene Settings", default=False, options={"SKIP_SAVE"},
        description="Read every option from the UVPack sidebar instead of this dialog")

    # Set by execute() so callers (and the headless test) can read the packer result.
    last_scale = 0.0
    last_layout = None
    last_layouts = []
    last_request = None
    last_targets = []
    last_bundles = 0
    last_followers = 0

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    # ------------------------------------------------------------ gathering
    def _gather(self, context):
        """The islands to pack and the request(s) to send. Returns None after reporting,
        when there is nothing to pack."""
        if self.use_scene_settings:
            return self._gather_settings(context)
        return self._gather_simple(context)

    def _gather_simple(self, context):
        st = SimpleSettings(only_selected=self.only_selected,
                            split_at_vertices=self.split_at_vertices)
        # Follow groups have to see the unselected islands. The collector always extracts
        # every visible face and only *partitions* on the selection, so the candidates
        # are already there as the frozen ones -- nothing extra has to be read.
        got = collect(context, st,
                      with_geometry=self.needs_geometry() or self.follow_groups)
        if not got.meshes:
            self.report({"ERROR"}, "No mesh in edit mode")
            return None
        if not got.owners:
            self.report({"ERROR"}, "No faces to pack")
            return None
        if not got.islands:
            self.report({"ERROR"}, "No islands could be extracted from the selected faces")
            return None
        state = state_from(got, [], [[0, 0, 1, 1]])
        prepare_matching(state, self.match_params(),
                         bundles=self.bundles, follow_groups=self.follow_groups)
        request = {"target": [0, 0, 1, 1],
                   "islands": [request_island(state, i) for i in got.islands],
                   "options": simple_options(self)}
        state["requests"], state["request"] = [request], request
        return state

    def _gather_settings(self, context):
        st = settings(context)
        if st is None:
            self.report({"ERROR"}, "UVPack scene settings are not registered")
            return None
        if not [o for o in context.objects_in_mode_unique_data if o.type == "MESH"]:
            self.report({"ERROR"}, "No mesh in edit mode")
            return None

        overlaid = st.pack_mode == "SETS_OVERLAID" or st.sets_overlaid
        grouped = st.pack_mode in SET_MODES or st.sets_adjacent or overlaid
        rule = st.rule() if grouped else None
        params = uv_similarity.MatchParams(mode=st.match_mode, threshold=st.match_threshold,
                                           resolution=st.resolution)
        got = collect(context, st, rule,
                      with_geometry=st.match_mode != "outline" or st.follow_groups)
        if not got.owners:
            self.report({"ERROR"}, "No faces to pack")
            return None
        if not got.islands:
            self.report({"ERROR"}, "No islands to pack (all of them are pinned or unselected)")
            return None

        used_sets = {isl["group"] for isl in got.islands}
        targets, groups = build_targets(st, context, rule, used_sets)
        obstacles = [dict(p) for p in got.obstacles] if got.obstacles else None
        state = state_from(got, [], targets, st=st, rule=rule, used_sets=used_sets)
        prepare_matching(state, params, bundles=st.bundles, follow_groups=st.follow_groups)

        requests = []
        if overlaid and rule is not None and per_set_engine_overrides(rule, used_sets):
            # One engine run per set. The engine's own `overlay_groups` would stream the
            # cumulative layout, but it takes one set of global options for the whole
            # run -- so a set that overrides the scale mode, the pixel margin or the
            # prealign flag can only be honored by giving it a run of its own.
            for num in sorted(used_sets):
                entry = rule.set_by_num(num)
                members = [i for i in got.islands if i["group"] == num]
                if not members:
                    continue
                regions = [r.as_list() for r in entry.regions if r.is_valid()] if entry is not None else []
                request = {"targets": regions or targets,
                           "islands": [request_island(state, i, entry) for i in members],
                           "options": engine_options(st, context, entry)}
                if obstacles:
                    request["obstacles"] = [dict(p) for p in obstacles]
                requests.append(request)
        else:
            entry = None
            if rule is not None and st.pack_mode == "SETS_TO_TILES" and len(used_sets) == 1:
                entry = rule.set_by_num(next(iter(used_sets)))
            options = engine_options(st, context, entry)
            if groups:
                options["groups"] = groups
            if overlaid:
                # The engine's own per-set search: one process, one stream, each set
                # over its own regions at its own scale.
                options["overlay_groups"] = True
            if st.pack_mode == "SETS_ADJACENT" or (
                    st.pack_mode == "SETS_TO_TILES" and st.sets_adjacent):
                options["group_adjacent"] = True
                options["group_cohesion"] = st.set_cohesion
            request = {"islands": [request_island(state, i,
                                                  rule.set_by_num(i["group"]) if rule else None)
                                   for i in got.islands],
                       "options": options}
            if len(targets) == 1 and not groups:
                request["target"] = targets[0]
            else:
                # Set-to-tile restrictions index into "targets", so the list has to go
                # out even when it holds a single region.
                request["targets"] = targets
            if obstacles:
                request["obstacles"] = obstacles
            requests.append(request)
        state["requests"], state["request"] = requests, requests[0] if requests else None
        return state

    # ------------------------------------------------------------ applying
    @staticmethod
    def _apply(state, layout):
        """Write one layout onto the mesh. Always starts from the UVs the islands were
        extracted from, so applying an improved layout over an earlier one -- which is
        what a streamed run does on every event -- never compounds the transforms.
        Returns the number of faces written."""
        by_id = {isl["id"]: isl for isl in state["islands"]}
        sent, pre = state.get("sent") or {}, state.get("pre") or {}
        snap_report = layout.get("snap_to_texels") or {}
        entries, placements = {}, {}
        for item in layout.get("islands", []):
            # The vertex output of a snap mode pairs with the rings the packer *saw*,
            # which for a bundled island is the stacked copy, not what is on the mesh.
            island = sent.get(item["id"]) or by_id.get(item["id"])
            if island is None:
                continue
            entries[item["id"]] = (item["transform"],
                                   uv_islands.vertex_map(item, island),
                                   uv_islands.texel_snapper(item, snap_report),
                                   pre.get(item["id"]))
            placements[item["id"]] = uv_similarity.Transform.from_cli(item["transform"])
        owners, original = state["owners"], state["original"]
        moved = 0
        for isl in state["islands"]:
            entry = entries.get(isl["id"])
            if entry is None:
                continue
            transform, vmap, snap, before = entry
            for f in isl["faces"]:
                face, uv_layer = owners[f]
                source = original[f]
                for loop, uv in zip(face.loops, source):
                    # The packer saw the stacked island, so its transform applies after
                    # the stacking; the loops still hold the original UVs.
                    start = before.apply(uv) if before is not None else uv
                    loop[uv_layer].uv = uv_islands.map_uv(start, transform, vmap, snap)
                moved += 1
        # Follow groups: an island the pack never touched replays its leader's motion in
        # its own frame (M^-1 . P . M), which is what makes it follow.
        followed = 0
        for match in state.get("followers") or ():
            leader = placements.get(match.leader_id)
            if leader is None:
                continue
            before = pre.get(match.leader_id)
            replay = match.replay(leader.compose(before) if before is not None else leader)
            for f in match.follower["faces"]:
                face, uv_layer = owners[f]
                for loop, uv in zip(face.loops, original[f]):
                    loop[uv_layer].uv = replay.apply(uv)
            followed += 1
        state["followed"] = followed
        for obj, _ in state["meshes"]:
            bmesh.update_edit_mesh(obj.data)
        return moved

    @classmethod
    def _apply_all(cls, state, layouts):
        """Several layouts (one per set, in the one-run-per-set path) as if they were
        one: every island appears in exactly one of them."""
        if len(layouts) == 1:
            return cls._apply(state, layouts[0])
        merged = {"islands": [i for l in layouts for i in l.get("islands", [])],
                  "snap_to_texels": next((l.get("snap_to_texels") for l in layouts
                                          if l.get("snap_to_texels")), {})}
        return cls._apply(state, merged)

    def _finish(self, context, state, layouts, moved, note=""):
        """Record the result, report it, and do the sidebar's after-pack chores."""
        layouts = [l for l in layouts if l]
        type(self).last_layouts = layouts
        type(self).last_layout = layouts[0] if layouts else None
        type(self).last_request = state["requests"][0] if state["requests"] else None
        type(self).last_targets = [list(t) for t in state["targets"]]
        type(self).last_scale = float(layouts[0].get("scale", 0.0)) if layouts else 0.0
        result = state.get("bundles")
        type(self).last_bundles = len(result.bundles) if result else 0
        type(self).last_followers = state.get("followed", 0)

        st, rule = state.get("st"), state.get("rule")
        transforms = {i["id"] for l in layouts for i in l.get("islands", [])}
        if st is not None:
            set_overlay_targets(state["targets"], st, rule, state.get("used_sets"))

        rejected = sum(l.get("input", {}).get("rejected", 0) for l in layouts)
        unplaced = sorted({i for l in layouts for i in l.get("unplaced", [])})
        invalid = [i["id"] for i in state["islands"] if i["id"] not in transforms]
        if st is not None and st.overlay_select_invalid and invalid:
            select_faces(state["collected"], invalid)
        scales = ", ".join(f"{l.get('scale', 0):.4f}" for l in layouts) or "0"
        summary = (f"packed {len(transforms)} islands ({moved} faces) at scale {scales}{note}"
                   + (f", {type(self).last_bundles} bundles" if type(self).last_bundles else "")
                   + (f", {type(self).last_followers} followers followed"
                      if type(self).last_followers else "")
                   + (f", {state['frozen']} islands left in place" if state["frozen"] else ""))
        if layouts and not all(l.get("valid") for l in layouts):
            failure = next((l.get("failure") or l.get("error") for l in layouts if not l.get("valid")), "")
            self.report({"WARNING"}, f"packing not fully valid: {failure}")
        elif rejected or unplaced or invalid:
            # Those faces keep the UVs they had, so they sit wherever they were --
            # usually on top of the freshly packed layout. Say so loudly rather than
            # leaving a silently half-packed result behind.
            self.report({"WARNING"}, f"{summary}; {rejected} islands rejected, {len(unplaced)} "
                                     f"unplaced, {len(invalid)} islands left at their old UVs")
        else:
            self.report({"INFO"}, summary)

    # ------------------------------------------------------------ entry points
    def _wants_stream(self, context):
        """The realtime stream needs a window and an event loop; in background mode
        (the headless test, a --python script) there is neither, so pack in one go."""
        if bpy.app.background or not context.window_manager or not context.window:
            return False
        st = settings(context) if self.use_scene_settings else None
        return bool(st.stream if st is not None else self.realtime)

    def execute(self, context):
        cli = bpy.path.abspath(self.cli_path) if self.cli_path else get_cli_path(context)
        state = self._gather(context)
        if state is None:
            return {"CANCELLED"}
        # Only a single request can be streamed: the one-run-per-set path would need as
        # many sessions as there are sets, and the engine's own overlay_groups already
        # streams the cumulative layout for the case that matters.
        if len(state["requests"]) == 1 and self._wants_stream(context):
            return self._start_stream(context, cli, state)
        return self._run_blocking(context, cli, state)

    def _run_blocking(self, context, cli, state):
        layouts = []
        for request in state["requests"]:
            layout, error = run_cli(cli, request)
            if layout is None:
                self.report({"ERROR"}, error)
                return {"CANCELLED"}
            layouts.append(layout)
        moved = self._apply_all(state, layouts)
        self._finish(context, state, layouts, moved)
        return {"FINISHED"}

    # ------------------------------------------------------------ streaming path
    # uvpack_cli streams one JSON line per improved layout and one final "done" line.
    # The operator reads stdout without blocking on a timer and redraws the UV editor
    # for the latest improvement, so the search is visible while it runs. ESC signals
    # its request-owned cancel file; the search stops and still writes its done line --
    # so cancelling keeps the best layout found so far instead of throwing it away.
    def _start_stream(self, context, cli, state):
        try:
            session = StreamSession.start(cli, state["requests"][0])
        except Exception as ex:  # noqa: BLE001
            self.report({"ERROR"}, f"could not run uvpack_cli at '{cli}': {ex}")
            return {"CANCELLED"}
        self._session = session
        self._state = state
        self._moved = 0
        self._timer = context.window_manager.event_timer_add(0.05, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            self._session.interrupt()
            return {"RUNNING_MODAL"}
        if event.type != "TIMER":
            return {"RUNNING_MODAL"}

        status = self._session.pump(lambda layout: self._draw(context, layout))
        if status == "running":
            return {"RUNNING_MODAL"}
        if status == "done":
            layout = self._session.layout
            moved = self._apply(self._state, layout)
            note = f" ({self._session.improved} realtime updates"
            note += ", cancelled)" if layout.get("cancelled") else ")"
            state = self._state
            self._cleanup(context)
            self._finish(context, state, [layout], moved, note)
            return {"FINISHED"}
        # "failed": the process ended without a result line, or the interrupt was
        # ignored. Whatever was streamed is already on the mesh, so keep it.
        detail, improved, state = self._session.error, self._session.improved, self._state
        moved = self._moved
        self._cleanup(context)
        if improved:
            self.report({"WARNING"}, f"uvpack_cli stopped early ({detail}); kept the best of "
                                     f"{improved} streamed layouts")
            self._finish(context, state, [{"scale": 0.0, "islands": [], "valid": True}], moved,
                         f" from {improved} realtime updates")
            return {"FINISHED"}
        self.report({"ERROR"}, f"uvpack_cli produced no layout: {detail}")
        return {"CANCELLED"}

    def _draw(self, context, layout):
        """Put one streamed layout on the mesh and ask the editors to repaint."""
        self._moved = self._apply(self._state, layout)
        screen = getattr(context, "screen", None)
        for area in (screen.areas if screen is not None else ()):
            if area.type in {"IMAGE_EDITOR", "VIEW_3D"}:
                area.tag_redraw()

    def _cleanup(self, context):
        timer = getattr(self, "_timer", None)
        if timer is not None:
            context.window_manager.event_timer_remove(timer)
            self._timer = None
        session = getattr(self, "_session", None)
        if session is not None:
            session.close()
            self._session = None

    def cancel(self, context):
        # Blender calls this when the modal operator is torn down without finishing.
        session = getattr(self, "_session", None)
        if session is not None:
            session.interrupt()
        self._cleanup(context)

    def invoke(self, context, event):
        if self.use_scene_settings:
            return self.execute(context)
        return context.window_manager.invoke_props_dialog(self)


class UVPACK_OT_pack_scene(bpy.types.Operator):
    """Pack with the options from the UVPack sidebar."""
    bl_idname = "uv.uvpack_pack_scene"
    bl_label = "Pack"
    bl_options = {"REGISTER", "UNDO"}

    others_mode: bpy.props.EnumProperty(
        name="Unselected Islands", options={"SKIP_SAVE"}, default="SETTINGS",
        items=[("SETTINGS", "From Settings", "")] + [(i, l, "") for i, l, _d in OTHERS_MODES])

    @classmethod
    def poll(cls, context):
        return context.mode == "EDIT_MESH"

    def execute(self, context):
        st = settings(context)
        saved = None
        if st is not None and self.others_mode != "SETTINGS":
            saved, st.others_mode = st.others_mode, self.others_mode
        try:
            # The islands are collected inside this call, before it can go modal, so the
            # restore below still happens after the setting was read.
            return bpy.ops.uv.uvpack_pack("EXEC_DEFAULT", use_scene_settings=True)
        finally:
            if saved is not None:
                st.others_mode = saved


classes = (UVPACK_OT_pack, UVPACK_OT_pack_scene)
