# CLI and geometry reference

## Read and write layouts

The CLI accepts a UTF-8 JSON request from a file or standard input. The default
operation is `pack`. Use `operation: "validate"` to check placed polygons or
`operation: "normalize"` for chart-scale normalization. Query `uvpack_cli --version`
for the compiled worker flag, supported operations, wire dialect, API versions,
capabilities and output precisions. The Python companion exposes the same source
metadata through `build_info()`.

A request contains an `islands` array. Each island can specify an integer `id`,
an `outer` ring, `holes` and additional rigid polygon `parts`. Each vertex is a
pair of UV coordinates. Use a target rectangle `[umin, vmin, umax, vmax]`, or
supply multiple rectangles through `targets`. For example:

```json
{
  "targets": [[0, 0, 1, 1], [1, 0, 2, 1]],
  "islands": [
    {"id": 7, "group": 1, "outer": [[0, 0], [0.3, 0], [0.3, 0.2], [0, 0.2]]},
    {"id": 8, "outer": [[0, 0], [0.2, 0], [0, 0.4]]}
  ],
  "options": {"groups": {"1": {"tiles": [1]}}, "margin": 0.01, "output_precision": "float32",
              "require_complete": true, "threads": 1, "trials": 2}
}
```

A complete valid result has `valid: true` and `complete: true`. Its `islands`
array contains per-ID placement, tile, and row-major transform data. With a
transform `[a, b, c, d, e, f, 0, 0, 1]`, map an original UV `(x, y)` to
`(a*x + b*y + c, d*x + e*y + f)`. Read `unplaced` and the input/validation
diagnostics for partial results. An unsuccessful operation returns exit 2.
Malformed input returns exit 1 with a JSON error. Successful requests return 0.

## Input limits

Input must be UTF-8 JSON. Files and stdin are limited to **64 MiB**, with at most
**4,194,304 JSON values** (containers count) and a maximum value depth of **128**
(the root has depth zero). These limits are checked before unbounded allocation
or recursion. The parser rejects duplicate keys, invalid escapes/UTF-8, unpaired
surrogates, non-JSON numeric spellings, overflow and nonzero numbers that underflow
to zero. Representable subnormal doubles remain supported. Unknown optional
metadata can still be carried by a request; supported fields must have their
declared types. An explicit `null` does not select a field's default.

The writer emits finite numbers and correctly escaped UTF-8 independently of the
host's numeric locale. Malformed input returns exit **1** with one JSON error
object. If a well-formed request has already selected streaming, that error is a
`done` record; syntax errors can occur before streaming is recognized. Result
exits **0** (valid) and **2** (unsuccessful) retain their structured bodies.
Requests are checked before geometry repair/search. Integer fields reject
fractional decimals even if binary floating-point rounding would erase the
fraction. IDs and group/cluster numbers must be exact integers within
`±(2^53 − 1)` and the field's machine type; island/group IDs and tile indices are
nonnegative, and seeds fit `uint32_t`. Group-map keys use canonical unsigned
decimal text (no sign, leading zeros or suffix). Tile indices must name a supplied
target. Negative per-island rotation and density-group sentinels remain supported.

The CLI enforces the following limits.

| Input/control | Limit |
| --- | --- |
| Islands; obstacles | 65,536 each |
| Vertices, including holes, parts, envelope points and obstacles | 1,048,576 combined |
| Rings; polygon parts | 262,144 each, combined across the request |
| Targets; tile references in one group | 4,096 each |
| Explicit `threads`, `compaction_threads`; `sub_rows` | 256 each |
| `trials`, `gpu_trials`, `max_evaluations`, `bracket_rounds`, compaction sweep/round caps | 65,536 each |
| Search, compaction and stall budgets | 0–86,400 seconds each |
| Target height divided by `stripe_height` | At most 65,536 rows for packing |
| Target width multiplied by `target_aspect` | At most `(2^31 − 1) / 2^20` output units for the fixed-point raster |

All numeric controls are finite. Gaps, areas, densities, texture size, budgets
and extra obstacle margins are nonnegative; scale, aspect, stripe height and
per-island multipliers are positive. Negative `border_margin` retains the packing
sentinel “same as margin”; validation requires a nonnegative border. Pixel margins
and enabled snapping require a positive texture size. Cohesion, bracket start,
compaction delta and tolerances are in `[0,1]`; single-pass start and initial step
are in `(0,1]`, and single-pass margin reserve is at least one. Existing zero/default
controls remain supported, including automatic threads and zero tolerances.

## Packing options

| Option | Meaning |
| --- | --- |
| `scale_mode` | `max` (default): largest uniform scale. `fixed`: pack at `scale`. `max_margin`: keep `scale`, find the largest gap. |
| `margin` | Required island-to-island gap in output units. Each island is expanded by half of it before placement. |
| `border_margin` | Island-to-target-border gap; negative means the same as `margin`. |
| `stripe_height` | Row spacing of the occupancy representation. Smaller is tighter and slower. |
| `fast_search` | `true` (default): indexed occupancy and constant-time row cost bounds in the greedy search. `false` falls back to the plain interval search, with the same placement rules. |
| `trials`, `threads` | Greedy attempts per scale, run in parallel rounds. |
| `time_budget` | Seconds for the refine stage after the bracket. 0 disables it. |
| `compaction_time` | Extra seconds for the overlap-tolerant compact stage that runs after the refine rounds. 0 disables it. |
| `compaction_rotation` | Continuous per-island rotation during compaction (default on). Off keeps every island on the angle the greedy chose. |
| `compaction_polish`, `compaction_delta` | Run a zero-growth slack pass before growing (default on); first relative scale step of the growth loop (default 0.004, scaled down when there are many islands). `compaction_delta` 0 means polish only, which is what `scale_mode: "fixed"` uses. |
| `compaction_threads` | Independent compaction attempts per round, each on its own thread with its own step from a ladder around the current one. 0 (default) uses `threads`; 1 is the single-threaded loop. |
| `compaction_attempt_sweeps`, `compaction_max_rounds` | Reproducible runs: bound an attempt by local-search sweeps and the stage by rounds instead of by the clock. Both 0 (default) means "spend the wall-clock budget", which is not reproducible. |
| `seed` | Fixes the trial orders. Same seed and thread count give the same layout, except when a wall-clock budget cuts a round short. |
| `validate` | Run the built-in continuous validator on the result (default on). |
| `sub_rows` | Sub-stripe placement resolution: an island may start at k/`sub_rows` of a stripe. 8 wins back nearly all of the row-quantization loss at a small cost. |
| `parallel_bracket` | Evaluate one scale per thread per round during the scale bracket. |
| `bracket_rounds`, `bracket_start`, `initial_step`, `max_evaluations` | Search tuning knobs, best left alone: bracket rounds (default 3), where the bracket starts as a fraction of the area bound, the first relative step of the refine stage, and a cap on scale evaluations (0 = none). |
| `targets`, `tile_fill`, `groups` | Several target boxes at one shared scale. `sequential` (default) fills a box before moving on; `any` lets the cost decide. A group's islands (`group` on the island) are restricted to the listed tiles. |
| `group_adjacent`, `group_cohesion` | Pack the islands of a group next to each other. `group_cohesion` in [0, 1] trades cohesion against coverage: 0 is a mild preference, 1 a strong one. |
| `fill` | `free` (default, the plain search), `width_first` (fill the width, grow upward), `height_first` (fill the height, grow rightward), `proportional` (grow towards the target's own shape). |
| `origin` | `bottom_left` (default, a no-op), `bottom_right`, `top_right`, `top_left`: mirror the finished layout inside the target. The scale never changes; `mirrored` and `angle` on each island compose with it. |
| `overlay_groups` | `scale_mode: "max"` only (there is no per-group scale to search for at a fixed one). Run the whole search once per `group`, each over the full target(s) with its own scale, and combine the results. Groups overlap by design: the result carries `"overlaid_groups": true` and validators must only compare islands of the same group. `stats.group_scales` lists what each group reached and `scale` is the smallest of them. A `time_budget` or `compaction_time` is split evenly between the groups. |
| `shelve_unplaced` | With `allow_partial`, lay the islands that did not fit in rows above the target and emit transforms for them, flagged `"shelved": true`. They stay listed under `unplaced`. |
| `stall_timeout` | Seconds without an improvement after which the refine stage stops instead of spending the rest of `time_budget`. 0 (default) disables it. |
| `obstacles` | Static polygons in target space that islands must avoid, with the island gap plus `extra_margin`. |
| `tex_size`, `pixel_margin`, `pixel_border_margin` | Margins in texels of a texture of `tex_size`; they override the UV-unit margins. |
| `snap_to_texels` | After packing, pull each island onto the texel grid of `tex_size`. The packer reserves one texel of gap and half a texel of border so the snapped layout still honors the requested margin; the result is revalidated. |
| `snap_mode` | Which vertices move: `bbox_min` (default, the bounding-box minimum, a rigid translation), `bbox` (both bounding-box edges, which needs a slight per-island non-uniform scale), `outline` (every boundary vertex, hole borders included), `all` (every vertex). See "Snap to texels". |
| `snap_target` | `corner` (default): multiples of `1 / tex_size`. `center`: texel centres, `(k + 0.5) / tex_size`. |
| `stream` | Write one JSON line per improved layout while the search runs, then a final `done` line. Off: the output is one JSON object, exactly as before. |
| `target_aspect` | Width/height of a non-square texture. Packing happens in the square space; gaps are isotropic texel distances there. |
| `allow_mirroring`, per-island `flip` | Mirrored orientations (use only when the texture is symmetric). |
| Per-island `rotation`, `rotation_step`, `scale_locked`, `pre_scale` | Rotation lock or step override, keep the island at scale 1, apply a pre-scale. |
| `equalize_density` with `area_3d` | Texel-density normalization: pre-scale islands so UV area per 3D area is uniform. |
| `density_space` | `local` (default) reads each island's `area_3d`, `global` reads `area_3d_global`. Which 3D area the caller measured, nothing more. |
| Per-island `density_group`, `scale_multiplier` | Normalize only inside a group (islands without one stay untouched); multiply one island's scale afterwards (1.0 = none). |
| `texel_density` with `tex_size` | Pre-scale every island with a 3D area to this texel density, then pack at a fixed scale of 1. Islands without a 3D area keep their scale. |
| `scale_policy` | `shared` (default), `per_group`, `custom` (per-group `scale_cluster`) or `by_tiles`: how many uniform scales one run may produce. |
| `shrink_spanning` | During the refine stage, shrink islands that nearly span the target so one of them stops capping the scale of the rest. Needs `time_budget`. |
| `bind_overlapping`, `bind_group` | `touching` binds input islands that overlap, `identical` binds identical stacked islands, a shared `bind_group` binds explicitly. Bound islands move rigidly. |
| `allow_partial` | Fixed-scale modes place what fits and list the rest under `unplaced`. |
| `strict_input` | Input issues become rejections instead of repairs. Rejected and repaired islands are listed under `input`. |
| `progress` | Print refine rounds to stderr. SIGINT/SIGTERM return the best layout found so far. |
| `prealign` | `true` (default): lay each island on the axes of its own minimum-area rectangle before the rotation steps. `false` keeps it in the orientation it arrived in. |
| `single_pass`, `single_pass_margin_reserve`, `single_pass_start` | One open greedy pass with a uniform rescale afterwards; gaps stay exact through the reserve. |
| `extent_estimate`, `estimate_tolerance` | Estimate the scale from one open pass before the bracket; the tolerance sets how many bounded rounds follow (0.05 = one). |
| `dedup_orientations` | Drop orientations with identical rows (safe; default on). |

## Streaming and cancellation

`"stream": true` turns the single JSON object into a stream of JSON lines:

```text
{"event":"improved","dialect":"uvpack","scale":0.9398,"elapsed":0.027,"islands":[{"id":0,...,"transform":[...]},...]}
{"event":"improved","dialect":"uvpack","scale":0.9504,"elapsed":0.048,"islands":[...]}
...
{"event":"done","dialect":"uvpack","valid":true,"scale":0.9834,"islands":[...],"validation":{...},"stats":{...}}
```

One `improved` line every time the search commits a better layout -- each round of
the bracket stage, each successful refine round, each compact accept -- and
exactly one `done` line at the end carrying the whole result described above. The
lines are flushed as they are written, so a UI can draw each layout while the search
keeps running; `elapsed` is seconds since the pack started. The `islands` array of an
`improved` line has the same shape as the result's, so a client applies either with
the same code. Scales never go backwards.

For packing or validation, `uvpack_cli --cancel-file PATH request.json` watches a
caller-owned file. Create that file to cancel this request on Windows or POSIX;
give concurrent requests different paths. The CLI neither creates nor removes the
signal file. A pre-existing file prevents search or certification. SIGINT/SIGTERM remain supported
by the CLI's signal adapter. Cancellation joins its monitor and packing workers
before returning, and streaming still emits exactly one final `done` record.
`"cancelled": true` can accompany a valid, revalidated incumbent (exit 0); without
one the result is invalid (exit 2). Protocol/internal errors use exit 1. Without
streaming, the result remains one object with no `event` key. The CLI advertises
`request_cancellation_v1` and `cancel_file_v1` in its capabilities.

Validation-only requests use the same monitor and emit one final `done` record
when `options.stream` is true. An interrupted validation has `valid=false`,
`complete=false`, `cancelled=true` and `validation.cancelled=true` (exit 2), with
no partial geometry certificate or measured minimum gap/border. A fully completed
certificate remains valid if cancellation arrives afterward during monitor
shutdown. Bounded JSON parsing/schema checks finish before geometry cancellation;
cancel-file polling has a 10 ms interval, subject to OS scheduling.

Native callers use `PackOptions::cancel`, a `CancellationToken` with `request()`
and `requested()` methods. Copies share one request's state. Construct a new token
for the next request; cancelled tokens are not reset. Separately constructed
options are independent, including concurrent calls. `Layout::cancelled` records
cancellation independently of `valid`. Checks run during preparation, candidate
row traversal, search rounds and compaction. Cancellation can wait for an
in-flight GPU dispatch or indivisible geometry operation to finish.

For stored-geometry validation, use `ValidationOptions::cancel` with the same
token semantics. Checks include ring self-intersections, hole containment,
pairwise polygon distances and measured clearance. `ValidationReport::cancelled`
means the report is incomplete and `ok` is false. The older `validate_layout` and
`validate_polygons` interfaces retain their synchronous behavior; packing's final
incumbent validation still has to finish before returning its certificate.

The old `request_cancel()`/`cancel_requested()` functions are a deprecated global
adapter. They affect native calls only when the caller explicitly assigns
`options.cancel = legacy_cancellation_token()`. They cannot cancel default options
or poison a later independent request. The CLI uses its own token and monitor.

The callback behind this is `PackOptions::on_improved`. It is invoked only from the
thread that called `pack_max_scale` (the search's own accept points and the
compact stage's, never a worker thread and never concurrently), so it needs no
locking -- but it runs inside the search, so it has to be cheap.

## Snap to texels

`"snap_to_texels": true` with a `tex_size` snaps the packed islands onto the texel grid
so a baked texture has no half-texel bleed at the island borders. `snap_mode`
picks what moves:

| Mode | What moves | Output |
| --- | --- | --- |
| `bbox_min` | The island, rigidly, so its bounding-box minimum lands on the grid. | `transform` |
| `bbox` | The island, stretched slightly and independently in u and v, so both bounding-box edges land on the grid. | `transform` (no longer a similarity) |
| `outline` | Every boundary vertex on its own: the outer ring of every part and every hole ring. | `vertices` |
| `all` | Every vertex. | `vertices` plus `snap_interior` |

No vertex moves more than half a texel on either axis in any mode, which is what the
reserved texel of gap covers. The packer is handed island *outlines*, so at this
level `all` snaps the same vertices `outline` does; what it adds is `snap_interior`,
telling the client to snap the interior vertices it owns and the packer never saw
(`uv_islands.map_uv` and the Blender operator both do).

`outline` and `all` produce geometry no affine map can express, so the result carries
per-island `vertices`: `outer`, `holes`, and `parts`, following the island's **input**
rings one vertex for one, so a client pairs them positionally. The `transform` is
still emitted for anything not listed there. That emitted geometry -- not the
transform's image -- is what the built-in validator checks (`validate_polygons`), and
an island whose snapped geometry would break containment, an overlap or the requested
gap is reverted on its own and reported under `snap_to_texels.fallbacks`; the rest keep
their snap. Bound islands (`bind_overlapping`, `bind_group`) share one placement and
have no per-member vertex list, so a vertex mode downgrades to `bbox_min` for them and
says so in `snap_to_texels.downgraded`.

## Python mesh companion

```python
from uvpack_geometry import extract_mesh_islands, apply_mesh_layout

extraction = extract_mesh_islands(
    uvs, face_starts, face_sizes, corner_verts, chart_ids=chart_ids,
    face_ids=face_ids, corner_ids=corner_ids,
    face_area_uv=face_area_uv, face_area_3d=face_area_3d,
)
# Pass island outlines and original area data to the native packer.
# Check extraction.issues first; recovery requires a native policy.
packed_uvs = apply_mesh_layout(uvs, extraction, native_layout, precision="float32")
```

Face offsets partition the corner arrays contiguously; each face has at least
three corners. IDs are nonnegative, exactly representable integers; face and
corner IDs are unique. Supplied chart IDs define rigid islands. Otherwise only
source edges with exactly matching UV endpoints connect faces. Coincident UVs on
disconnected geometry never join islands. Every face/corner retains ownership,
including degenerate members; entirely degenerate charts are reported explicitly.
No hull is silently substituted and tiny positive areas are not discarded.

Per-face areas should come from the caller's canonical triangulation. If supplied,
the companion records original area sums and the density cross sum needed by the
uniform-layout density operation.

Application requires exactly one finite affine transform per island and unchanged
input UVs. Responses may be reordered. Snapped boundary vertices use ring/corner
identity, with the transform for interior corners (and interior texel snapping
when requested). Output can be quantized to float32; final native geometry and
stored-layout validation remain separate required steps.

Duplicate directed boundary use is reported as `ambiguous_boundary`.
Self-touching or crossing walks within a source component yield `no_outline`.
The returned IDs and original envelope points let the caller apply an explicit
recovery policy while preserving source membership.

## Explicit native input recovery

`check_and_repair(islands, InputPolicy{...})` adds opt-in `hull_policy` (`reject`
by default, or `convex_hull`). `Island::envelope_points` supplies original chart
corners when no usable outline exists. The policy validates finite source points,
rejects duplicate IDs before mutation, retains rejected/recovered IDs in the
report, and checks that accepted envelopes contain the supplied corners. Strict
input checking never permits hull substitution. A hull is a conservative packing
envelope; it does not repair folded UVs or certify the original surface.
Non-adjacent boundary contacts, including repeated vertices and endpoints on
another edge, count as self-intersections during input checking. Their rejected
or explicitly recovered IDs appear in the input report before packing; the
stored-geometry validator still checks the original emitted surface separately.

The CLI accepts `options.hull_policy` and `options.require_complete` (default
false for compatibility). Input reports contain `complete`, `rejected_ids`,
`hull_ids`, and per-ID issues/reasons. The top-level `complete` additionally
requires every original ID to be placed, unshelved, in a valid layout. With
`require_complete=true`, input rejection stops packing and returns exit 2;
the primary failure names the rejected island and input reason, rather than a
missing output from the search that was skipped. Placement loss also makes the
result unsuccessful. Legacy partial clients can
still receive a valid subset, now explicitly marked `complete=false`.

Hull recovery supports rigid snapping and unsnapped placement. The CLI rejects
its combination with per-vertex outline or all-vertex snapping.

## Uniform layout density

`set_uniform_layout_density(items, target, tex_w, tex_h)` scales every movable item's
`pre_scale` by one common factor. Its current density is
`sqrt(tex_w*tex_h) * sum(pre_scale*density_cross_sum) / sum(area_3d)`.
`density_cross_sum` is the sum of `sqrt(face_uv_area*face_surface_area)` over
original faces. It cannot be reconstructed from per-island area totals. Inputs
must be finite, nonnegative and consistent with the Cauchy bound (64 double
epsilons of relative tolerance). Invalid input fails before mutating any scales.
`DensityIsland::fixed` preserves that item's existing scale in all density
operations. In uniform mode, fixed items contribute to the whole-layout mean;
movable items supply the remainder. An impossible target throws `domain_error`
without changing any scales. If there is no movable density contribution, the
current value must match the target within 64 double epsilons of relative
tolerance. Equalization excludes fixed items from both the mean and rescaling.

CLI `options.density_mode="uniform_layout"` selects this operation; the existing
`per_island` mode remains the default. Supply `area_uv`, `area_3d` and
`density_cross_sum` per island, or the matched `_global` surface/cross fields with
`density_space="global"`. `area_uv` always measures original source triangles,
independently of the packing envelope. Bound groups sum these moments; unequal
pre-scales within a bound group are rejected.

Uniform mode requires a positive target, fixed pack scale 1, complete placement,
unit multipliers and no equalization, shrinking, scale locks or per-group scaling.
Only rigid `bbox_min` texel snapping is permitted. Contradictory requests fail;
an oversized valid layout returns no fit without lowering density. The result's
`density` record reports mode, requested density, common factor and achieved
density (null unless the full layout is valid). The caller must validate actual
stored coordinates after applying the result.

## Normalize charts before host packing

`operation="normalize"` exposes the native density controls without a packing
search. It accepts the original chart records from `extract_mesh_islands`,
including a unique `id`, nonempty `envelope_points` containing all original
corners, and original `area_uv` and `area_3d`. Surface areas may be zero when
unknown; equalization leaves those charts unchanged. Set `fixed:true` on a chart
to preserve its coordinates. All corners of each chart receive one affine
transform about its bounding-box center, so exact shared coordinates stay shared.
This does not invoke host normalization or correct individual corners afterward.

Choose exactly one normalization policy:

- `options.equalize_density=true` equalizes movable charts' UV area per surface
  area while preserving their total UV area. Existing `density_group` semantics
  apply within that movable subset.
- `options.density_mode="uniform_layout"`, positive `tex_size` and
  `texel_density` request a whole-layout, area-weighted face density. Each chart
  also requires `density_cross_sum`. Fixed charts contribute their original
  density; one common positive factor scales the movable charts. Targets that
  require a zero or negative movable scale fail explicitly.

Optional `density_space="global"` selects `area_3d_global` and
`density_cross_sum_global`. `target_aspect` defines a texture of dimensions
`tex_size*target_aspect` by `tex_size`. `output_precision` is `float64` (default)
or `float32`; transformed original corners must fit the declared storage range.
`pre_scale` and `scale_multiplier`, if supplied, must be one; normalization starts
from the supplied original coordinates. Uniform mode rejects density groups.
Unsupported normalization options are rejected rather than silently ignored.

Success returns exit 0 with `operation:"normalize"`, `accepted:true`,
`complete:true`, `fixed_ids`, `output_precision` and one `id`/`transform` entry per
input chart. Apply that result with `apply_mesh_layout(..., precision=...)`.
Fixed charts receive the identity transform. The `density` record contains the
mode, common factor (one for equalization), and `achieved`: an analytic value from
the original face moments and scales, or null without texture size/usable moments.
It is not a measurement of rounded output. Normalization emits no `valid` or
`validation` packing certificate: charts can overlap or lie outside the tile.
The host must check actual stored triangle coverage, density and the eventual
packed layout.

Infeasible fixed density, coordinate overflow or cancellation returns exit 2,
`accepted:false`, `complete:false`, an empty `islands` list and a `failure`
diagnostic. Malformed input returns exit 1. Signals and `--cancel-file` use the
same cancellation mechanism as packing. `stream:true` emits only the final
`done` event. `fixed` applies to normalization; to preserve those charts during
packing, hold them out of the movable island list and submit their footprints
as obstacles. Validate the combined stored layout afterward.

## Check an existing stored layout

`uvpack_cli` accepts `operation="validate"` with placed `islands` (original chart
`outer`, `holes`, `parts`), a target or targets, and options `margin`,
`border_margin`, `target_aspect`, `output_precision` (`float64` or `float32`).
Pixel margin options use the existing `tex_size` convention. This operation does
not invoke input repair or packing and returns no replacement coordinates.
It exits 0 for valid geometry, 2 for invalid geometry/clearance, and 1 for malformed
requests. Unknown operations are rejected rather than silently packed.

The native API is `validate_stored_polygons(placed, target, ValidationOptions)`.
It validates a copy rounded to the declared precision, rejects malformed rings,
checks holes, unique IDs, gap, target containment and independent border clearance.
Float32 certification materializes binary32 values before checking geometry.
Placed islands may specify a zero-based `tile`; containment and border clearance
then apply to that target alone, including when targets overlap. An omitted tile
retains the legacy any-target policy. Unknown target indices fail validation.
The native field is `PlacedIsland::tile`; native layouts and snap proposals always
retain their placement's assigned tile.

Validation-only callers can supply per-island `group` and `shelved`, and
`options.overlay_groups=true`. Different overlay groups are independent; ordinary
groups still require separation. Shelved geometry is checked for overlap but is
outside the target by design. These explicit exemptions must match the layout's
declared policy. Native equivalents are `PlacedIsland::{group,shelved}` and
`ValidationOptions::overlaid_groups`.

Validation also accepts top-level `obstacles` with `outer`, `holes` and optional
nonnegative `extra_margin` (native `ValidationOptions::obstacles`). Island-to-obstacle
clearance is `margin + extra_margin`. Obstacles are checked and rounded at the
declared precision, but need not lie inside a target or be disjoint from each other.
Messages identify obstacle indices; offending IDs refer only to movable islands.
Tolerance is `8 * epsilon(precision) * max(1, abs(target coordinates))`; it is
reported explicitly with minimum measured gap/border and offending island IDs.
A 1% gap shortage is not forgiven as a percentage of the requested margin.
Distances use the established aspect-corrected packing space for rectangular
textures. Polygon checks cannot detect interior triangle folds absent from the
request; the integration must also check original triangle coverage.

An effective gap of exactly zero permits boundary-only contact, including shared
edges, shared vertices and an island exactly filling another island's hole.
Interior overlap still fails. Ambiguous zero-distance contacts use exact rational
interior tests after a bounding-box check; the supplied binary coordinates are
not snapped to an integer grid or sampled at rounded midpoints. A positive gap,
including obstacle extra margin, still forbids exact contact even below the
reported numerical tolerance. The separate legacy binding predicate retains its
"touching" behavior. The stripe rasterizer's horizontal guard and float32
packing's conservative rounding reserve can still prevent a tightly constrained
fixed-scale fit; validation of a supplied stored zero-gap layout does not promise
that every search mode will find it.

Bounded native packing/compaction returns and CLI snap/result checks now use the
full requested gap, independent border clearance and target-scaled numerical tolerance;
the native layout validator also rejects duplicate, unknown or missing original IDs.
The legacy `validate_layout` and `validate_polygons` APIs accept an optional final
`border` argument (default zero); negative/default `tolerance` selects
`validation_tolerance(target, targets)`. Explicit nonnegative tolerances remain
available to independent callers. Packing responses report `tolerance`,
`border_violations` and `min_border` alongside the existing gap/containment fields.
Snap fallback uses the caller's original border, before any packing reserve.

Native callers with a `Layout` can use
`validate_stored_layout(original_islands, layout, target, options)` to apply its
transforms to original polygon footprints and check the rounded result without
changing any input. It checks original/placed/unplaced ID accounting, assigned
tiles, holes, extra parts, obstacles, mirroring and rectangular texture aspect.
Overlaid groups remain independent, and explicitly shelved islands retain their
unplaced membership and overlap-only policy. A valid geometry report for a
partial layout does not mean all original islands were packed. An invalid layout
flag, malformed original rings, storage collapse or cancellation cannot produce
a valid certificate. Supply the original footprints, not replacement hulls;
interior triangle folds absent from those footprints still require source-mesh QA.

Bounded native callers can opt into `PackOptions::output_precision="float32"`
for `pack_fixed_scale`, `pack_max_scale`, `pack_max_margin` and `compact_layout`.
The default remains `float64`. Float32 packing reserves conservative rounding
clearance before search and checks rounded original footprints before accepting
candidates and returning a valid result. Returned transforms remain doubles;
apply them to the original footprints and store the result at the declared
precision. Fixed-scale requests retain their requested scale: a reserve that
prevents fitting produces an explicit failure, never a final fitting shrink.
A geometric fit that fails its float32 certificate does not end the CPU trial
portfolio: the remaining requested trials can try other placements at the same
scale. Failed certificates are excluded from partial-result selection as well;
trial limits and cancellation still apply. If no trial survives, the native
search failure retains the storage diagnostic. If the search returns no placements,
the CLI preserves that cause in `failure` while still reporting missing output
in `validation`, in both ordinary and streaming responses.
Maximum-margin results report the achieved user margin without the internal reserve.
Partial results retain their existing invalid flag and unplaced/shelved accounting.

Float32 improvement callbacks receive only fully accounted, certified results;
group/cluster searches may emit fewer intermediate previews. Candidate validation
shares the search cancellation token. The final check of a retained incumbent uses
an independent token and must finish before that result can be returned as valid.
Cancellation may therefore suppress a preview while the final response retains a
better certified layout. Clients must apply the final response; it need not be
identical to their last observed preview.

`pack_open` and `pack_strip` reject float32 because they do not promise a bounded
target. The opaque `PackOptions::storage_policy` member is internal search state;
callers must leave it empty.

The CLI accepts the same `options.output_precision` and reports it in final and
improvement records. Final validation applies the emitted transforms or per-vertex
coordinates to original input footprints at that precision, including all bound
member IDs, holes and extra parts. `options.validate=false` suppresses successful
diagnostic reports; it cannot bypass certification. Failed certificates always
include diagnostics and cannot produce `valid=true` or `complete=true`.

Optional snapping is checked at storage precision. If the emitted original
geometry fails its final check, the CLI falls back to the unchanged unsnapped
layout and reports the reason. It never repairs clearance with a final shrink.
Only certified original-footprint improvements are streamed. An explicit
envelope-only request with approved hull recovery retains its hull-footprint
contract; a supplied malformed original ring cannot be hidden by that hull.
Rigidly bound members remain one compound shape with intentional internal overlap.

These certificates cover the supplied polygon footprints. `snap_mode="all"`
also asks the host to snap interior vertices the packer has not received, so
original-triangle QA and actual host readback remain the consumer’s responsibility.
Validate the stored result, including zero-gap contacts, at the requested precision.
