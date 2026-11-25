#pragma once
// High-level packing entry points: fixed-scale placement of ordinary polygons and a
// uniform max-scale search on top of it. This is the packer's public surface; what the
// pieces underneath do is described in README.md.
#include "uvpack/orientations.hpp"
#include "uvpack/cancellation.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>

namespace uvpack {

struct PackTarget { double min_x = 0, min_y = 0, max_x = 1, max_y = 1; };

// Which direction the layout is encouraged to fill. `Free` is the plain search; the
// others bias the growth term so the layout fills one target dimension first.
enum class FillMode { Free, WidthFirst, HeightFirst, Proportional };
// The corner the finished layout is anchored to: a final reflection inside the target.
enum class Origin { BottomLeft, BottomRight, TopRight, TopLeft };
// Texel density policy: how many uniform scales a run is allowed to produce.
// Islands are partitioned by their `group` into clusters, and each cluster gets its
// own max-scale search. See `PackOptions::scale_policy`.
enum class ScalePolicy {
    Shared,    // One scale for everything (the default, and what every other mode assumes).
    PerGroup,  // Every group is its own cluster.
    Custom,    // Groups sharing a `scale_cluster` value form a cluster.
    ByTiles,   // Groups whose allowed tile sets intersect form a cluster.
};
// Snap to texels: which vertices are snapped to the texel grid, and what they are
// snapped to. BBoxMin is the bounding-box-minimum translation of a whole island; the
// other three snap larger sets of vertices (see include/uvpack/snap.hpp for the exact
// semantics of each).
enum class SnapMode { BBoxMin, BBox, Outline, All };
enum class SnapTarget { Corner, Center };
// How the tiles are filled when there are several: `Sequential` fills tile i before
// tile i + 1, `Any` lets the placement cost decide.
enum class TileFill { Sequential, Any };

struct Layout; // Defined below; PackOptions carries a callback that takes one.
namespace detail { struct PackingStoragePolicy; }

struct PackOptions {
    double margin = 0;              // Island-to-island gap in output units (expansion margin/2 per island).
    double border_margin = -1;      // Island-to-target-border gap; negative = same as margin.
    std::size_t sub_rows = 8;         // Sub-row placement resolution (1 = row-aligned).
    bool fast_search = true;        // Indexed occupancy and exact row-cost bound in the greedy search.
    bool parallel_bracket = true;   // Evaluate `threads` scales per round (k-section) while bracketing.
    bool extent_estimate = true;    // One open pack estimates the scale from its extent before bracketing.
    double estimate_tolerance = 4e-3; // Bracket tolerance after an estimate (one refinement round).
    // ONE open pass at the area bound,
    // then a uniform rescale of the whole layout to the target. The margin is inflated
    // by `single_pass_margin_reserve` before the pass so the rescale never shrinks gaps below
    // the request; if the layout spills further than that, the bounded search runs.
    bool single_pass = false;
    double single_pass_margin_reserve = 1.05;
    double single_pass_start = 0.9;  // First open pass at this fraction of the area bound (sqrt of an efficiency prior).
    double bracket_tolerance = 1e-3; // Stop the bracket when high/low < 1 + tolerance (improvement rounds refine further).
    double bracket_start = 0.5; // First round spans [low_fraction, initial_fraction] of the area bound.
    std::size_t max_evaluations = 10; // Scale evaluations in the bracket stage.
    double stripe_height = 0.002;      // Sampling row spacing in output units.
    bool allow_rotation = true;
    bool prealign = true;           // Align each island to its minimum-area rectangle before the rotation steps.
    int rotation_step = 90;
    bool allow_mirroring = false;   // Offer mirrored orientations (texture-symmetric use only).
    double target_aspect = 1.0;        // Non-square textures: x is scaled by this during packing.
    bool allow_partial = false;     // Fixed-scale modes: place what fits and report the rest.
    // Fill mode. `Auto` is the plain search and leaves every code path
    // bit-identical. The strip modes seed the occupied canvas with the fixed
    // dimension so the cost only charges growth in the free one; `Square` seeds the
    // frame's aspect excess so isotropic growth reaches both frame edges together
    // (nothing at all on a square frame, where it coincides with `Auto`).
    FillMode fill = FillMode::Free;
    // Anchor corner: mirror the finished layout inside its target box.
    // BottomLeft (the default) is the identity and changes nothing.
    Origin origin = Origin::BottomLeft;
    // Overlaid groups: run the whole max-scale search once per group,
    // each over the full target(s) with its own scale, and merge. Groups overlap by
    // design, so validation must skip cross-group pairs.
    bool overlay_groups = false;
    // Shelve what did not fit: with allow_partial, lay the islands that did not
    // fit in rows above the target and emit transforms for them (IslandPlacement::shelved).
    bool shelve_unplaced = false;
    // Stall timeout: stop the refine stage when the incumbent
    // has not improved for this many seconds. 0 disables it.
    double stall_timeout = 0;
    bool progress = false;          // Print search progress to stderr.
    double snap_tex_size = 0; // > 0: snap each island to this texel grid after packing.
    SnapMode snap_mode = SnapMode::BBoxMin;       // Which vertices move.
    SnapTarget snap_target = SnapTarget::Corner;  // Texel corner or texel centre.
    // Multiple targets (tiles). Empty = the single `target` argument. All tiles share
    // one uniform scale. `sequential` fills tile i before tile i+1; `any` lets the
    // cost decide. group_tiles restricts a group's islands to the listed tile indices.
    std::vector<PackTarget> targets;
    TileFill tile_fill = TileFill::Sequential;
    std::map<std::size_t, std::vector<std::size_t>> group_tiles;
    // Groups packed together. Islands sharing `group`
    // are ordered contiguously, pulled towards their group's current bounding box by
    // a compactness cost, and confined to the tile the group's first island landed in.
    bool group_adjacent = false;
    double group_cohesion = 0.5;  // 0 = mild tie-break, 1 = strong; clamped to [0, 1].
    // Texel density policy. Anything but Uniform splits the islands into clusters
    // by group and runs one max-scale search per cluster, so clusters end up at
    // different scales (different relative texel densities). Clusters are packed
    // SEQUENTIALLY in descending total-area order and the islands a cluster placed
    // become obstacles for the clusters after it. No order is canonical here; descending
    // area gives the largest cluster the free canvas. Each cluster is restricted to its groups'
    // tiles through `group_tiles`; clusters that share a tile therefore share its space
    // in that order. Reported per island as `IslandPlacement::cluster` and per cluster
    // in `SearchStats::cluster_scales`.
    ScalePolicy scale_policy = ScalePolicy::Shared;
    std::map<std::size_t, long long> scale_cluster; // Custom: group id -> cluster id.
    // Mixed scales: during the improvement sub_row, whenever a valid layout has SOME
    // islands nearly spanning the target and some not, the near-spanning ones lose a
    // sixth of their per-island scale (floor 0.01) for the following rounds, so one
    // oversized island stops capping the
    // uniform scale of every other one. Needs `time_budget_seconds > 0` (that is the
    // stage it lives in) and makes the result's texel density differ per island.
    bool shrink_spanning = false;
    bool dedup_orientations = true;  // Drop orientations whose rows come out exactly identical.
    std::size_t trials = 8;          // Greedy attempts per scale (diversified orders, failure repair).
    std::size_t threads = 0;         // 0 = hardware concurrency.
    std::uint32_t seed = 1;
    // GPU worker (Vulkan compute, src/gpu_vulkan.cpp). With a time budget the
    // device runs whole extra trials of its own, at its own ladder of scales, next to
    // the CPU rounds. A device layout is only ever an extra candidate: it replaces the
    // incumbent only after the exact validator accepts it and only when its scale is
    // larger, so the CPU search is bit for bit the search it would have run alone. It
    // stays out of its own accord when there is no device, when there is no time
    // budget, and for the problems the kernel does not cover (several tiles,
    // obstacles, adjacent groups, partial packing, event rows, open and strip frames).
    //
    // Off by default on measurement, not principle: on the Apple M2's integrated GPU
    // one device trial costs roughly twenty times what one CPU core's trial costs, and
    // there are eight of those cores, so no batch ever overtook the CPU rounds on
    // mixed100/300/1000 while the improver thread took a core away from them. The
    // README's "GPU worker" section has the numbers. Worth turning on for a discrete
    // device, where the arithmetic is different.
    bool gpu = false;
    std::size_t gpu_trials = 48;     // Device trials per batch; 0 disables the device.
    // pack_fixed_scale only: run a device batch inside the round and wait for it, so a
    // scale the CPU trials could not reach still has the device's attempts behind it.
    // Off by default because that wait is seconds where a CPU round is milliseconds;
    // pack_max_scale always leaves it off and drives the device from its own search
    // instead, where nothing has to wait for it.
    bool gpu_in_round = false;
    // Max-scale search controls.
    std::size_t bracket_rounds = 12;
    std::size_t max_halvings = 16;
    double initial_fraction = 0.98;  // First trial scale as a fraction of the area upper bound.
    // Budgeted improvement after bisection: adaptive upward scale steps with fresh
    // trial rounds. 0 disables it. The best valid layout is always retained.
    double time_budget_seconds = 0;
    double initial_step = 0.02;      // Relative scale step for the refine stage.
    double min_step = 0.0005;
    // Overlap-tolerant compaction after the budgeted rounds (src/compaction.cpp).
    // Grows the incumbent layout in place and repairs the overlaps with a guided
    // local search that may rotate islands continuously. 0 disables it.
    double compaction_time = 0;      // Extra seconds spent compacting.
    bool compaction_rotation = true; // Allow continuous per-island rotation.
    bool compaction_polish = false;  // Zero-growth slack pass before growing (measured: not worth its share of the budget when growing; always on when compaction_delta is 0).
    double compaction_delta = 0.004; // First relative scale step; 0 = polish only, no growth.
    // Portfolio parallelism for the compact stage. Each round runs one
    // independent attempt per thread from the same incumbent, with its own RNG stream
    // and its own target delta taken from a ladder around the current one; the largest
    // validated scale of the round becomes the next incumbent. 1 selects the
    // single-threaded loop, 0 (the default) uses `threads`.
    std::size_t compaction_threads = 0;
    // Reproducible runs: with a nonzero sweep budget an attempt stops after that many
    // guided-local-search sweeps instead of when its time slice runs out, and with a
    // nonzero round cap the stage stops after that many growth rounds. Both zero (the
    // default) means "spend the wall-clock budget", which is not reproducible.
    std::size_t compaction_attempt_sweeps = 0;
    std::size_t compaction_max_rounds = 0;
    // Internal: island ids in the order that produced the incumbent; improvement
    // rounds warm-start from it. Set by pack_max_scale, not meant for callers.
    std::vector<std::uint64_t> warm_order;
    // Internal (texel density clusters): count only the tiles this island set can
    // actually reach towards the area upper bound. Set by the cluster driver.
    bool cluster_tile_bound = false;
    // Realtime solutions: every time the search commits a
    // better layout it is handed to this callback immediately, so a UI can draw the
    // solution while the search continues. Called from the thread that called
    // pack_max_scale (the improvement loop and the compaction accept points), never
    // from a worker thread and never concurrently, so the callee needs no locking --
    // but it runs inside the search, so it must be cheap. Empty = no streaming.
    std::function<void(const Layout&)> on_improved;
    CancellationToken cancel; // Request-owned; copies of these options share it.
    // Bounded packing/compaction: reserve rounding clearance and certify transformed
    // source polygons at this storage precision. Coordinates/transforms remain doubles.
    std::string output_precision = "float64"; // float64 | float32
    // Internal search state. Created once by a bounded entry point, never by callers.
    std::shared_ptr<const detail::PackingStoragePolicy> storage_policy;
};

struct IslandPlacement {
    std::uint64_t id;
    double angle;          // Rotation applied to the scaled original polygon, radians.
    Vec2 source_origin;    // Bounds minimum of the rotated, margin-expanded polygon.
    Vec2 offset;           // Packed position of that origin inside the target frame.
    bool mirrored = false; // x negated before scaling and rotation.
    double island_scale = 1; // Uniform scale actually applied to this island (1 for locked ones).
    std::size_t tile = 0;    // Target index the island was placed in.
    // shelve_unplaced: this island did not fit and was laid out above the
    // target instead. It is outside the target by design and skipped by containment
    // checks and by the start-corner mirror.
    bool shelved = false;
    std::size_t cluster = 0; // Texel-density cluster that produced this scale (0 unless clustered).
    // Original vertex p maps to: rotate(mirror(p) * island_scale, angle) - source_origin + offset + target.min,
    // then x is divided by target_aspect.
    std::array<double, 9> transform(double scale, const PackTarget& target, double target_aspect = 1.0) const;
};

struct Layout {
    bool valid = false;
    double scale = 1;
    double occupied_width = 0, occupied_height = 0; // Extent of the placed islands inside the frame.
    std::vector<IslandPlacement> placements;
    std::vector<std::uint64_t> order; // Island ids in the placement order that produced this layout.
    std::vector<std::uint64_t> unplaced; // Islands that did not fit (allow_partial only).
    // overlay_groups: each group was packed over the full target on its own,
    // so placements of different groups overlap by design and `scale` is the smallest of
    // the per-group scales. Validators must only compare islands of the same group.
    bool overlaid_groups = false;
    std::string failure; // Non-empty when the trial threw or could not place everything.
    bool cancelled = false; // A valid retained incumbent may accompany cancellation.
    Vec2 map_vertex(const IslandPlacement& p, Vec2 original, const PackTarget& target) const;
};

// One entry per texel-density cluster, in the order the clusters were packed.
struct ClusterScale {
    std::size_t cluster = 0;   // Dense index; 0 is the cluster with the most island area.
    double scale = 0;          // The uniform scale this cluster reached.
    std::size_t islands = 0;
    bool valid = false;
};

struct SearchStats {
    std::size_t evaluations = 0;      // Placer pack() calls.
    std::size_t scales_tried = 0;
    std::size_t improve_rounds = 0;   // Scale steps attempted in the refine stage.
    double best_scale = 0;
    double upper_bound = 0;
    double seconds = 0;
    double prepare_seconds = 0;       // Geometry preparation (margin, hull, orientations, rows) plus placer setup.
    double geometry_seconds = 0;      // Of which: margin, hull, orientation rasters.
    double setup_seconds = 0;         // Of which: placer construction (indexes) and prepare().
    double place_seconds = 0;         // Greedy placement trials (wall time, parallel).
    std::vector<std::pair<double, bool>> history; // (scale, valid) per scale tried.
    // overlay_groups: the scale each group reached, in group order.
    std::vector<std::pair<std::size_t, double>> group_scales;
    // Texel density policy and mixed scales.
    std::vector<ClusterScale> cluster_scales; // Empty under the Uniform policy.
    std::size_t shrunk_islands = 0;       // Islands whose scale the mixed-scale feedback cut.
    // Compact stage (src/compaction.cpp).
    std::size_t compaction_attempts = 0;
    std::size_t compaction_accepted = 0;
    std::size_t compaction_rotations = 0;
    std::size_t compaction_threads = 0; // Parallel attempts per compaction round.
    std::size_t compaction_rounds = 0;  // Portfolio rounds (1 attempt per thread each).
    double compaction_start_scale = 0;  // Scale of the incumbent the stage was handed.
    double compaction_seconds = 0;
    std::string compaction_skipped;
    // GPU worker (src/gpu_vulkan.cpp).
    std::size_t gpu_trials = 0;        // Whole trials dispatched to the device.
    std::size_t gpu_evaluations = 0;   // Of those, trials the device placed completely.
    std::size_t gpu_accepted = 0;      // Of those, layouts the exact validator accepted.
    std::size_t gpu_used = 0;          // Of those, layouts that actually became the result.
    std::size_t gpu_dispatches = 0;
    double gpu_upload_seconds = 0, gpu_compute_seconds = 0, gpu_wait_seconds = 0;
    std::string gpu_device;            // Empty when the device was never reached.
    std::string gpu_skipped;           // Why the device was not used, when it was not.
};

// Static obstacles: polygons already in target space that packed islands must avoid.
struct Obstacle { Polygon polygon; double extra_margin = 0; };

// Place all islands at one uniform scale. Runs up to `options.trials` greedy
// attempts in parallel rounds with different orders; a failed attempt moves the
// island that did not fit to the front of its order for the next round.
// Deterministic for a given seed and thread count.
Layout pack_fixed_scale(const std::vector<Island>& islands, const PackTarget& target,
                        double scale, const PackOptions& options, SearchStats* stats = nullptr,
                        const std::vector<Obstacle>& obstacles = {});
// Strip variant: same width, unbounded height (height_factor x target height). Always
// tries to place everything; the caller reads occupied_height to judge the scale.
Layout pack_strip(const std::vector<Island>& islands, const PackTarget& target, double scale,
                  const PackOptions& options, double height_factor, SearchStats* stats = nullptr);
// Open variant: both frame dimensions enlarged by `factor`; the growth term keeps the
// layout compact, and occupied_width/height report its extent.
Layout pack_open(const std::vector<Island>& islands, const PackTarget& target, double scale,
                 const PackOptions& options, double factor, SearchStats* stats = nullptr);

// Uniform max-scale search: area upper bound, downward probe until feasible,
// bisection, then (with a time budget) adaptive upward steps. A failed round is
// treated as "probably infeasible at this scale", never as a proof.
Layout pack_max_scale(const std::vector<Island>& islands, const PackTarget& target,
                      const PackOptions& options, SearchStats* stats = nullptr,
                      const std::vector<Obstacle>& obstacles = {});
// Pixel-perfect alignment: translate each placed island so its bounding-box minimum
// (in output UV space) lands on the texel grid of `tex_size`. Returns the number of
// islands moved. Callers must re-validate; gaps can shrink by up to one texel.
std::size_t snap_to_texels(Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                           double tex_size, double target_aspect);
// Anchor corner: mirror the whole finished arrangement inside its target box
// (BottomRight mirrors x, TopLeft mirrors y, TopRight both; BottomLeft is a no-op).
// Islands are rigid, so this is one mirror of the layout: it composes with each
// island's own `mirrored` flag and negates/complements its angle accordingly. Applied
// by the public pack entry points; exported so callers that post-process a layout
// themselves can apply it last. Arranged islands (outside the target) are left alone.
void apply_origin(Layout& layout, const PackOptions& options, const PackTarget& target);
// Shelve what did not fit: lay the islands of `layout.unplaced` in rows above the
// target, sorted by descending bounding-box area, starting at (target.min_x,
// target.max_y + 0.05), left to right with 0.05 gaps, wrapping past target.min_x + 5,
// each row as tall as its tallest island. They are appended to `layout.placements` with
// `shelved = true` and stay listed in `layout.unplaced`. Returns how many were laid out.
std::size_t shelve_unplaced(Layout& layout, const std::vector<Island>& islands,
                             const PackTarget& target, const PackOptions& options);
// Deprecated process-wide adapter. Native calls only observe it when explicitly
// given options.cancel = legacy_cancellation_token(). New callers own their token.
CancellationToken legacy_cancellation_token();
void request_cancel();
bool cancel_requested();

// Fixed-scale max-margin search: keep the scale, find the largest island gap that
// still packs. Bisection on the margin with the same greedy rounds; the best
// valid layout (largest margin) is retained and its margin returned via `margin`.
Layout pack_max_margin(const std::vector<Island>& islands, const PackTarget& target, double scale,
                       const PackOptions& options, double& margin, SearchStats* stats = nullptr,
                       const std::vector<Obstacle>& obstacles = {});

} // namespace uvpack
