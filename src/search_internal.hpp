#pragma once
// Internal to the search: what scale_search.cpp shares with the pieces it drives.
// Not an installed header and not part of the public API -- that is packer.hpp.
#include "uvpack/packer.hpp"
#include "cancellation_internal.hpp"
#include "uvpack/tile_placer.hpp"
#include <chrono>
#include <vector>

namespace uvpack {

using Clock = std::chrono::steady_clock;

// The scale this island is actually packed at: its own pre-scale, times the run's
// uniform scale unless the island is scale-locked.
double island_scale_for(const Island& island, double scale);

// The tiles a run packs into: the explicit list when there is one, else the single
// target it was handed.
std::vector<PackTarget> effective_targets(const PackTarget& target, const PackOptions& options);

// Worker threads for this run: the option, or hardware concurrency when it is 0.
std::size_t thread_count(const PackOptions& options);

// Post-processing shared by the public entry points; the internal search probes skip it.
void finish_layout(Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                   const PackOptions& options);

// One greedy pack at a fixed scale. `height_factor` and `width_factor` enlarge the
// frame for the strip and open variants; 1.0 is the bounded pack.
Layout pack_impl(const std::vector<Island>& islands, const PackTarget& target,
                 double scale, const PackOptions& options, double height_factor, SearchStats* stats,
                 const std::vector<Obstacle>& obstacles, double width_factor = 1.0);

// Rasterize a static obstacle into one tile's canvas (src/obstacles.cpp).
void add_obstacle(TilePlacer& placer, std::size_t tile, const Obstacle& obstacle, const PackTarget& target,
                  double inset, double frame_w, double frame_h, const PackOptions& options);

// Scale policy: one max-scale search per cluster of groups (src/clusters.cpp).
Layout pack_clusters(const std::vector<Island>& islands, const PackTarget& target,
                     const PackOptions& options, SearchStats& st, const std::vector<Obstacle>& obstacles);

// Overlaid groups: one whole search per group over the full target (src/overlay_groups.cpp).
Layout pack_overlay_groups(const std::vector<Island>& islands, const PackTarget& target,
                           const PackOptions& options, SearchStats& st,
                           const std::vector<Obstacle>& obstacles);

// One open pass at the area bound (src/single_pass.cpp). It measures how far the scale
// is off, and under `single_pass` it can also be the answer on its own.
struct OpenProbe {
    double estimate = 0;   // Scale estimated from the open pass's extent; 0 = no estimate.
    bool accepted = false; // Single-pass only: `layout` validated and is the answer.
    Layout layout;
};
OpenProbe open_probe(const std::vector<Island>& islands, const PackTarget& target,
                     const PackOptions& options, std::size_t k, double upper,
                     SearchStats& st, const std::vector<Obstacle>& obstacles);

} // namespace uvpack
