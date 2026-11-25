// Scale policy: several uniform scales in one run, one per cluster of groups.
#include "search_internal.hpp"
#include "uvpack/geometry.hpp"
#include <algorithm>
#include <functional>
#include <map>
#include <numeric>
#include <string>

namespace uvpack {
namespace {

// Group id -> dense cluster index. Clusters are numbered by descending island area, so
// cluster 0 is the one that packs first and the numbering is deterministic.
std::vector<std::size_t> cluster_of_island(const std::vector<Island>& islands, const PackOptions& options,
                                           std::size_t tile_count) {
    std::vector<std::size_t> groups;
    for (const auto& island : islands) groups.push_back(island.group);
    std::sort(groups.begin(), groups.end());
    groups.erase(std::unique(groups.begin(), groups.end()), groups.end());
    const std::size_t n = groups.size();
    auto slot_of = [&](std::size_t group) {
        return static_cast<std::size_t>(std::lower_bound(groups.begin(), groups.end(), group) - groups.begin());
    };
    // Union-find over group slots; the policy only decides which pairs get united.
    std::vector<std::size_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    std::function<std::size_t(std::size_t)> find = [&](std::size_t i) { return parent[i] == i ? i : parent[i] = find(parent[i]); };
    auto unite = [&](std::size_t a, std::size_t b) { parent[find(a)] = find(b); };
    auto tiles_of = [&](std::size_t group) {
        auto it = options.group_tiles.find(group);
        std::vector<std::size_t> tiles;
        if (it == options.group_tiles.end()) { // Unrestricted: every tile.
            tiles.resize(tile_count);
            std::iota(tiles.begin(), tiles.end(), std::size_t{0});
        } else {
            for (const auto t : it->second) if (t < tile_count) tiles.push_back(t);
        }
        return tiles;
    };
    switch (options.scale_policy) {
    case ScalePolicy::PerGroup:
        break; // Nothing united: one cluster per group.
    case ScalePolicy::Custom:
        for (std::size_t a = 0; a < n; ++a) {
            auto ia = options.scale_cluster.find(groups[a]);
            if (ia == options.scale_cluster.end()) continue; // Unlisted group: its own cluster.
            for (std::size_t b = a + 1; b < n; ++b) {
                auto ib = options.scale_cluster.find(groups[b]);
                if (ib != options.scale_cluster.end() && ib->second == ia->second) unite(a, b);
            }
        }
        break;
    case ScalePolicy::ByTiles:
        for (std::size_t a = 0; a < n; ++a) {
            const auto ta = tiles_of(groups[a]);
            for (std::size_t b = a + 1; b < n; ++b) {
                const auto tb = tiles_of(groups[b]);
                for (const auto t : ta)
                    if (std::find(tb.begin(), tb.end(), t) != tb.end()) { unite(a, b); break; }
            }
        }
        break;
    default:
        for (std::size_t a = 1; a < n; ++a) unite(a, 0);
        break;
    }
    // Rank the roots by summed island area (the same measure the area bound uses).
    std::map<std::size_t, double> area;
    for (const auto& island : islands) {
        const double s = island.pre_scale;
        area[find(slot_of(island.group))] += measure_island(island).area * s * s * options.target_aspect;
    }
    std::vector<std::pair<std::size_t, double>> ranked(area.begin(), area.end());
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    std::map<std::size_t, std::size_t> index;
    for (std::size_t i = 0; i < ranked.size(); ++i) index[ranked[i].first] = i;
    std::vector<std::size_t> out;
    for (const auto& island : islands) out.push_back(index[find(slot_of(island.group))]);
    return out;
}

// A placed island as a static obstacle for the clusters packed after it: the ORIGINAL
// polygon (and its extra parts) mapped through the placement into target space.
void placed_obstacles(const std::vector<Island>& islands, const Layout& layout, const PackTarget& target,
                      double target_aspect, std::vector<Obstacle>& out) {
    for (const auto& p : layout.placements) {
        auto it = std::find_if(islands.begin(), islands.end(), [&](const Island& i) { return i.id == p.id; });
        if (it == islands.end()) continue;
        const auto t = p.transform(layout.scale, target, target_aspect);
        auto map_ring = [&](const Ring& ring) {
            Ring out_ring;
            for (const auto v : ring) out_ring.push_back({t[0] * v.x + t[1] * v.y + t[2], t[3] * v.x + t[4] * v.y + t[5]});
            return out_ring;
        };
        auto map_polygon = [&](const Polygon& poly) {
            Polygon mapped;
            mapped.outer = map_ring(poly.outer);
            for (const auto& hole : poly.holes) mapped.holes.push_back(map_ring(hole));
            return mapped;
        };
        out.push_back({map_polygon(it->polygon), 0.0});
        for (const auto& part : it->extra_parts) out.push_back({map_polygon(part), 0.0});
    }
}

} // namespace

Layout pack_clusters(const std::vector<Island>& islands, const PackTarget& target,
                     const PackOptions& options, SearchStats& st, const std::vector<Obstacle>& obstacles) {
    const auto targets = effective_targets(target, options);
    const auto cluster = cluster_of_island(islands, options, targets.size());
    std::size_t count = 0;
    for (const auto c : cluster) count = std::max(count, c + 1);
    Layout combined;
    combined.valid = true;
    PackOptions inner = options;
    inner.scale_policy = ScalePolicy::Shared; // One scale inside a cluster.
    inner.cluster_tile_bound = true;                     // Bound by the tiles this subset can reach.
    std::vector<Obstacle> blocked = obstacles;
    for (std::size_t c = 0; c < count; ++c) {
        std::vector<Island> subset;
        for (std::size_t i = 0; i < islands.size(); ++i) if (cluster[i] == c) subset.push_back(islands[i]);
        if (subset.empty()) continue;
        SearchStats cs;
        // A streamed solution is the whole picture: earlier clusters as placed, plus
        // the incumbent of the cluster being searched right now.
        if (options.on_improved)
            inner.on_improved = [&, c](const Layout& partial) {
                Layout shown = combined;
                for (auto placement : partial.placements) { placement.cluster = c; shown.placements.push_back(placement); }
                shown.scale = std::max(combined.scale, partial.scale);
                shown.valid = true;
                options.on_improved(shown);
            };
        const auto layout = pack_max_scale(subset, target, inner, &cs, blocked);
        st.evaluations += cs.evaluations;
        st.scales_tried += cs.scales_tried;
        st.improve_rounds += cs.improve_rounds;
        st.prepare_seconds += cs.prepare_seconds;
        st.geometry_seconds += cs.geometry_seconds;
        st.setup_seconds += cs.setup_seconds;
        st.place_seconds += cs.place_seconds;
        st.compaction_seconds += cs.compaction_seconds;
        st.compaction_attempts += cs.compaction_attempts;
        st.compaction_accepted += cs.compaction_accepted;
        st.compaction_rotations += cs.compaction_rotations;
        st.shrunk_islands += cs.shrunk_islands;
        st.upper_bound = std::max(st.upper_bound, cs.upper_bound);
        for (const auto& h : cs.history) st.history.push_back(h);
        st.cluster_scales.push_back({c, layout.scale, subset.size(), layout.valid});
        if (!layout.valid) {
            combined.valid = false;
            if (combined.failure.empty())
                combined.failure = "cluster " + std::to_string(c) + ": " + (layout.failure.empty() ? "no valid layout" : layout.failure);
        }
        for (auto placement : layout.placements) { placement.cluster = c; combined.placements.push_back(placement); }
        for (const auto id : layout.order) combined.order.push_back(id);
        for (const auto id : layout.unplaced) combined.unplaced.push_back(id);
        combined.occupied_width = std::max(combined.occupied_width, layout.occupied_width);
        combined.occupied_height = std::max(combined.occupied_height, layout.occupied_height);
        // `scale` is one number for a run that produced several: report the largest, and
        // read the real per-cluster answer from SearchStats::cluster_scales. Nothing
        // downstream uses Layout::scale - every transform carries its own island_scale.
        combined.scale = std::max(combined.scale, layout.scale);
        placed_obstacles(subset, layout, target, options.target_aspect, blocked);
    }
    if (combined.placements.empty() && combined.failure.empty()) combined.failure = "empty input or target";
    st.best_scale = combined.scale;
    return combined;
}

} // namespace uvpack
