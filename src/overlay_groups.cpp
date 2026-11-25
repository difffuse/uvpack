// Overlaid groups: run the whole max-scale search once per group, each over the full
// target(s) with its own scale, and merge. Groups overlap by design, so validation has
// to skip cross-group pairs.
#include "search_internal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace uvpack {

Layout pack_overlay_groups(const std::vector<Island>& islands, const PackTarget& target,
                           const PackOptions& options, SearchStats& st,
                           const std::vector<Obstacle>& obstacles) {
    std::vector<std::size_t> group_ids;
    for (const auto& island : islands) group_ids.push_back(island.group);
    std::sort(group_ids.begin(), group_ids.end());
    group_ids.erase(std::unique(group_ids.begin(), group_ids.end()), group_ids.end());
    Layout merged;
    merged.overlaid_groups = true;
    if (group_ids.empty()) { merged.failure = "empty input or target"; return merged; }
    PackOptions sub = options;
    sub.overlay_groups = false;
    // The budgets stay totals: every group gets an equal share of the wall clock.
    const double share = 1.0 / double(group_ids.size());
    if (sub.time_budget_seconds > 0) sub.time_budget_seconds *= share;
    if (sub.compaction_time > 0) sub.compaction_time *= share;
    merged.valid = true;
    double smallest = std::numeric_limits<double>::infinity();
    for (const auto group : group_ids) {
        std::vector<Island> subset;
        for (const auto& island : islands) if (island.group == group) subset.push_back(island);
        SearchStats group_stats;
        if (options.on_improved)
            sub.on_improved = [&](const Layout& partial) {
                Layout shown = merged;
                shown.placements.insert(shown.placements.end(), partial.placements.begin(), partial.placements.end());
                shown.scale = partial.scale;
                shown.valid = true;
                options.on_improved(shown);
            };
        const auto layout = pack_max_scale(subset, target, sub, &group_stats, obstacles);
        st.evaluations += group_stats.evaluations;
        st.scales_tried += group_stats.scales_tried;
        st.improve_rounds += group_stats.improve_rounds;
        st.prepare_seconds += group_stats.prepare_seconds;
        st.geometry_seconds += group_stats.geometry_seconds;
        st.setup_seconds += group_stats.setup_seconds;
        st.place_seconds += group_stats.place_seconds;
        st.compaction_attempts += group_stats.compaction_attempts;
        st.compaction_accepted += group_stats.compaction_accepted;
        st.compaction_rotations += group_stats.compaction_rotations;
        st.compaction_rounds += group_stats.compaction_rounds;
        st.compaction_seconds += group_stats.compaction_seconds;
        st.upper_bound = std::max(st.upper_bound, group_stats.upper_bound);
        for (const auto& entry : group_stats.history) st.history.push_back(entry);
        st.group_scales.emplace_back(group, layout.valid ? layout.scale : 0.0);
        if (!layout.valid) {
            merged.valid = false;
            if (merged.failure.empty())
                merged.failure = "group " + std::to_string(group) + ": " + layout.failure;
        }
        merged.placements.insert(merged.placements.end(), layout.placements.begin(), layout.placements.end());
        merged.order.insert(merged.order.end(), layout.order.begin(), layout.order.end());
        merged.unplaced.insert(merged.unplaced.end(), layout.unplaced.begin(), layout.unplaced.end());
        merged.occupied_width = std::max(merged.occupied_width, layout.occupied_width);
        merged.occupied_height = std::max(merged.occupied_height, layout.occupied_height);
        if (layout.valid) smallest = std::min(smallest, layout.scale);
    }
    // One number has to stand for several: report the group that had to shrink most.
    merged.scale = std::isfinite(smallest) ? smallest : 0.0;
    st.best_scale = merged.scale;
    return merged;
}

} // namespace uvpack
