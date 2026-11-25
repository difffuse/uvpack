// Single-pass mode, and the extent estimate it shares with the bracket stage.
//
// One greedy pass into an OPEN frame at (a fraction of) the area bound, then a uniform
// rescale of the whole layout down onto the target. The extent the open pass reached
// says how far the scale was off, which is the estimate the bracket starts from; under
// `single_pass` the rescaled layout is the answer itself, and the margin is inflated by
// `single_pass_margin_reserve` before the pass so the rescale cannot shrink a gap below
// what was asked for. Like every other candidate, it only counts once the exact
// validator has accepted it.
#include "search_internal.hpp"
#include "uvpack/validator.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace uvpack {

OpenProbe open_probe(const std::vector<Island>& islands, const PackTarget& target,
                     const PackOptions& options, std::size_t k, double upper,
                     SearchStats& st, const std::vector<Obstacle>& obstacles) {
    OpenProbe probe;
    double& estimate = probe.estimate;
    PackOptions open = options;
    open.trials = std::min<std::size_t>(k, 4); // A few diversified orderings; keep the tightest extent.
    open.warm_order.clear();
    if (options.single_pass) open.margin = options.margin * options.single_pass_margin_reserve;
    double s0 = upper * (options.single_pass ? options.single_pass_start : options.initial_fraction);
    const double border = options.border_margin < 0 ? options.margin : options.border_margin;
    const double inset = std::max(0.0, border - 0.5 * options.margin);
    const double frame_w = (target.max_x - target.min_x) * options.target_aspect - 2 * inset, frame_hh = target.max_y - target.min_y - 2 * inset;
    SearchStats open_stats;
    Layout layout;
    double extent = 0;
    // One open pass; in single-pass mode, one retry at the measured extent when the
    // first pass spilled further than the margin reserve covers.
    for (int attempt = 0; attempt < (options.single_pass ? 2 : 1); ++attempt) {
        layout = pack_open(islands, target, s0, open, 1.6, &open_stats);
        ++st.scales_tried;
        extent = 0;
        if (layout.valid) {
            extent = std::max(layout.occupied_width / frame_w, layout.occupied_height / frame_hh);
            if (extent > 0 && std::isfinite(extent)) estimate = s0 / extent;
        }
        if (options.progress) std::fprintf(stderr, "[uvpack] open pass %d: s0 %.4f extent %.4f -> estimate %.4f\n", attempt, s0, extent, estimate);
        if (!options.single_pass || !layout.valid || extent <= options.single_pass_margin_reserve || estimate <= 0) break;
        s0 = estimate * 0.995;
    }
    st.evaluations += open_stats.evaluations;
    st.prepare_seconds = std::max(st.prepare_seconds, open_stats.prepare_seconds);
    st.place_seconds = std::max(st.place_seconds, open_stats.place_seconds);
    // Single pass: the rescaled open layout is the answer when the reserved margin
    // covers the shrink (gaps scale by 1/extent), validated like any other result.
    if (options.single_pass && layout.valid && extent > 0 && extent <= options.single_pass_margin_reserve) {
        Layout scaled = layout;
        const double f = 1.0 / std::max(extent, 1.0); // Never enlarge: a layout inside the frame stays as packed.
        for (auto& p : scaled.placements) {
            p.island_scale *= f;
            // Offsets carry the border inset already; rescale about the frame origin.
            p.offset.x = (p.offset.x - inset) * f + inset; p.offset.y = (p.offset.y - inset) * f + inset;
            p.source_origin.x *= f; p.source_origin.y *= f;
        }
        scaled.scale = s0 * f;
        scaled.occupied_width = layout.occupied_width * f; scaled.occupied_height = layout.occupied_height * f;
        const auto report = validate_layout(islands, scaled, target, options.margin, -1, obstacles, options.target_aspect, options.targets,false,border);
        if (report.ok && storage_candidate_valid(scaled,islands,target,options,obstacles)) {
            st.history.emplace_back(scaled.scale, true);
            probe.layout = std::move(scaled);
            probe.accepted = true;
        } else if (options.progress) {
            std::fprintf(stderr, "[uvpack] single pass rejected by validation (%s); falling back\n", report.messages.empty() ? "" : report.messages.front().c_str());
        }
    }

    return probe;
}

} // namespace uvpack
