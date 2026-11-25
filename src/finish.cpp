// Finishing a layout: the origin reflection inside the target, and shelving the
// islands that did not fit in rows above it. Both run after the search, on the layout
// the public entry points are about to return.
#include "search_internal.hpp"
#include "uvpack/validator.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace uvpack {

PackOptions storage_search_options(const std::vector<Island>& islands,const PackTarget& target,
                                  const PackOptions& options,const std::vector<Obstacle>& obstacles,
                                  bool finish_compaction_candidates) {
    if(options.storage_policy || options.output_precision=="float64")return options;
    if(options.output_precision!="float32")throw std::invalid_argument("unknown output_precision");
    if(!std::isfinite(options.margin) || options.margin<0 || !std::isfinite(options.border_margin) ||
       !std::isfinite(options.target_aspect) || options.target_aspect<=0)
        throw std::invalid_argument("invalid storage clearance policy");
    // One full float32 epsilon per coordinate bounds rounding in either direction.
    // Include obstacle vertices: their rounded edges participate in the certificate.
    double magnitude=validation_tolerance(target,options.targets,"float32")/
        (8*std::numeric_limits<float>::epsilon());
    const auto include=[&](const Ring& ring) {
        for(const auto p:ring) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("nonfinite obstacle coordinate");
            magnitude=std::max({magnitude,std::abs(p.x),std::abs(p.y)});
        }
    };
    for(const auto& obstacle:obstacles) {
        include(obstacle.polygon.outer);
        for(const auto& hole:obstacle.polygon.holes)include(hole);
    }
    if(magnitude>std::numeric_limits<float>::max())throw std::invalid_argument("storage coordinates exceed float32 range");
    const double dy=std::numeric_limits<float>::epsilon()*magnitude,dx=dy*options.target_aspect;
    PackOptions prepared=options;
    prepared.margin=std::nextafter(options.margin+2*std::hypot(dx,dy),std::numeric_limits<double>::infinity());
    if(options.border_margin>=0)
        prepared.border_margin=std::nextafter(options.border_margin+std::max(dx,dy),std::numeric_limits<double>::infinity());
    if(!std::isfinite(prepared.margin)||!std::isfinite(prepared.border_margin))
        throw std::invalid_argument("float32 clearance reserve overflow");
    auto policy=std::make_shared<detail::PackingStoragePolicy>();
    policy->gap_reserve=prepared.margin-options.margin;
    policy->border_reserve=prepared.border_margin-options.border_margin;
    policy->finish_compaction_candidates=finish_compaction_candidates;
    prepared.storage_policy=std::move(policy);
    // Nested bounded calls use the already-reserved policy; internal open probes
    // still work in doubles, then certify the bounded candidate after rescaling.
    prepared.output_precision="float64";
    if(options.on_improved) {
        const auto callback=options.on_improved;
        prepared.on_improved=[callback,&islands,target,options,&obstacles](const Layout& candidate) {
            const auto checked=finish_result(candidate,islands,target,options,obstacles,false);
            if(checked.valid)callback(checked);
        };
    }
    return prepared;
}

Layout finish_result(Layout layout,const std::vector<Island>& islands,const PackTarget& target,
                           const PackOptions& options,const std::vector<Obstacle>& obstacles,
                           bool final_certificate) {
    layout.cancelled=options.cancel.requested();
    if(layout.valid) {
        try {
            ValidationReport report;
            if(options.storage_policy || options.output_precision=="float32") {
                ValidationOptions policy;policy.output_precision="float32";
                policy.gap=options.margin-(options.storage_policy?options.storage_policy->gap_reserve:0);
                policy.border=options.border_margin<0?policy.gap:
                    options.border_margin-(options.storage_policy?options.storage_policy->border_reserve:0);
                policy.target_aspect=options.target_aspect;policy.targets=options.targets;policy.obstacles=obstacles;
                // A cancelled search may retain a usable incumbent. Its final
                // certificate has an independent token and must finish completely.
                if(!final_certificate)policy.cancel=options.cancel;
                report=validate_stored_layout(islands,layout,target,policy);
            } else {
                if(options.output_precision!="float64")throw std::invalid_argument("unknown output_precision");
                report=validate_layout(islands,layout,target,options.margin,-1,
                    obstacles,options.target_aspect,options.targets,layout.overlaid_groups,
                    options.border_margin<0?options.margin:options.border_margin);
            }
            layout.cancelled=options.cancel.requested();
            if(!report.ok) {
                layout.valid=false;
                layout.failure=layout.cancelled?"cancelled incumbent failed validation":"layout failed final clearance validation";
                if(!report.messages.empty())layout.failure+=": "+report.messages.front();
            }
        } catch(const std::exception& error) {
            layout.valid=false;layout.failure=std::string("layout validation failed: ")+error.what();
        }
    } else if(layout.cancelled)layout.failure="packing cancelled";
    return layout;
}

bool storage_candidate_valid(Layout& layout,const std::vector<Island>& islands,const PackTarget& target,
                             const PackOptions& options,const std::vector<Obstacle>& obstacles,bool finished) {
    if(!layout.valid || (!options.storage_policy && options.output_precision=="float64"))return layout.valid;
    auto candidate=layout;
    if(!finished)finish_layout(candidate,islands,target,options);
    candidate=finish_result(std::move(candidate),islands,target,options,obstacles,false);
    layout.cancelled=candidate.cancelled;
    if(!candidate.valid){layout.valid=false;layout.failure=candidate.failure;}
    return layout.valid;
}

void apply_origin(Layout& layout, const PackOptions& options, const PackTarget& target) {
    const bool mirror_x = options.origin == Origin::BottomRight ||
                          options.origin == Origin::TopRight;
    const bool mirror_y = options.origin == Origin::TopLeft ||
                          options.origin == Origin::TopRight;
    if (!mirror_x && !mirror_y) return; // BottomLeft: identity.
    constexpr double pi = 3.14159265358979323846;
    const auto targets = effective_targets(target, options);
    const double ratio = options.target_aspect;
    for (auto& p : layout.placements) {
        if (p.shelved) continue; // Laid out above the target on purpose.
        const auto& tile = targets[std::min(p.tile, targets.size() - 1)];
        // transform() maps a packing-space vertex q to A q + t, with
        // A = R(angle) * scale * diag(mirrored ? -1 : 1, 1) and t below.
        double tx = -p.source_origin.x + p.offset.x + target.min_x * ratio;
        double ty = -p.source_origin.y + p.offset.y + target.min_y;
        // Reflecting the arrangement is F o (A q + t) = (F A) q + (F t + c).
        if (mirror_x) tx = (tile.min_x + tile.max_x) * ratio - tx;
        if (mirror_y) ty = (tile.min_y + tile.max_y) - ty;
        // F A back into the R * S * M form: diag(-1,1) A negates the angle and flips the
        // mirror, diag(1,-1) A turns it into pi - angle and flips the mirror, and doing
        // both is a plain half turn that leaves the island's own mirror alone.
        if (mirror_x && mirror_y) p.angle = p.angle + pi;
        else if (mirror_x) { p.angle = -p.angle; p.mirrored = !p.mirrored; }
        else { p.angle = pi - p.angle; p.mirrored = !p.mirrored; }
        p.angle = std::fmod(p.angle, 2 * pi);
        if (p.angle < 0) p.angle += 2 * pi;
        // The reflected orientation's own bounds minimum is not the reflected one, so the
        // whole translation moves into `offset` (the convention compaction also writes).
        p.source_origin = {0, 0};
        p.offset = {tx - target.min_x * ratio, ty - target.min_y};
    }
}

std::size_t shelve_unplaced(Layout& layout, const std::vector<Island>& islands,
                             const PackTarget& target, const PackOptions& options) {
    if (layout.unplaced.empty()) return 0;
    constexpr double gap = 0.05;     // Fixed spacing between shelved islands, in output units.
    constexpr double wrap_width = 5; // Start a new shelf past target.min_x + 5.
    struct Item { std::uint64_t id; double scale, min_x, min_y, width, height; };
    std::vector<Item> items;
    for (const auto id : layout.unplaced) {
        auto it = std::find_if(islands.begin(), islands.end(), [&](const Island& i) { return i.id == id; });
        if (it == islands.end()) continue;
        double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
        auto visit = [&](const Polygon& poly) {
            for (const auto v : poly.outer) {
                min_x = std::min(min_x, v.x); max_x = std::max(max_x, v.x);
                min_y = std::min(min_y, v.y); max_y = std::max(max_y, v.y);
            }
        };
        visit(it->polygon);
        for (const auto& part : it->extra_parts) visit(part);
        if (!(max_x >= min_x && max_y >= min_y)) continue;
        const double s = island_scale_for(*it, layout.scale);
        items.push_back({id, s, min_x, min_y, (max_x - min_x) * s, (max_y - min_y) * s});
    }
    // Largest bounding box first, ties by id so the rows are reproducible.
    std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        const double area_a = a.width * a.height, area_b = b.width * b.height;
        return area_a != area_b ? area_a > area_b : a.id < b.id;
    });
    double x = target.min_x, y = target.max_y + gap, row_height = 0;
    for (const auto& item : items) {
        if (x > target.min_x + wrap_width && row_height > 0) { // New row.
            x = target.min_x;
            y += row_height + gap;
            row_height = 0;
        }
        IslandPlacement p;
        p.id = item.id;
        p.angle = 0;
        p.mirrored = false;
        p.island_scale = item.scale;
        p.tile = 0;
        p.shelved = true;
        // Unrotated: the island's bounds minimum lands exactly on (x, y).
        p.source_origin = {item.min_x * item.scale * options.target_aspect, item.min_y * item.scale};
        p.offset = {(x - target.min_x) * options.target_aspect, y - target.min_y};
        layout.placements.push_back(p);
        x += item.width + gap;
        row_height = std::max(row_height, item.height);
    }
    return items.size();
}

// Post-processing shared by the public entry points; the internal search probes skip it.
void finish_layout(Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                   const PackOptions& options) {
    if (options.shelve_unplaced) shelve_unplaced(layout, islands, target, options);
    apply_origin(layout, options, target);
}

} // namespace uvpack
