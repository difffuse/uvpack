// Static obstacles: polygons already in target space that the packed islands must
// avoid. One obstacle becomes fixed occupancy in a tile's canvas, margin-expanded,
// reflected the way the finished layout will be, and clipped to the tile's frame.
#include "search_internal.hpp"
#include "uvpack/offset.hpp"
#include "uvpack/stripes.hpp"
#include "uvpack/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace uvpack {

// Rasterize an obstacle (target space, target aspect applied) into frame rows, clipped.
void add_obstacle(TilePlacer& placer, std::size_t tile, const Obstacle& obstacle, const PackTarget& target,
                  double inset, double frame_w, double frame_h, const PackOptions& options) {
    Island fixed;
    fixed.id = 0; fixed.group = 0; fixed.polygon = obstacle.polygon;
    // Origin anchoring: the finished layout is reflected inside each tile, so the
    // search must see the obstacles reflected the same way. Reflecting within the tile
    // commutes with the clip to the tile's frame below, so an obstacle spanning tiles
    // is still handled exactly. A single-axis reflection flips the winding, which the
    // margin offset cares about, so the ring order is reversed to restore it.
    {
        const bool mirror_x = options.origin == Origin::BottomRight ||
                              options.origin == Origin::TopRight;
        const bool mirror_y = options.origin == Origin::TopLeft ||
                              options.origin == Origin::TopRight;
        if (mirror_x || mirror_y) {
            auto reflect = [&](Ring& ring) {
                for (auto& p : ring) {
                    if (mirror_x) p.x = target.min_x + target.max_x - p.x;
                    if (mirror_y) p.y = target.min_y + target.max_y - p.y;
                }
                if (mirror_x != mirror_y) std::reverse(ring.begin(), ring.end());
            };
            reflect(fixed.polygon.outer);
            for (auto& hole : fixed.polygon.holes) reflect(hole);
        }
    }
    for (auto& p : fixed.polygon.outer) p.x *= options.target_aspect;
    for (auto& hole : fixed.polygon.holes) for (auto& p : hole) p.x *= options.target_aspect;
    fixed.polygon = expand_packing_margin(fixed.polygon, options.margin + 2.0 * obstacle.extra_margin);
    // Disjoint obstacles need no occupancy. Test this in target space before
    // converting their potentially very distant origins to integer grid offsets.
    // Still validate the polygon, including holes, before ignoring it.
    if (fixed.polygon.outer.size() < 3 || measure_polygon(fixed.polygon).area <= 0)
        throw std::invalid_argument("invalid obstacle polygon");
    Bounds bounds{fixed.polygon.outer.front().x, fixed.polygon.outer.front().y,
                  fixed.polygon.outer.front().x, fixed.polygon.outer.front().y};
    for (const auto p : fixed.polygon.outer) {
        bounds.min_x = std::min(bounds.min_x, p.x); bounds.max_x = std::max(bounds.max_x, p.x);
        bounds.min_y = std::min(bounds.min_y, p.y); bounds.max_y = std::max(bounds.max_y, p.y);
    }
    const double left = target.min_x * options.target_aspect + inset;
    const double bottom = target.min_y + inset;
    if (bounds.max_x < left || bounds.min_x > left + frame_w ||
        bounds.max_y < bottom || bounds.min_y > bottom + frame_h) return;
    // Stripe endpoints use signed 32-bit fixed point, with one outward unit
    // added by the rasterizer. Reject an unrepresentable intersecting span.
    const double coordinate_limit = double(std::numeric_limits<rows::Coordinate>::max());
    if ((bounds.max_x - bounds.min_x) * rows::units > coordinate_limit - 2)
        throw std::overflow_error("obstacle width exceeds the stripe grid");
    auto raster = rasterize_polygon_stripes(fixed, 0, options.stripe_height);
    const double origin_x = raster.source_origin.x - target.min_x * options.target_aspect - inset;
    const double origin_y = raster.source_origin.y - target.min_y - inset;
    // Align the obstacle's stripes with the frame grid: re-rasterize at a sub-row offset if needed.
    const std::size_t sub_rows = 64;
    const double row_f = origin_y / options.stripe_height;
    const double offset_units = origin_x * rows::units;
    if (!std::isfinite(row_f) || std::abs(row_f) > coordinate_limit ||
        !std::isfinite(offset_units) || std::abs(offset_units) > coordinate_limit)
        throw std::overflow_error("obstacle origin exceeds the stripe grid");
    auto first_row = static_cast<std::int64_t>(std::floor(row_f));
    auto k = static_cast<std::size_t>(std::round((row_f - double(first_row)) * double(sub_rows)));
    // Rounding the fraction to the sub-row grid can reach a whole stripe: that IS the next
    // row at sub-row 0, not this row unshifted. Without the carry the obstacle sat a full
    // stripe too low and the gap above it came out one stripe_height short, which happens
    // whenever the obstacle's origin lands just under a stripe boundary.
    if (k >= sub_rows) { ++first_row; k = 0; }
    rows::Shape rows = raster.raster.orientations[0].rows;
    if (k > 0) {
        Bounds frame{raster.source_origin.x, raster.source_origin.y, raster.source_origin.x, raster.source_origin.y};
        for (const auto p : fixed.polygon.outer) { frame.max_x = std::max(frame.max_x, p.x); frame.max_y = std::max(frame.max_y, p.y); }
        rows = stripe_sub_rows(std::vector<Polygon>{fixed.polygon}, frame, options.stripe_height, k, sub_rows);
    }
    const auto ox = static_cast<std::int64_t>(std::llround(offset_units));
    const auto frame_units = static_cast<std::int64_t>(std::floor(frame_w * rows::units));
    const auto frame_rows = static_cast<std::int64_t>(std::ceil(frame_h / options.stripe_height));
    rows::Shape clipped;
    std::size_t first = 0;
    bool started = false;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto r = first_row + static_cast<std::int64_t>(i);
        if (r < 0 || r >= frame_rows) continue;
        rows::Row row;
        for (const auto iv : rows[i]) {
            const auto b = std::max<std::int64_t>(0, std::int64_t(iv.begin) + ox);
            const auto e = std::min<std::int64_t>(frame_units, std::int64_t(iv.end) + ox);
            if (e > b) row.push_back({static_cast<rows::Coordinate>(b), static_cast<rows::Coordinate>(e), iv.y_low, iv.y_high});
        }
        if (!started) { first = static_cast<std::size_t>(r); started = true; }
        clipped.push_back(std::move(row));
    }
    if (started) placer.add_fixed_occupancy(tile, std::move(clipped), first);
}

} // namespace uvpack
