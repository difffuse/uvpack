#include "uvpack/orientations.hpp"
#include "uvpack/bounding.hpp"
#include "uvpack/offset.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace uvpack {
namespace {
const double pi = std::acos(-1.0);

Vec2 rotate(Vec2 point, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {point.x * c - point.y * s, point.x * s + point.y * c};
}

// Two orientations are interchangeable when their rows are the same intervals in the
// same bands - the two halves of a shape with 180-degree symmetry, for instance. That
// is an exact test on what the search actually reads, so dropping one of them cannot
// change a single placement; it only makes the trial cheaper.
bool same_rows(const RasterOrientation& a, const RasterOrientation& b) {
    if (a.geometry.width != b.geometry.width || a.geometry.height != b.geometry.height ||
        a.rows.size() != b.rows.size()) return false;
    for (std::size_t i = 0; i < a.rows.size(); ++i) {
        if (a.rows[i].size() != b.rows[i].size()) return false;
        for (std::size_t j = 0; j < a.rows[i].size(); ++j) {
            const auto& p = a.rows[i][j];
            const auto& q = b.rows[i][j];
            if (p.begin != q.begin || p.end != q.end || p.y_low != q.y_low || p.y_high != q.y_high)
                return false;
        }
    }
    return true;
}
} // namespace

std::vector<double> orientation_angles(bool allow_rotation, bool prealign,
                                       int step_degrees, double prealign_angle) {
    const double base = allow_rotation && prealign ? prealign_angle : 0;
    if (!std::isfinite(base)) throw std::invalid_argument("nonfinite prealign angle");
    std::vector<double> angles{base};
    if (!allow_rotation || step_degrees <= 0 || step_degrees > 180) return angles;
    // Whole steps around the full turn. Each angle is computed from the base rather
    // than accumulated, so the k-th angle carries one rounding, not k of them.
    const double step = double(step_degrees) * pi / 180.0;
    const int count = 360 / step_degrees;
    for (int k = 1; k < count; ++k) angles.push_back(base + double(k) * step);
    return angles;
}

Vec2 OrientationTransform::map_vertex(Vec2 original, Vec2 placement_offset) const {
    if (mirrored) original.x = -original.x;
    const auto transformed = rotate(original, angle);
    return {transformed.x - source_origin.x + placement_offset.x,
            transformed.y - source_origin.y + placement_offset.y};
}

PreparedOrientations prepare_polygon_orientations(const Island& input, const Ring& hull,
                                                  double hull_area, const OrientationSettings& settings) {
    double prealign_angle = 0;
    if (settings.allow_rotation && settings.prealign)
        prealign_angle = smallest_enclosing_rectangle(hull).angle;
    const auto angles = orientation_angles(settings.allow_rotation, settings.prealign,
                                           settings.step_degrees, prealign_angle);
    // The schedule, each angle first upright and then (optionally) mirrored in x.
    std::vector<std::pair<double, bool>> schedule;
    for (const auto angle : angles) schedule.emplace_back(angle, false);
    if (settings.mirror) for (const auto angle : angles) schedule.emplace_back(angle, true);

    RasterIsland raster{input.id, hull_area, {}};
    std::vector<OrientationTransform> transforms;
    std::vector<std::size_t> source_indices;
    for (std::size_t index = 0; index < schedule.size(); ++index) {
        settings.cancel.throw_if_requested();
        const double angle = schedule[index].first;
        const bool mirrored = schedule[index].second;
        auto place = [&](Vec2 p) { if (mirrored) p.x = -p.x; return rotate(p, angle); };
        auto turn_polygon = [&](Polygon polygon) {
            for (auto& p : polygon.outer) p = place(p);
            for (auto& hole : polygon.holes) for (auto& p : hole) p = place(p);
            if (mirrored) { // Mirroring reverses winding; put it back.
                std::reverse(polygon.outer.begin(), polygon.outer.end());
                for (auto& hole : polygon.holes) std::reverse(hole.begin(), hole.end());
            }
            return polygon;
        };
        Island turned = input;
        turned.polygon = turn_polygon(input.polygon);
        for (auto& part : turned.extra_parts) part = turn_polygon(part);

        // All the parts of one island share a single frame, so their rows line up and
        // can simply be unioned row by row.
        Bounds frame{turned.polygon.outer.front().x, turned.polygon.outer.front().y,
                     turned.polygon.outer.front().x, turned.polygon.outer.front().y};
        auto grow = [&](const Polygon& poly) {
            for (const auto p : poly.outer) {
                frame.min_x = std::min(frame.min_x, p.x); frame.max_x = std::max(frame.max_x, p.x);
                frame.min_y = std::min(frame.min_y, p.y); frame.max_y = std::max(frame.max_y, p.y);
            }
        };
        grow(turned.polygon);
        for (const auto& part : turned.extra_parts) grow(part);

        auto rasterized = rasterize_polygon_stripes(turned, hull_area, settings.stripe_height,
                                                    settings.max_rows, &frame);
        for (const auto& part : turned.extra_parts) {
            Island extra;
            extra.id = turned.id;
            extra.group = turned.group;
            extra.polygon = part;
            const auto more = rasterize_polygon_stripes(extra, hull_area, settings.stripe_height,
                                                        settings.max_rows, &frame);
            auto& rows = rasterized.raster.orientations[0].rows;
            const auto& add = more.raster.orientations[0].rows;
            if (rows.size() < add.size()) rows.resize(add.size());
            for (std::size_t i = 0; i < add.size(); ++i)
                for (const auto interval : add[i]) rows::insert_piece(rows[i], interval);
            auto& geometry = rasterized.raster.orientations[0].geometry;
            geometry.width = std::max(geometry.width, more.raster.orientations[0].geometry.width);
        }
        auto orientation = std::move(rasterized.raster.orientations[0]);
        orientation.geometry.angle = angle;

        if (settings.dedup_orientations) {
            bool duplicate = false;
            for (const auto& kept : raster.orientations) duplicate = duplicate || same_rows(kept, orientation);
            if (duplicate) continue;
        }
        transforms.push_back({angle, rasterized.source_origin, mirrored});
        source_indices.push_back(index);
        if (settings.sub_rows > 1) {
            auto parts = std::make_shared<std::vector<Polygon>>();
            parts->push_back(turned.polygon);
            for (const auto& part : turned.extra_parts) parts->push_back(part);
            orientation.source = parts;
            orientation.sub_row_count = settings.sub_rows;
            orientation.frame = {frame.min_x, frame.min_y, frame.max_x, frame.max_y};
        }
        raster.orientations.push_back(std::move(orientation));
    }
    return {std::move(raster), std::move(transforms), std::move(source_indices), angles.size()};
}

PreparedOrientations prepare_polygon_orientations(const Island& original,
                                                  const OrientationSettings& settings, double margin) {
    Ring all_points(original.polygon.outer.begin(), original.polygon.outer.end());
    for (const auto& part : original.extra_parts)
        all_points.insert(all_points.end(), part.outer.begin(), part.outer.end());
    const auto original_hull = original.extra_parts.empty() ? polygon_boundary(original.polygon)
                                                            : convex_boundary(all_points);
    const double original_hull_area = std::abs(clockwise_ring_area(original_hull));

    Island prepared = original;
    prepared.polygon = expand_packing_margin_fast(original.polygon, margin);
    for (auto& part : prepared.extra_parts) part = expand_packing_margin_fast(part, margin);
    Ring packing_points(prepared.polygon.outer.begin(), prepared.polygon.outer.end());
    for (const auto& part : prepared.extra_parts)
        packing_points.insert(packing_points.end(), part.outer.begin(), part.outer.end());
    const auto packing_hull = prepared.extra_parts.empty() ? polygon_boundary(prepared.polygon)
                                                            : convex_boundary(packing_points);
    return prepare_polygon_orientations(prepared, packing_hull, original_hull_area, settings);
}
} // namespace uvpack
