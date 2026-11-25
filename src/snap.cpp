#include "uvpack/snap.hpp"
#include <algorithm>
#include <cmath>

namespace uvpack {
namespace {

struct Box { double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300; };

void grow(Box& box, Vec2 p) {
    box.min_x = std::min(box.min_x, p.x); box.max_x = std::max(box.max_x, p.x);
    box.min_y = std::min(box.min_y, p.y); box.max_y = std::max(box.max_y, p.y);
}

// The island's polygon and its extra parts, in input order.
std::vector<const Polygon*> polygons_of(const Island& island) {
    std::vector<const Polygon*> out{&island.polygon};
    for (const auto& part : island.extra_parts) out.push_back(&part);
    return out;
}

// Apply an affine map to every vertex of every ring, preserving ring lengths and
// order so the result can be paired with the input vertex for vertex.
std::vector<Polygon> map_parts(const std::vector<const Polygon*>& parts, const std::array<double, 9>& t) {
    std::vector<Polygon> out;
    auto map_ring = [&](const Ring& ring) {
        Ring mapped;
        mapped.reserve(ring.size());
        for (const auto v : ring) mapped.push_back({t[0] * v.x + t[1] * v.y + t[2], t[3] * v.x + t[4] * v.y + t[5]});
        return mapped;
    };
    for (const auto* polygon : parts) {
        Polygon p;
        p.outer = map_ring(polygon->outer);
        for (const auto& hole : polygon->holes) p.holes.push_back(map_ring(hole));
        out.push_back(std::move(p));
    }
    return out;
}

Box bounds_of(const std::vector<Polygon>& parts) {
    Box box;
    for (const auto& p : parts) {
        for (const auto v : p.outer) grow(box, v);
        for (const auto& hole : p.holes) for (const auto v : hole) grow(box, v);
    }
    return box;
}

void for_each_vertex(std::vector<Polygon>& parts, const std::function<void(Vec2&)>& fn) {
    for (auto& p : parts) {
        for (auto& v : p.outer) fn(v);
        for (auto& hole : p.holes) for (auto& v : hole) fn(v);
    }
}

double shift_between(const std::vector<Polygon>& a, const std::vector<Polygon>& b) {
    double worst = 0;
    for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
        auto ring_shift = [&](const Ring& x, const Ring& y) {
            for (std::size_t k = 0; k < x.size() && k < y.size(); ++k)
                worst = std::max(worst, std::max(std::abs(x[k].x - y[k].x), std::abs(x[k].y - y[k].y)));
        };
        ring_shift(a[i].outer, b[i].outer);
        for (std::size_t h = 0; h < a[i].holes.size() && h < b[i].holes.size(); ++h) ring_shift(a[i].holes[h], b[i].holes[h]);
    }
    return worst;
}

} // namespace

double snap_to_texel(double coordinate, double tex_size, SnapTarget align_target) {
    if (!(tex_size > 0)) return coordinate;
    if (align_target == SnapTarget::Corner) return std::round(coordinate * tex_size) / tex_size;
    // Texel centres sit at (k + 0.5) / tex_size; the nearest one has k = round(c*T - 0.5).
    return (std::round(coordinate * tex_size - 0.5) + 0.5) / tex_size;
}

AlignResult align_layout(const Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                         double tex_size, double target_aspect, SnapMode mode, SnapTarget align_target,
                         double gap, double tolerance, const std::vector<Obstacle>& obstacles,
                         const std::vector<PackTarget>& targets,double border,const std::string& output_precision) {
    if(output_precision!="float64" && output_precision!="float32")throw std::invalid_argument("unknown output_precision");
    AlignResult result;
    const std::size_t n = layout.placements.size();
    std::vector<std::vector<Polygon>> plain(n), snapped(n);
    std::vector<std::array<double, 9>> plain_t(n), snapped_t(n);
    std::vector<char> changed(n, 0);

    for (std::size_t i = 0; i < n; ++i) {
        const auto& placement = layout.placements[i];
        AlignedIsland out;
        out.id = placement.id;
        const auto it = std::find_if(islands.begin(), islands.end(), [&](const Island& x) { return x.id == placement.id; });
        const auto t = placement.transform(layout.scale, target, target_aspect);
        plain_t[i] = t;
        snapped_t[i] = t;
        if (it == islands.end()) {
            // No geometry to align (should not happen); keep the placement as it is.
            out.transform = t;
            result.islands.push_back(std::move(out));
            continue;
        }
        const auto parts = polygons_of(*it);
        plain[i] = map_parts(parts, t);
        snapped[i] = plain[i];

        if (!(tex_size > 0)) {
            out.transform = t;
            result.islands.push_back(std::move(out));
            continue;
        }
        switch (mode) {
        case SnapMode::BBoxMin: {
            // Rigid translation onto the grid, by the island's bounding-box minimum.
            const Box box = bounds_of(plain[i]);
            const double dx = snap_to_texel(box.min_x, tex_size, align_target) - box.min_x;
            const double dy = snap_to_texel(box.min_y, tex_size, align_target) - box.min_y;
            for_each_vertex(snapped[i], [&](Vec2& v) { v.x += dx; v.y += dy; });
            snapped_t[i][2] += dx;
            snapped_t[i][5] += dy;
            break;
        }
        case SnapMode::BBox: {
            // Both bounding-box edges on the grid: a per-axis scale about the minimum.
            const Box box = bounds_of(plain[i]);
            const double w = box.max_x - box.min_x, h = box.max_y - box.min_y;
            double new_min_x = snap_to_texel(box.min_x, tex_size, align_target);
            double new_max_x = snap_to_texel(box.max_x, tex_size, align_target);
            double new_min_y = snap_to_texel(box.min_y, tex_size, align_target);
            double new_max_y = snap_to_texel(box.max_y, tex_size, align_target);
            // An island thinner than a texel has nothing to stretch onto: snapping both
            // edges of it would collapse or double it, so leave that axis alone.
            const bool fit_x = w * tex_size >= 1.0 && new_max_x > new_min_x;
            const bool fit_y = h * tex_size >= 1.0 && new_max_y > new_min_y;
            const double sx = fit_x ? (new_max_x - new_min_x) / w : 1.0;
            const double sy = fit_y ? (new_max_y - new_min_y) / h : 1.0;
            if (!fit_x) new_min_x = snap_to_texel(box.min_x, tex_size, align_target);
            if (!fit_y) new_min_y = snap_to_texel(box.min_y, tex_size, align_target);
            for_each_vertex(snapped[i], [&](Vec2& v) {
                v.x = new_min_x + (v.x - box.min_x) * sx;
                v.y = new_min_y + (v.y - box.min_y) * sy;
            });
            // The same map, composed into the emitted transform: still one affine map.
            snapped_t[i] = {sx * t[0], sx * t[1], sx * t[2] + new_min_x - sx * box.min_x,
                            sy * t[3], sy * t[4], sy * t[5] + new_min_y - sy * box.min_y,
                            0, 0, 1};
            break;
        }
        case SnapMode::Outline:
        case SnapMode::All: {
            // Every vertex on its own. The packer only ever sees boundary vertices, so
            // Border and All snap the same set here; All additionally asks the client
            // to snap the interior vertices it owns (the CLI flags that separately).
            for_each_vertex(snapped[i], [&](Vec2& v) {
                v.x = snap_to_texel(v.x, tex_size, align_target);
                v.y = snap_to_texel(v.y, tex_size, align_target);
            });
            break;
        }
        }
        out.transform = snapped_t[i];
        out.max_shift = shift_between(plain[i], snapped[i]);
        changed[i] = out.max_shift > 0 ? 1 : 0;
        out.vertices_moved = mode == SnapMode::Outline || mode == SnapMode::All;
        result.islands.push_back(std::move(out));
    }

    // Accept what validates. An island whose snapped geometry breaks containment, an
    // overlap or the gap is reverted on its own and the layout is checked again; the
    // reverted geometry is exactly what the unaligned layout had, so the loop always
    // terminates on a valid answer.
    std::vector<char> active(n, 1);
    ValidationReport report;
    auto placed_island=[&](std::size_t i,const std::vector<Polygon>& parts) {
        PlacedIsland placed{result.islands[i].id,parts,layout.placements[i].tile};
        placed.shelved=layout.placements[i].shelved;
        const auto source=std::find_if(islands.begin(),islands.end(),[&](const Island& island){return island.id==placed.id;});
        if(source!=islands.end())placed.group=source->group;
        return placed;
    };
    auto validate=[&](const std::vector<PlacedIsland>& placed) {
        if(output_precision=="float64")
            return validate_polygons(placed,target,gap,tolerance,obstacles,target_aspect,targets,border,layout.overlaid_groups);
        ValidationOptions policy;policy.output_precision=output_precision;policy.gap=gap;policy.border=border;
        policy.obstacles=obstacles;policy.targets=targets;policy.target_aspect=target_aspect;
        policy.overlaid_groups=layout.overlaid_groups;
        return validate_stored_polygons(placed,target,policy);
    };
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<PlacedIsland> placed;
        placed.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (plain[i].empty()) continue; // Placement without geometry; nothing to check.
            placed.push_back(placed_island(i,active[i]?snapped[i]:plain[i]));
        }
        report=validate(placed);
        if (report.ok) break;
        bool reverted = false;
        for (const auto id : report.offenders)
            for (std::size_t i = 0; i < n; ++i)
                if (active[i] && result.islands[i].id == id) { active[i] = 0; reverted = true; }
        if (!reverted) {
            // The problem is not something alignment introduced (or the offender list is
            // empty); give the whole thing back unaligned and report that.
            std::fill(active.begin(), active.end(), 0);
            placed.clear();
            for (std::size_t i = 0; i < n; ++i) {
                if (plain[i].empty()) continue;
                placed.push_back(placed_island(i,plain[i]));
            }
            report=validate(placed);
            break;
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        auto& out = result.islands[i];
        out.snapped = active[i] != 0;
        if (!out.snapped) {
            out.parts = plain[i];
            out.transform = plain_t[i];
            out.vertices_moved = false;
            out.max_shift = 0;
            if (changed[i]) ++result.fallbacks;
        } else {
            out.parts = snapped[i];
            if (changed[i]) ++result.moved;
            result.max_shift = std::max(result.max_shift, out.max_shift);
            if (out.vertices_moved) result.vertex_output = true;
        }
    }
    result.validation = report;
    return result;
}

} // namespace uvpack
