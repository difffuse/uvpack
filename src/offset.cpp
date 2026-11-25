#include "uvpack/offset.hpp"
#include "uvpack/geometry.hpp"
#include "clipper.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uvpack {
namespace {
// Clipper works on integers, so the rings are put on a fixed grid first. A nanounit
// grid keeps the quantization four orders of magnitude below the smallest margin
// anyone packs with, and still leaves Clipper's 64-bit area products six decades of
// headroom at the coordinate limit below.
constexpr double clipper_grid = 1e9;
constexpr double coordinate_limit = 1e15;

ClipperLib::Path integer_ring(const Ring& ring, bool outer, double delta) {
    std::size_t count = ring.size();
    if (count > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) --count;
    if (count < 3) throw std::invalid_argument("margin ring needs at least three vertices");
    ClipperLib::Path path;
    path.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double x = ring[i].x * clipper_grid, y = ring[i].y * clipper_grid;
        if (!std::isfinite(x) || !std::isfinite(y) ||
            std::abs(x) + 2 * delta > coordinate_limit ||
            std::abs(y) + 2 * delta > coordinate_limit)
            throw std::overflow_error("margin coordinates exceed the offset grid");
        path.emplace_back(static_cast<ClipperLib::cInt>(std::llround(x)),
                          static_cast<ClipperLib::cInt>(std::llround(y)));
    }
    if (ClipperLib::Area(path) == 0)
        throw std::invalid_argument("margin ring collapsed on the offset grid");
    if (ClipperLib::Orientation(path) != outer) ClipperLib::ReversePath(path);
    return path;
}

Ring floating_ring(const ClipperLib::Path& path, bool outer) {
    Ring ring;
    ring.reserve(path.size() + 1);
    for (const auto& point : path)
        ring.push_back({double(point.X) / clipper_grid, double(point.Y) / clipper_grid});
    if (ring.size() < 3) throw std::runtime_error("degenerate margin output");
    // Outer rings come back clockwise (positive by clockwise_ring_area), holes the
    // other way round, and every ring is closed with a repeat of its first vertex.
    if ((clockwise_ring_area(ring) > 0) != outer) std::reverse(ring.begin(), ring.end());
    ring.push_back(ring.front());
    return ring;
}

double cross2(Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

// Strictly crossing segments: touching endpoints and collinear overlaps do not count.
bool proper_cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double d1 = cross2(c, d, a), d2 = cross2(c, d, b), d3 = cross2(a, b, c), d4 = cross2(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

bool self_intersects(const Ring& ring) {
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;
            if (proper_cross(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n])) return true;
        }
    return false;
}

// Crossing number against an open ring.
bool ring_contains(const Ring& ring, Vec2 p) {
    bool inside = false;
    const std::size_t n = ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = ring[i], b = ring[j];
        if ((a.y > p.y) != (b.y > p.y) &&
            p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}

bool rings_cross(const Ring& a, const Ring& b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = 0; j < b.size(); ++j)
            if (proper_cross(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()])) return true;
    return false;
}

// Move every edge of `input` a distance `r` towards its right-hand side (so the
// material is on the left) and join the moved edges. At a vertex the two offset lines
// meet at cur + r * (n0 + n1) / (1 + n0.n1); when that miter would run more than 2r
// from the corner the join is squared off instead, which is the same bound Clipper's
// miter join uses. An empty ring means "this one is degenerate, use Clipper".
Ring offset_ring(const Ring& input, double r) {
    Ring ring = input;
    if (ring.size() > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) ring.pop_back();
    Ring clean;
    for (const auto p : ring) if (clean.empty() || clean.back().x != p.x || clean.back().y != p.y) clean.push_back(p);
    if (clean.size() > 1 && clean.front().x == clean.back().x && clean.front().y == clean.back().y) clean.pop_back();
    const std::size_t n = clean.size();
    if (n < 3) return {};
    Ring out;
    out.reserve(n * 2);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 prev = clean[(i + n - 1) % n], cur = clean[i], next = clean[(i + 1) % n];
        const double dx0 = cur.x - prev.x, dy0 = cur.y - prev.y, l0 = std::hypot(dx0, dy0);
        const double dx1 = next.x - cur.x, dy1 = next.y - cur.y, l1 = std::hypot(dx1, dy1);
        if (l0 == 0 || l1 == 0) return {};
        const Vec2 n0{dy0 / l0, -dx0 / l0}, n1{dy1 / l1, -dx1 / l1}; // Right-hand normals.
        const double dot = n0.x * n1.x + n0.y * n1.y;
        const double turn = dx0 * dy1 - dy0 * dx1; // > 0: a left turn, convex on this side.
        const double denom = 1.0 + dot;            // |miter| = r * sqrt(2 / denom).
        if (turn > 0 && denom < 0.5) { // Miter longer than 2r: square the corner off.
            out.push_back({cur.x + n0.x * r + dx0 / l0 * r, cur.y + n0.y * r + dy0 / l0 * r});
            out.push_back({cur.x + n1.x * r - dx1 / l1 * r, cur.y + n1.y * r - dy1 / l1 * r});
        } else if (denom < 1e-9) {
            return {}; // A reflex hairpin: the offset folds over itself.
        } else {
            out.push_back({cur.x + r * (n0.x + n1.x) / denom, cur.y + r * (n0.y + n1.y) / denom});
        }
    }
    return out;
}
} // namespace

Polygon expand_packing_margin(const Polygon& polygon, double margin) {
    if (!std::isfinite(margin)) throw std::invalid_argument("nonfinite packing margin");
    const double radius = margin * 0.5;
    if (radius <= 0) return polygon;
    if (measure_polygon(polygon).area <= 0) throw std::invalid_argument("margin polygon has no area");
    const double delta = radius * clipper_grid;
    if (!std::isfinite(delta) || 2 * delta > coordinate_limit)
        throw std::overflow_error("packing margin exceeds the offset grid");

    ClipperLib::Paths input;
    input.push_back(integer_ring(polygon.outer, true, delta));
    for (const auto& hole : polygon.holes) input.push_back(integer_ring(hole, false, delta));
    ClipperLib::ClipperOffset offset; // Defaults: miter limit 2, the square-off bound wanted here.
    offset.AddPaths(input, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    ClipperLib::Paths paths;
    offset.Execute(paths, delta);

    Polygon result;
    for (const auto& path : paths) {
        if (ClipperLib::Orientation(path)) {
            if (!result.outer.empty()) throw std::runtime_error("margin result has multiple outer rings");
            result.outer = floating_ring(path, true);
        } else result.holes.push_back(floating_ring(path, false));
    }
    if (result.outer.empty()) throw std::runtime_error("margin result has no outer ring");
    return result;
}

Polygon expand_packing_margin_fast(const Polygon& polygon, double margin) {
    if (!std::isfinite(margin)) throw std::invalid_argument("nonfinite packing margin");
    const double r = margin * 0.5;
    if (r <= 0) return polygon;
    // Material grows to the right of each edge, so the outer ring is walked
    // counter-clockwise and the holes clockwise.
    Ring outer = polygon.outer;
    if (outer.size() > 1 && outer.front().x == outer.back().x && outer.front().y == outer.back().y) outer.pop_back();
    if (outer.size() < 3) return expand_packing_margin(polygon, margin);
    if (clockwise_ring_area(outer) > 0) std::reverse(outer.begin(), outer.end());
    Ring grown = offset_ring(outer, r);
    if (grown.size() < 3 || self_intersects(grown) || clockwise_ring_area(grown) >= 0)
        return expand_packing_margin(polygon, margin);
    Polygon result;
    result.outer = std::move(grown);
    for (auto hole : polygon.holes) {
        if (hole.size() > 1 && hole.front().x == hole.back().x && hole.front().y == hole.back().y) hole.pop_back();
        if (hole.size() < 3) continue;
        if (clockwise_ring_area(hole) < 0) std::reverse(hole.begin(), hole.end());
        Ring shrunk = offset_ring(hole, r);
        if (shrunk.size() < 3) continue; // The hole closed up: material fills it.
        const double area = clockwise_ring_area(shrunk);
        if (area <= 0 || std::abs(area) < 1e-12) continue; // Inverted or vanished.
        // A hole offset past its own centre comes back point-reflected, which keeps its
        // winding, so the sign above does not catch it. A shrunk hole must lie inside
        // the hole it came from; when it does not, the hole has closed up and the
        // material simply fills it (which is the conservative answer either way).
        bool contained = true;
        for (const auto p : shrunk) contained = contained && ring_contains(hole, p);
        if (!contained) continue;
        if (self_intersects(shrunk) || rings_cross(shrunk, result.outer)) return expand_packing_margin(polygon, margin);
        result.holes.push_back(std::move(shrunk));
    }
    // Same output convention as the Clipper path: clockwise outer, counter-clockwise
    // holes, every ring closed.
    std::reverse(result.outer.begin(), result.outer.end());
    for (auto& hole : result.holes) std::reverse(hole.begin(), hole.end());
    result.outer.push_back(result.outer.front());
    for (auto& hole : result.holes) hole.push_back(hole.front());
    return result;
}
} // namespace uvpack
