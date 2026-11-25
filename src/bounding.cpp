#include "uvpack/bounding.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uvpack {
namespace {

void check_finite(Vec2 p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::invalid_argument("nonfinite vertex in a boundary computation");
}

bool lexicographic(Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }
bool same_point(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }

// Twice the signed area of the triangle o->a->b: positive when the path turns left.
double turn(Vec2 o, Vec2 a, Vec2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// One chain of the monotone sweep. `want_left` builds the lower chain (left turns
// only), its negation the upper one. Collinear vertices are dropped: they add nothing
// to the hull and would give the caliper sweep zero-length edges to divide by.
void sweep(Ring& chain, Vec2 p, bool want_left) {
    while (chain.size() >= 2) {
        const double t = turn(chain[chain.size() - 2], chain.back(), p);
        if (want_left ? t > 0 : t < 0) break;
        chain.pop_back();
    }
    chain.push_back(p);
}

Ring opened(const Ring& ring) {
    Ring out = ring;
    if (out.size() > 1 && same_point(out.front(), out.back())) out.pop_back();
    return out;
}

} // namespace

Ring convex_boundary(Ring points) {
    for (const auto p : points) check_finite(p);
    std::sort(points.begin(), points.end(), lexicographic);
    points.erase(std::unique(points.begin(), points.end(), same_point), points.end());
    if (points.size() < 3) throw std::invalid_argument("a convex boundary needs three distinct points");
    Ring lower, upper;
    for (const auto p : points) sweep(lower, p, true);
    for (auto it = points.rbegin(); it != points.rend(); ++it) sweep(upper, *it, true);
    // Each chain ends where the other begins, so drop both duplicated endpoints.
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    if (lower.size() < 3) throw std::invalid_argument("collinear points enclose no area");
    return lower;
}

Ring polygon_boundary(const Polygon& polygon) { return convex_boundary(polygon.outer); }

EnclosingRectangle smallest_enclosing_rectangle(const Ring& boundary) {
    const Ring hull = opened(boundary);
    const std::size_t n = hull.size();
    for (const auto p : hull) check_finite(p);
    if (n < 3) throw std::invalid_argument("an enclosing rectangle needs a boundary with three vertices");

    // The four supporting vertices, as offsets from the current edge's first vertex:
    // `far` is the vertex furthest from the edge's line, `ahead` and `behind` the
    // extremes along the edge. Each only ever moves forward around the hull as the
    // edge turns, which is what makes the whole sweep linear.
    auto at = [&](std::size_t i) { return hull[i % n]; };
    auto height_of = [&](std::size_t i, std::size_t j) { // Distance * |edge|.
        return turn(at(i), at(i + 1), at(j));
    };
    auto along = [&](std::size_t i, std::size_t j) {     // Projection * |edge|.
        const Vec2 a = at(i), b = at(i + 1), p = at(j);
        return (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y);
    };

    std::size_t far = 1, ahead = 1, behind = 0;
    for (std::size_t j = 1; j < n; ++j) {
        if (height_of(0, j) > height_of(0, far)) far = j;
        if (along(0, j) > along(0, ahead)) ahead = j;
        if (along(0, j) < along(0, behind)) behind = j;
    }

    EnclosingRectangle best{0, 0, 0, std::numeric_limits<double>::infinity()};
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = at(i), b = at(i + 1);
        double dx = b.x - a.x, dy = b.y - a.y;
        const double length = std::hypot(dx, dy);
        if (!(length > 0)) throw std::invalid_argument("duplicate adjacent boundary vertices");
        if (!std::isfinite(length)) throw std::overflow_error("boundary edge length overflow");
        // Advance each support while the next hull vertex is no worse. The guards keep
        // a degenerate hull from spinning: no support passes the whole ring.
        for (std::size_t step = 0; step < n && height_of(i, far + 1) >= height_of(i, far); ++step) ++far;
        for (std::size_t step = 0; step < n && along(i, ahead + 1) >= along(i, ahead); ++step) ++ahead;
        for (std::size_t step = 0; step < n && along(i, behind + 1) <= along(i, behind); ++step) ++behind;

        const double height = height_of(i, far) / length;
        const double span = (along(i, ahead) - along(i, behind)) / length;
        const double area = height * span;
        if (!std::isfinite(area)) throw std::overflow_error("enclosing rectangle overflow");
        if (area < best.area) {
            // Walking an edge backwards describes the same rectangle, so fold the
            // direction into the right half-plane; the rotation that lays this edge on
            // the x-axis is then the one angle in [-pi/2, pi/2].
            if (dx < 0 || (dx == 0 && dy < 0)) { dx = -dx; dy = -dy; }
            best = {-std::atan2(dy, dx), std::max(span, height), std::min(span, height), area};
        }
    }
    return best;
}

} // namespace uvpack
