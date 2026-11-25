// Conservative stripe occupancy.
//
// Row r of an island represents the full horizontal stripe [r*w, (r+1)*w) of its
// bounding frame, not one sampled line. The occupied x-set of a region within a
// stripe equals the x-projection of that region's boundary inside the stripe:
// every vertical line that meets the region crosses either a polygon edge lying
// in the stripe or a filled cross-section on the stripe's boundary lines. So the
// row is the union of (a) x-extents of all edges clipped to the stripe and (b)
// the filled intervals on the two boundary lines. Holes are handled by the same
// argument because hole edges are part of the boundary. Fixed-point conversion
// rounds outward by one unit, so the representation never under-covers.
#include "uvpack/stripes.hpp"
#include "uvpack/geometry.hpp"
#include <memory>
#include <algorithm>
#include <cmath>

namespace uvpack {
namespace {
struct Span { double begin, end, y_low, y_high; }; // y as a fraction of the full stripe height.
constexpr float band_epsilon = 1e-6f;

// An edge of the polygon with its y extent and the reciprocal of its height, so a
// stripe can skip everything out of reach and clip what is left without dividing.
struct Edge { Vec2 a, b; double y_low, y_high, inv_dy; };

void add_edge_extent(std::vector<Span>& spans, const Edge& edge, double y0, double y1, double min_x, double stripe) {
    const Vec2 a = edge.a, b = edge.b;
    if (edge.y_high < y0 || edge.y_low > y1) return;
    double xa = a.x, xb = b.x, ya = a.y, yb = b.y;
    if (a.y != b.y) {
        // Clip the segment to [y0, y1] in parameter space.
        const double dy = b.y - a.y, dx = b.x - a.x;
        double t0 = 0, t1 = 1;
        const double ta = (y0 - a.y) * edge.inv_dy, tb = (y1 - a.y) * edge.inv_dy;
        const double tlo = std::min(ta, tb), thi = std::max(ta, tb);
        t0 = std::max(t0, tlo); t1 = std::min(t1, thi);
        if (t0 > t1) return;
        xa = a.x + dx * t0; xb = a.x + dx * t1;
        ya = a.y + dy * t0; yb = a.y + dy * t1;
    }
    spans.push_back({std::min(xa, xb) - min_x, std::max(xa, xb) - min_x,
                     (std::min(ya, yb) - y0) / stripe, (std::max(ya, yb) - y0) / stripe});
}

void add_cross_section(std::vector<Span>& spans, const std::vector<const Edge*>& active, double y,
                       double min_x, double y0, double stripe) {
    const double band = (y - y0) / stripe;
    thread_local std::vector<double> xs; // Reused: one row of a stripe raster is a few crossings.
    xs.clear();
    for (const Edge* edge : active) {
        const Vec2 a = edge->a, b = edge->b;
        if (a.y == b.y) continue;
        // Half-open rule: count the edge when y is in [min, max).
        const bool inside = (a.y <= y && y < b.y) || (b.y <= y && y < a.y);
        if (!inside) continue;
        xs.push_back(a.x + (y - a.y) * (b.x - a.x) * edge->inv_dy);
    }
    if (xs.empty()) return;
    std::sort(xs.begin(), xs.end());
    if (xs.size() % 2 != 0) {
        // Malformed input for this line; stay conservative with the full extent.
        spans.push_back({xs.front() - min_x, xs.back() - min_x, band, band});
        return;
    }
    for (std::size_t i = 0; i + 1 < xs.size(); i += 2) spans.push_back({xs[i] - min_x, xs[i + 1] - min_x, band, band});
}

rows::Row finish_row(std::vector<Span>& spans) {
    if (spans.empty()) return {};
    std::sort(spans.begin(), spans.end(), [](const Span& a, const Span& b) { return a.begin < b.begin; });
    thread_local rows::Row row;
    row.clear();
    for (const auto& s : spans) {
        const double b = std::floor(s.begin * rows::units) - 1;
        const double e = std::ceil(s.end * rows::units) + 1;
        const auto begin = static_cast<rows::Coordinate>(std::max(0.0, b));
        const auto end = static_cast<rows::Coordinate>(std::max(double(begin) + 1, e));
        float y_low = float(s.y_low), y_high = float(s.y_high);
        if (y_high > y_low) { y_low = std::max(0.0f, y_low - band_epsilon); y_high = std::min(1.0f, y_high + band_epsilon); }
        // Band-aware insertion keeps the row disjoint and only unions bands where
        // pieces truly share x, so an L-shape's notch stays free within its stripe.
        // Zero-height boundary pieces take part in the union: a bottom cross-section
        // plus a top cross-section over the same x means the material spans the stripe.
        rows::insert_piece(row, {begin, end, y_low, y_high});
    }
    // Pieces still zero-height after the union are lone boundary touches; drop them.
    rows::Row kept;
    kept.reserve(row.size());
    for (const auto& iv : row) if (iv.y_high > iv.y_low) kept.push_back(iv);
    return kept;
}
} // namespace

namespace {
rows::Shape stripe_rows(const Polygon& polygon, const Bounds& bounds, double base_y, double stripe_height,
                            std::size_t row_count, double& width) {
    // Edges sorted by their lower end: stripes go up, so one cursor adds each edge
    // when it comes into reach and one pass drops it when it is left behind. Rows then
    // cost their own crossings instead of the whole outline.
    std::vector<Edge> edges;
    auto ring_edges = [&](const Ring& ring) {
        std::size_t n = ring.size();
        if (n > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) --n;
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = ring[i], b = ring[(i + 1) % n];
            edges.push_back({a, b, std::min(a.y, b.y), std::max(a.y, b.y),
                             a.y == b.y ? 0.0 : 1.0 / (b.y - a.y)});
        }
    };
    ring_edges(polygon.outer);
    for (const auto& hole : polygon.holes) ring_edges(hole);
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) { return a.y_low < b.y_low; });
    std::vector<const Edge*> active;
    std::size_t pending = 0;

    rows::Shape rows;
    rows.reserve(row_count);
    std::vector<Span> spans; // One buffer for every stripe of this orientation.
    for (std::size_t r = 0; r < row_count; ++r) {
        const double y0 = base_y + double(r) * stripe_height;
        const double y1 = std::min(bounds.max_y, base_y + double(r + 1) * stripe_height);
        while (pending < edges.size() && edges[pending].y_low <= y1) active.push_back(&edges[pending++]);
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&](const Edge* e) { return e->y_high < y0; }), active.end());
        spans.clear();
        for (const Edge* edge : active) add_edge_extent(spans, *edge, y0, y1, bounds.min_x, stripe_height);
        if (y0 > bounds.min_y) add_cross_section(spans, active, y0, bounds.min_x, y0, stripe_height);
        if (y1 < bounds.max_y) add_cross_section(spans, active, y1, bounds.min_x, y0, stripe_height);
        auto row = finish_row(spans);
        if (!row.empty()) width = std::max(width, double(row.back().end) / rows::units);
        rows.push_back(std::move(row));
    }
    return rows;
}
} // namespace

namespace {
void union_rows(rows::Shape& into, const rows::Shape& from) {
    if (into.size() < from.size()) into.resize(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
        for (const auto iv : from[i]) rows::insert_piece(into[i], iv);
}
} // namespace

rows::Shape stripe_sub_rows(const std::vector<Polygon>& parts, const Bounds& frame, double stripe_height,
                                  std::size_t sub_row, std::size_t sub_rows, std::size_t max_rows) {
    if (sub_rows < 2 || sub_row == 0 || sub_row >= sub_rows) throw std::invalid_argument("invalid stripe sub_row");
    // Bottom raised by sub_row/sub_rows of a stripe: the partition starts below the frame.
    const double base_y = frame.min_y - stripe_height * double(sub_row) / double(sub_rows);
    const auto count = static_cast<std::size_t>(std::ceil((frame.max_y - base_y) / stripe_height));
    if (count > max_rows) throw std::length_error("stripe row limit exceeded");
    double width = frame.max_x - frame.min_x;
    // One part is the common case; rasterize straight into the result, no union pass.
    if (parts.size() == 1) return stripe_rows(parts.front(), frame, base_y, stripe_height, count, width);
    rows::Shape rows;
    for (const auto& part : parts)
        union_rows(rows, stripe_rows(part, frame, base_y, stripe_height, count, width));
    return rows;
}

SampledRaster rasterize_polygon_stripes(const Island& island, double hull_area, double stripe_height,
                                        std::size_t max_rows, const Bounds* frame) {
    const auto measures = measure_polygon(island.polygon);
    if (island.polygon.outer.size() < 3 || measures.area <= 0 || !std::isfinite(hull_area) || hull_area < 0 ||
        !std::isfinite(stripe_height) || stripe_height <= 0)
        throw std::invalid_argument("invalid stripe rasterization input");
    Bounds bounds{island.polygon.outer.front().x, island.polygon.outer.front().y,
                  island.polygon.outer.front().x, island.polygon.outer.front().y};
    for (const auto p : island.polygon.outer) {
        bounds.min_x = std::min(bounds.min_x, p.x); bounds.max_x = std::max(bounds.max_x, p.x);
        bounds.min_y = std::min(bounds.min_y, p.y); bounds.max_y = std::max(bounds.max_y, p.y);
    }
    if (frame) bounds = *frame;
    const double height = bounds.max_y - bounds.min_y;
    double width = bounds.max_x - bounds.min_x;
    if (!(height > 0) || !(width > 0)) throw std::invalid_argument("degenerate polygon bounds");
    const double rows_needed = std::ceil(height / stripe_height);
    if (!(rows_needed >= 1) || rows_needed > double(max_rows)) throw std::length_error("stripe row limit exceeded");
    const auto row_count = static_cast<std::size_t>(rows_needed);
    auto rows = stripe_rows(island.polygon, bounds, bounds.min_y, stripe_height, row_count, width);
    RasterOrientation orientation{{0, width, height}, std::move(rows), nullptr, 1, {}};
    return {{island.id, hull_area, {std::move(orientation)}}, {bounds.min_x, bounds.min_y}};
}
} // namespace uvpack
