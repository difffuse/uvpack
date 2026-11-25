#include "uvpack/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uvpack {
namespace {
void validate(const Ring& ring) {
    for (const auto& point : ring)
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            throw std::invalid_argument("nonfinite polygon vertex");
}
}

double clockwise_ring_area(const Ring& ring) {
    validate(ring);
    if (ring.size() < 3) return 0;
    double sum = 0;
    auto previous = ring.back();
    for (const auto point : ring) {
        sum += (previous.y - point.y) * (previous.x + point.x);
        previous = point;
    }
    return sum * 0.5;
}

double ring_perimeter(const Ring& ring) {
    validate(ring);
    if (ring.size() < 2) return 0;
    double sum = 0;
    auto previous = ring.back();
    for (const auto point : ring) {
        const double dx = previous.x - point.x, dy = previous.y - point.y;
        sum += std::sqrt(dx * dx + dy * dy);
        previous = point;
    }
    return sum;
}

PolygonMeasures measure_polygon(const Polygon& polygon) {
    PolygonMeasures result{std::abs(clockwise_ring_area(polygon.outer)), ring_perimeter(polygon.outer)};
    for (const auto& hole : polygon.holes) {
        result.area -= std::abs(clockwise_ring_area(hole));
        result.perimeter += ring_perimeter(hole);
    }
    if (!std::isfinite(result.area) || !std::isfinite(result.perimeter) || result.area < 0)
        throw std::invalid_argument("invalid polygon measures");
    return result;
}

Score aggregate_score(const std::vector<ClusterMeasures>& clusters) {
    Score result;
    for (const auto& cluster : clusters) {
        if (!std::isfinite(cluster.area) || !std::isfinite(cluster.perimeter) ||
            !std::isfinite(cluster.margin_parameter) || cluster.area < 0 || cluster.perimeter < 0)
            throw std::invalid_argument("invalid cluster measures");
        result.area += cluster.area;
        if (cluster.mode == ClusterScoreMode::MaxMargin)
            result.margin += cluster.perimeter * cluster.margin_parameter * 0.5;
    }
    return result;
}

PolygonMeasures measure_island(const Island& island) {
    auto total = measure_polygon(island.polygon);
    for (const auto& part : island.extra_parts) {
        const auto m = measure_polygon(part);
        total.area += m.area;
        total.perimeter += m.perimeter;
    }
    return total;
}
} // namespace uvpack
