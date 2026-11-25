#pragma once
#include "uvpack/island.hpp"
#include <cmath>
#include <cstddef>
#include <vector>

namespace uvpack {
// Shoelace sum, signed so that a clockwise ring comes out positive. Open rings are
// closed implicitly; an explicit repeated closing vertex is accepted too.
double clockwise_ring_area(const Ring& ring);
double ring_perimeter(const Ring& ring);
struct PolygonMeasures { double area, perimeter; };
// Winding-independent: the outer ring's magnitude minus the holes'. Assumes simple
// rings and contained, non-intersecting holes; this is not a topology validator.
PolygonMeasures measure_polygon(const Polygon& polygon);
// Summed over the island's polygon and its rigid extra parts.
PolygonMeasures measure_island(const Island& island);

// How well one layout packed: the island area it carried, and (for a max-margin
// search) the total gap area that area is worth. Area first, margin as the tie-break.
struct Score { double area = 0.0, margin = 0.0; };

enum class ClusterScoreMode { PreScale, MaxMargin };
struct ClusterMeasures {
    ClusterScoreMode mode;
    double area, perimeter, margin_parameter = 0;
};
Score aggregate_score(const std::vector<ClusterMeasures>& clusters);
} // namespace uvpack
