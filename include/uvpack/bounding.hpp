#pragma once
// Two shape measurements the packer needs before it can place anything: the convex
// boundary of a point set, and the smallest rectangle - at any angle - that encloses
// it. The second is what decides the angle an island is pre-aligned to, so that a
// diagonal sliver is packed lying flat instead of across the diagonal of a box that is
// mostly empty.
#include "uvpack/geometry.hpp"

namespace uvpack {

// Convex hull by monotone chain: sort the points, sweep once for the lower chain and
// once for the upper, dropping any vertex that does not turn left. The result is
// counter-clockwise, starts at the lexicographically smallest point and is OPEN (the
// first vertex is not repeated at the end). Duplicate points are fine; fewer than
// three distinct points, or points that are all collinear, throw - there is no shape
// to pack there. Input may be open or closed.
Ring convex_boundary(Ring points);
// The hull of a polygon's outer ring. Holes lie inside it and cannot change it.
Ring polygon_boundary(const Polygon& polygon);

// The smallest-area rectangle enclosing a convex boundary, and the rotation that puts
// it on the axes. `angle` is in radians, in [-pi/2, pi/2]: rotating the shape by it
// leaves the rectangle's long edges horizontal.
struct EnclosingRectangle {
    double angle;
    double width, height; // Measured along the rectangle's own axes, width >= height.
    double area;
};
// One side of a smallest enclosing rectangle always lies on a hull edge, so only the
// hull's own edge directions have to be tried; rotating calipers walks the four
// supporting vertices around the hull once as the edge turns, so the whole sweep is
// linear in the number of hull vertices. `boundary` must be a convex ring in
// counter-clockwise order, open or closed, with at least three distinct vertices.
EnclosingRectangle smallest_enclosing_rectangle(const Ring& boundary);

} // namespace uvpack
