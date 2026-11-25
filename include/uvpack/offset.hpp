#pragma once
// Growing an island by its share of the packing gap.
//
// A run's `margin` is the gap the caller wants between two packed islands. Every
// island is therefore grown by HALF of it before it is rasterized: two islands whose
// grown outlines merely touch are exactly `margin` apart. Holes shrink by the same
// half-margin, so something packed inside a hole keeps the gap as well.
#include "uvpack/stripes.hpp"

namespace uvpack {

// Miter offset computed directly from the rings, with no external library: each edge
// moves outward by half the margin and consecutive offset edges are intersected. Sharp
// convex corners are squared off rather than run to an unbounded spike, which makes
// the result a superset of a round offset and therefore still conservative. When the
// offset would self-intersect (a thin neck folding over, a hole collapsing onto the
// outline) it hands the polygon to `expand_packing_margin` instead. This is the offset
// every packed island goes through.
Polygon expand_packing_margin_fast(const Polygon&, double margin);

// The same expansion computed by Clipper's polygon offsetter, which resolves the
// self-intersections the direct offset declines. Used for static obstacles and by the
// compact stage's exact overlap tests, and as the fallback above. Nonpositive
// margins copy the polygon unchanged. Assumes simple rings with contained, disjoint
// holes; the output is closed, with a clockwise outer ring.
Polygon expand_packing_margin(const Polygon&, double margin);

} // namespace uvpack
