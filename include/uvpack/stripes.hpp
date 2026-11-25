#pragma once
// Turning one oriented polygon into the horizontal stripes the placement search works
// on. See src/stripes.cpp for why a stripe's x-set is exactly the union of
// the edge extents inside it and the filled cross-sections on its two boundary lines.
#include "uvpack/stripe_placer.hpp"

namespace uvpack {

struct Bounds { double min_x, min_y, max_x, max_y; };

struct SampledRaster {
    RasterIsland raster;
    Vec2 source_origin; // original vertex - source_origin + placement offset = packed vertex.
};

// One polygon (with holes) as stripe rows of `stripe_height`, rounded outward onto the
// fixed-point grid so a layout that is interval-disjoint here is overlap-free in the
// continuous domain. `frame`, when given, fixes the raster's origin and extent so
// several parts of one island can be unioned into one row set; it must contain the
// polygon.
SampledRaster rasterize_polygon_stripes(const Island&, double hull_area, double stripe_height,
                                        std::size_t max_rows = 65536, const Bounds* frame = nullptr);

// The same rows for all `parts` of one island, with the stripe partition shifted down
// by sub_row/sub_rows of a stripe, which is how an island is placed at a sub-row height.
// The placer computes these lazily, only for the orientations it actually refines.
rows::Shape stripe_sub_rows(const std::vector<Polygon>& parts, const Bounds& frame, double stripe_height,
                                  std::size_t sub_row, std::size_t sub_rows, std::size_t max_rows = 65536);

} // namespace uvpack
