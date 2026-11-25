#pragma once
// Every angle one island may be packed at, and the occupancy rows for each of them.
//
// An island is offered a fixed schedule of orientations: an optional prealign step
// that lays the shape on the axes of its own smallest enclosing rectangle, then even
// steps of `rotation_step` degrees all the way round, each optionally mirrored. The
// placement search picks one of them per trial; nothing here is a search.
#include "uvpack/stripes.hpp"
#include "uvpack/cancellation.hpp"

namespace uvpack {

// The schedule, in radians: `prealign_angle + k * step` for k in [0, 360 / step).
// With rotation off there is exactly one angle, zero. The prealign angle comes from
// smallest_enclosing_rectangle(); pass 0 (or clear `prealign`) to keep the island in
// the orientation it arrived in.
std::vector<double> orientation_angles(bool allow_rotation, bool prealign,
                                       int step_degrees, double prealign_angle);

struct OrientationSettings {
    bool allow_rotation = true;
    bool prealign = true;             // Lay the island on the axes of its own minimum-area rectangle first.
    int step_degrees = 90;
    double stripe_height = 0.002;
    std::size_t max_rows = 65536;
    std::size_t sub_rows = 1; // Sub-row placement resolution; 1 = row-aligned.
    bool mirror = false;              // Also offer each angle mirrored in x.
    bool dedup_orientations = false;  // Drop angles whose rows come out exactly equal.
    CancellationToken cancel;
};

// How to map a vertex of the original island onto its packed position: mirror it if
// this orientation is mirrored, rotate it, then move the orientation's bounding-box
// minimum to the placement offset the search chose.
struct OrientationTransform {
    double angle;
    Vec2 source_origin;    // Bounds minimum AFTER mirroring and rotation.
    bool mirrored = false;
    Vec2 map_vertex(Vec2 original, Vec2 placement_offset) const;
};

struct PreparedOrientations {
    RasterIsland raster;
    std::vector<OrientationTransform> transforms; // One per raster orientation.
    std::vector<std::size_t> source_indices;      // Their positions in the full schedule.
    std::size_t candidate_count;                  // Angles in the schedule before mirroring.
};

// Rasterize one already margin-expanded island at every orientation of its schedule.
// `packing_hull` is the convex boundary of the same expanded geometry and is only read
// when prealign is on. `original_hull_area` is caller metadata used for ordering.
PreparedOrientations prepare_polygon_orientations(const Island& packing_geometry,
                                                  const Ring& packing_hull,
                                                  double original_hull_area,
                                                  const OrientationSettings&);

// The whole preparation for one island: measure its original hull, grow it by the
// margin, take the convex boundary of the grown geometry, then orient and rasterize.
// Hand the ORIGINAL island to the placer's prepare(), so area scoring stays unpadded.
PreparedOrientations prepare_polygon_orientations(const Island& original,
                                                  const OrientationSettings&,
                                                  double margin = 0);

} // namespace uvpack
