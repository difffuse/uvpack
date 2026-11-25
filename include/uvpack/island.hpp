#pragma once
// The geometry the packer is given: plain polygons in UV space, grouped into islands.
//
// Nothing here knows about packing. A ring is a closed loop of vertices, given either
// open or with a repeated first vertex; a polygon is one outer ring plus any number of
// hole rings inside it; an island is one or more polygons that move as a rigid body,
// with the per-island controls a caller may set on it. Winding is not prescribed:
// every consumer normalises it from the signed area.
#include <cstddef>
#include <cstdint>
#include <vector>

namespace uvpack {

struct Vec2 { double x, y; };
using Ring = std::vector<Vec2>;
struct Polygon { Ring outer; std::vector<Ring> holes; };

// One packable body. `polygon` is its shape; `extra_parts` are further polygons that
// keep their relative position to it (islands bound together, stacked duplicates).
// Parts may overlap each other - they are one body, not a partition.
struct Island {
    std::uint64_t id = 0;
    std::size_t group = 0;
    Polygon polygon;
    // Per-island overrides. The defaults keep the three-field aggregate initialiser
    // `{id, group, polygon}` valid, and every negative value means "use the run's
    // setting" rather than a value of its own.
    bool scale_locked = false;  // Hold this island at scale 1 while the rest scale.
    int rotation_step = -1;     // Degrees between orientations; < 0 = the run's step.
    int rotation_mode = -1;     // -1 run default, 0 never rotate, 1 rotate.
    int flip_mode = -1;         // -1 run default, 0 never mirror, 1 may mirror.
    double pre_scale = 1.0;     // Applied to this island alone before packing.
    std::vector<Polygon> extra_parts;
    Ring envelope_points; // Original chart corners for explicit conservative recovery.
};

struct Input { std::vector<Island> islands; };

} // namespace uvpack
