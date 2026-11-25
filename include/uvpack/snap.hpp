#pragma once
// Pixel-perfect alignment: after packing,
// pull each island onto the texel grid of a texture of `tex_size` texels per UV
// unit, so that a texture baked from the layout has no half-texel bleeding at the
// island borders.
//
// Four modes, all of which snap to either the texel CORNERS (multiples of
// 1/tex_size) or the texel CENTRES ((k + 0.5)/tex_size):
//
//   Corners  Translate the island so its bounding-box *minimum* lands on the grid.
//            A rigid move; the island keeps its shape and its transform stays a
//            similarity. This is what `snap_to_texels()` in scale_search.cpp does and what
//            the CLI has always emitted, so it is the default.
//   BBox     Snap the bounding box so that BOTH its minimum and its maximum land on
//            the grid. The island therefore has to be stretched slightly and
//            independently in u and v -- a per-island non-uniform scale. Still one
//            affine map, so the result is still a `transform`; no per-vertex output
//            is needed.
//   Border   Snap every boundary vertex on its own: the outer ring of every part and
//            every hole ring (hole borders bleed like outer ones). Vertices move
//            independently, so the island is no longer an affine image of its input
//            and the result has to be emitted as per-vertex UVs.
//   All      Every vertex. The polygons this packer is handed are outlines, so at
//            this level All snaps the same vertex set as Border; the difference is
//            that All also asks the *client* to snap the interior vertices it owns
//            and the packer never saw (the CLI says so with "snap_interior": true).
//
// No vertex moves by more than half a texel on either axis in any mode, which is
// why the CLI reserves one texel of island gap and half a texel of border gap
// before packing: two islands can approach each other by at most one texel.
// Alignment is nevertheless revalidated, and any island whose snapped geometry
// breaks containment, overlap or the requested gap is reverted on its own.
#include "uvpack/packer.hpp"
#include "uvpack/validator.hpp"

namespace uvpack {

// One island's aligned geometry, in output UV space (what `transform` maps into).
struct AlignedIsland {
    std::uint64_t id = 0;
    bool snapped = false;        // False: reverted, this island kept the plain transform.
    bool vertices_moved = false; // True: `parts` is not the affine image of `transform`.
    std::array<double, 9> transform{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    // Island polygon first, then its extra parts, vertex for vertex in the order of
    // the corresponding Island's rings.
    std::vector<Polygon> parts;
    double max_shift = 0;        // Largest per-axis vertex move, in UV units.
};

struct AlignResult {
    std::vector<AlignedIsland> islands; // Parallel to layout.placements.
    std::size_t moved = 0;              // Islands whose geometry actually changed.
    std::size_t fallbacks = 0;          // Islands reverted because they broke the layout.
    bool vertex_output = false;         // Some island needs per-vertex UVs emitted.
    double max_shift = 0;
    ValidationReport validation;        // Of the geometry in `islands`.
};

// Align `layout` and validate the result. `gap` is the island-to-island distance the
// aligned layout must still honour (the CLI passes the margin the caller asked for,
// not the inflated one it packed with). Islands are reverted one at a time until the
// layout validates again, so the return value is always at least as good as the
// input; in the worst case every island falls back and nothing moves.
// Float32 checks rounded polygons before retaining snaps; the default float64
// path preserves caller-supplied numerical tolerance. Overlay/shelf policy comes
// from the layout. Invalid input can still yield an invalid fallback report.
AlignResult align_layout(const Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                         double tex_size, double target_aspect, SnapMode mode, SnapTarget align_target,
                         double gap, double tolerance = -1, const std::vector<Obstacle>& obstacles = {},
                         const std::vector<PackTarget>& targets = {}, double border = 0,
                         const std::string& output_precision = "float64");

// The coordinate snap the modes above are built from, exposed for tests and for
// clients that have to snap vertices the packer never saw (SnapMode::All).
double snap_to_texel(double coordinate, double tex_size, SnapTarget align_target);

} // namespace uvpack
