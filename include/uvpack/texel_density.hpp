#pragma once
// Texel density: the pre-packing scale controls that decide how much texture area an
// island gets. For an island of UV area A_uv and 3D area A_3d on a tex_w x tex_h
// texture the density is:
//
//   TD = sqrt(tex_w * tex_h) * sqrt(A_uv / A_3d)        [texels per unit of 3D length]
//
// Everything here only ever changes Island::pre_scale, so the packer itself is
// untouched: `texel_density` pre-scales to an absolute TD and then packs at a
// fixed scale, `equalize_density` levels TD across islands, and the per-island
// multiplier is a deliberate deviation applied on top.
//
// Density space (local vs global) is purely a question of which 3D area the caller
// measures: object-space areas are local, world-space areas (object scaling folded
// in) are global. The CLI therefore accepts both `area_3d` and
// `area_3d_global` per island and `density_space` picks one; nothing downstream
// changes. Islands whose chosen area is 0 (unknown) are left exactly as they are.
#include <cmath>
#include <cstddef>
#include <vector>
#include <optional>

namespace uvpack {

// TD of one island. Returns 0 when any input is unusable (unknown 3D area, empty
// polygon, degenerate texture), which callers read as "no density for this island".
inline double texel_density(double uv_area, double area_3d, double tex_w, double tex_h) {
    if (!(uv_area > 0) || !(area_3d > 0) || !(tex_w > 0) || !(tex_h > 0)) return 0.0;
    return std::sqrt(tex_w * tex_h) * std::sqrt(uv_area / area_3d);
}

// Uniform factor that takes an island of `current` density to `target`. TD is linear
// in the applied scale, so the factor is just the ratio.
inline double density_scale_factor(double current, double target) {
    return current > 0 && target > 0 ? target / current : 1.0;
}

// One island as far as the density controls are concerned. `uv_area` is the area of
// the ORIGINAL polygon (no pre-scale folded in); `pre_scale` is read and written.
struct DensityIsland {
    double uv_area = 0;
    double area_3d = 0;          // 0 = unknown; the island is never touched.
    double pre_scale = 1.0;      // In/out.
    long long density_group = -1;   // < 0 = not in any normalization group.
    double scale_multiplier = 1.0;
    std::optional<double> density_cross_sum; // Sum over original faces: sqrt(Auv*A3d).
    bool fixed = false;         // Preserve pre_scale in every density operation.
};

// Equalize density: level UV area per 3D area, i.e. give every island the same TD.
//
// The common density is the area-weighted mean sum(A_uv) / sum(A_3d) over the set, so
// the total UV area is preserved and the scale search sees the same problem size.
//
// Normalize Groups: if ANY island carries a density_group >= 0, each group is normalized
// on its own and islands without a group are left untouched. With no groups at all,
// every movable island with a known 3D area forms one set. Fixed islands are
// excluded from both the mean and rescaling, so movable UV area is preserved.
// Returns how many islands were rescaled.
std::size_t equalize_density(std::vector<DensityIsland>& items);

// Set texel density before packing: bring every island with a known 3D area to
// `target` texels per unit. Composes with whatever pre_scale is already there (so it
// can follow a normalization), and leaves unknown-area islands alone. The caller is
// expected to pack at a fixed scale of 1 afterwards. Returns how many were rescaled.
std::size_t set_texel_density(std::vector<DensityIsland>& items, double target,
                              double tex_w, double tex_h);

// One common factor from the arithmetic area-weighted face density. Throws
// invalid_argument before mutation for missing/inconsistent data or unsafe scales.
// Cauchy bound tolerance: 64 double epsilons, relative to sqrt(uv_area*area_3d).
// Fixed islands contribute their current density; one common positive factor
// scales only the movable islands to supply the remaining contribution. Throws
// domain_error without mutation if fixed contributions make the target infeasible.
// With no movable contribution, the current density must match within 64 epsilons.
double set_uniform_layout_density(std::vector<DensityIsland>& items, double target,
                                  double tex_w, double tex_h);

// Island scale multiplier (a plain factor, 1.0 = none). Applied last, on top of normalization/density, so it is a deliberate
// per-island departure from the common density.
void apply_scale_multipliers(std::vector<DensityIsland>& items);

} // namespace uvpack
