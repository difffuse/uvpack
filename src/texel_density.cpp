#include "uvpack/texel_density.hpp"

#include <map>
#include <limits>
#include <stdexcept>

namespace uvpack {
namespace {
// Equalize density over the given item indices: the common density is the
// area-weighted mean, so the summed UV area of the set does not move.
std::size_t normalize_subset(std::vector<DensityIsland>& items, const std::vector<std::size_t>& subset) {
    double uv_sum = 0, world_sum = 0;
    for (const auto i : subset) {
        if (items[i].fixed || !(items[i].area_3d > 0)) continue;
        uv_sum += items[i].uv_area * items[i].pre_scale * items[i].pre_scale;
        world_sum += items[i].area_3d;
    }
    if (!(world_sum > 0) || !(uv_sum > 0)) return 0;
    const double mean = uv_sum / world_sum;
    std::size_t changed = 0;
    for (const auto i : subset) {
        auto& item = items[i];
        if (item.fixed || !(item.area_3d > 0)) continue;
        const double density = item.uv_area * item.pre_scale * item.pre_scale / item.area_3d;
        if (!(density > 0)) continue;
        item.pre_scale *= std::sqrt(mean / density);
        ++changed;
    }
    return changed;
}
} // namespace

std::size_t equalize_density(std::vector<DensityIsland>& items) {
    bool any_group = false;
    for (const auto& item : items) if (item.density_group >= 0) { any_group = true; break; }
    if (!any_group) {
        std::vector<std::size_t> all(items.size());
        for (std::size_t i = 0; i < items.size(); ++i) all[i] = i;
        return normalize_subset(items, all);
    }
    std::map<long long, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < items.size(); ++i)
        if (items[i].density_group >= 0) groups[items[i].density_group].push_back(i);
    std::size_t changed = 0;
    for (const auto& [id, subset] : groups) changed += normalize_subset(items, subset);
    return changed;
}

std::size_t set_texel_density(std::vector<DensityIsland>& items, double target,
                              double tex_w, double tex_h) {
    if (!(target > 0) || !(tex_w > 0) || !(tex_h > 0)) return 0;
    std::size_t changed = 0;
    for (auto& item : items) {
        if (item.fixed) continue;
        const double scaled_area = item.uv_area * item.pre_scale * item.pre_scale;
        const double current = texel_density(scaled_area, item.area_3d, tex_w, tex_h);
        if (!(current > 0)) continue; // Unknown 3D area or empty polygon: left as is.
        item.pre_scale *= density_scale_factor(current, target);
        ++changed;
    }
    return changed;
}

double set_uniform_layout_density(std::vector<DensityIsland>& items, double target,
                                  double tex_w, double tex_h) {
    auto positive=[](double x){return std::isfinite(x) && x>0;};
    if(!positive(target)||!positive(tex_w)||!positive(tex_h))
        throw std::invalid_argument("uniform density needs finite positive target and texture dimensions");
    long double cross_sum=0, area_sum=0, fixed_cross=0, movable_cross=0;
    bool any_fixed=false;
    for(const auto& item:items) {
        if(!std::isfinite(item.uv_area)||item.uv_area<0||!std::isfinite(item.area_3d)||item.area_3d<0
           ||!positive(item.pre_scale)||item.scale_multiplier!=1||!item.density_cross_sum
           ||!std::isfinite(*item.density_cross_sum)||*item.density_cross_sum<0)
            throw std::invalid_argument("uniform density needs valid original areas, cross sums, and unit multipliers");
        const long double limit=std::sqrt((long double)item.uv_area)*std::sqrt((long double)item.area_3d);
        if(*item.density_cross_sum > limit*(1+64*std::numeric_limits<double>::epsilon()))
            throw std::invalid_argument("density cross sum violates original-area consistency");
        cross_sum+=(long double)item.pre_scale * *item.density_cross_sum;
        (item.fixed?fixed_cross:movable_cross)+=(long double)item.pre_scale * *item.density_cross_sum;
        any_fixed=any_fixed || item.fixed;
        area_sum+=item.area_3d;
    }
    if(!(cross_sum>0)||!(area_sum>0)) throw std::invalid_argument("uniform density has no usable surface");
    const long double current=std::sqrt((long double)tex_w)*std::sqrt((long double)tex_h)*cross_sum/area_sum;
    double factor=static_cast<double>((long double)target/current);
    if(any_fixed) {
        if(!(movable_cross>0)) {
            if(std::abs(current-target)>64*std::numeric_limits<double>::epsilon()*std::max(current,(long double)target))
                throw std::domain_error("fixed charts cannot meet requested layout density");
            return 1;
        }
        const long double remaining=(long double)target*area_sum/
            (std::sqrt((long double)tex_w)*std::sqrt((long double)tex_h))-fixed_cross;
        if(!(remaining>0))
            throw std::domain_error("fixed charts exhaust requested layout density");
        factor=static_cast<double>(remaining/movable_cross);
    }
    if(!positive(factor)) throw std::invalid_argument("uniform density factor exceeds numeric range");
    for(const auto& item:items)
        if(!item.fixed && !positive(item.pre_scale*factor)) throw std::invalid_argument("uniform density scale exceeds numeric range");
    for(auto& item:items) if(!item.fixed) item.pre_scale*=factor;
    return factor;
}

void apply_scale_multipliers(std::vector<DensityIsland>& items) {
    for (auto& item : items)
        if (!item.fixed && item.scale_multiplier > 0 && item.scale_multiplier != 1.0) item.pre_scale *= item.scale_multiplier;
}
} // namespace uvpack
