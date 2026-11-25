#include "uvpack/tile_placer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace uvpack {
TilePlacer::TilePlacer(std::vector<Tile> tiles, const std::vector<RasterIsland>& rasters,
                                       std::vector<std::vector<bool>> allowed_tiles, double tile_penalty,
                                       bool group_adjacent, double group_weight)
    : allowed_(std::move(allowed_tiles)), penalty_(tile_penalty), group_adjacent_(group_adjacent) {
    if (tiles.empty()) throw std::invalid_argument("at least one target tile is required");
    for (const auto& t : tiles) {
        tiles_.push_back(std::make_unique<StripePlacer>(t.target, rasters));
        if (group_adjacent_) tiles_.back()->set_group_weight(group_weight);
    }
    if (allowed_.size() != rasters.size()) throw std::invalid_argument("allowed-tile table must cover every raster");
    for (const auto& row : allowed_)
        if (row.size() != tiles_.size() || std::find(row.begin(), row.end(), true) == row.end())
            throw std::invalid_argument("every island needs at least one allowed tile");
}

PreparedProblem TilePlacer::prepare(const Input& input) {
    PreparedProblem first;
    for (std::size_t t = 0; t < tiles_.size(); ++t) {
        auto prepared = tiles_[t]->prepare(input);
        if (t == 0) first = std::move(prepared);
    }
    // Dense group slots for the per-trial tile pinning (group_adjacent).
    group_of_.clear(); group_ids_.clear();
    for (const auto& island : first.islands) group_ids_.push_back(island.group);
    std::sort(group_ids_.begin(), group_ids_.end());
    group_ids_.erase(std::unique(group_ids_.begin(), group_ids_.end()), group_ids_.end());
    for (const auto& island : first.islands)
        group_of_.push_back(static_cast<std::size_t>(
            std::lower_bound(group_ids_.begin(), group_ids_.end(), island.group) - group_ids_.begin()));
    group_tile_.assign(group_ids_.size(), -1);
    return first;
}

std::size_t TilePlacer::group_slot(std::size_t island) const {
    return island < group_of_.size() ? group_of_[island] : group_ids_.size();
}

double TilePlacer::group_box_area() const {
    double total = 0;
    for (const auto& t : tiles_) total += t->group_box_area();
    return total;
}

void TilePlacer::add_fixed_occupancy(std::size_t tile, rows::Shape rows, std::size_t first_row) {
    tiles_.at(tile)->add_fixed_occupancy(std::move(rows), first_row);
}

void TilePlacer::begin_trial(const PreparedProblem& problem, const Parameters& parameters) {
    for (auto& t : tiles_) t->begin_trial(problem, parameters);
    std::fill(group_tile_.begin(), group_tile_.end(), -1); // group_adjacent: no group is pinned yet.
}

const std::vector<std::int64_t>& TilePlacer::candidate_rows(std::size_t island, std::size_t orientation) {
    const auto raster_index = tiles_[0]->raster_index(island);
    // One tile: its rows already come out in nondecreasing cost bound, so hand them over.
    if (tiles_.size() == 1) {
        if (!allowed_.at(raster_index).at(0)) { merged_.clear(); return merged_; }
        return tiles_[0]->candidate_rows(island, orientation);
    }
    // Merge per-tile rows by their cost lower bound so the caller's early break stays exact.
    struct Entry { double bound; std::int64_t row; };
    std::vector<Entry> entries;
    // group_adjacent: once a group has a member in a tile, the rest must follow it.
    const auto slot = group_slot(island);
    const auto pinned = group_adjacent_ && slot < group_tile_.size() ? group_tile_[slot] : -1;
    for (std::size_t t = 0; t < tiles_.size(); ++t) {
        if (!allowed_.at(raster_index).at(t)) continue;
        if (pinned >= 0 && static_cast<std::size_t>(pinned) != t) continue;
        for (const auto r : tiles_[t]->candidate_rows(island, orientation))
            entries.push_back({tiles_[t]->row_cost_lower_bound(island, orientation, r) + penalty_ * double(t),
                               static_cast<std::int64_t>(t) * row_stride + r});
    }
    std::stable_sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.bound < b.bound; });
    merged_.clear();
    for (const auto& e : entries) merged_.push_back(e.row);
    return merged_;
}

std::optional<ClearLocation> TilePlacer::find_first_clear(std::size_t island, std::size_t orientation,
                                                                 std::int64_t row, double cost_ceiling) {
    const auto t = static_cast<std::size_t>(row / row_stride);
    // score_location adds the tile penalty, so the tile sees the ceiling without it.
    const double ceiling = cost_ceiling == std::numeric_limits<double>::infinity()
        ? cost_ceiling : cost_ceiling - penalty_ * double(t);
    auto found = tiles_.at(t)->find_first_clear(island, orientation, row % row_stride, ceiling);
    if (found) found->tile = t;
    return found;
}

ClearLocation TilePlacer::refine(std::size_t island, std::size_t orientation, const ClearLocation& location) {
    auto refined = tiles_.at(location.tile)->refine(island, orientation, location);
    refined.tile = location.tile;
    return refined;
}

void TilePlacer::commit(std::size_t island, std::size_t orientation, const ClearLocation& location) {
    tiles_.at(location.tile)->commit(island, orientation, location);
    if (group_adjacent_) { // Pin the group to this tile for the rest of the trial.
        const auto slot = group_slot(island);
        if (slot < group_tile_.size() && group_tile_[slot] < 0) group_tile_[slot] = static_cast<int>(location.tile);
    }
}

Evaluation TilePlacer::finish_trial(const std::vector<Placement>& placements, bool all_placed) {
    return tiles_[0]->finish_trial(placements, all_placed);
}

double TilePlacer::score_location(const ClearLocation& location) const {
    const double cost = tiles_.at(location.tile)->score_location(location);
    return std::isfinite(cost) ? cost + penalty_ * double(location.tile) : cost;
}

double TilePlacer::row_cost_lower_bound(std::size_t island, std::size_t orientation, std::int64_t row) const {
    const auto t = static_cast<std::size_t>(row / row_stride);
    return tiles_.at(t)->row_cost_lower_bound(island, orientation, row % row_stride) + penalty_ * double(t);
}
} // namespace uvpack
