#include "uvpack/stripe_placer.hpp"
#include "uvpack/stripes.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <utility>

namespace uvpack {
namespace {
// How many of the hungriest island rows take part in the free-gap rejection, and how
// many get a relaxed single-row search to bound the island's offset from below.
// Both are pure filters, so these only trade probe work against searches that would
// have been thrown away; the values below were measured on the benchmark fixtures.
constexpr std::size_t gap_checks = 8;
constexpr std::size_t probe_rows = 1;
}
StripePlacer::StripePlacer(FixedTarget target, std::vector<RasterIsland> rasters)
    : target_(target), rasters_(std::move(rasters)) {
    if (!std::isfinite(target.width) || !std::isfinite(target.height) ||
        !std::isfinite(target.stripe_height) || target.width <= 0 || target.height <= 0 ||
        target.stripe_height <= 0 || target.max_candidate_rows == 0 ||
        !std::isfinite(float(target.height)) || !std::isfinite(float(target.stripe_height)) ||
        float(target.stripe_height) == 0 ||
        target.width > double(INT32_MAX) / rows::units ||
        target.height / target.stripe_height > INT32_MAX)
        throw std::invalid_argument("invalid or unrepresentable fixed target");
    std::set<std::uint64_t> ids;
    for (const auto& island : rasters_) {
        if (!ids.insert(island.id).second || !std::isfinite(island.hull_area) || island.hull_area < 0 ||
            island.orientations.empty()) throw std::invalid_argument("invalid raster island metadata");
        for (const auto& orientation : island.orientations) {
            const auto& g = orientation.geometry;
            if (!std::isfinite(g.width) || !std::isfinite(g.height) || !std::isfinite(g.angle) ||
                g.width <= 0 || g.height <= 0 || orientation.rows.empty() ||
                (orientation.rows.size() - 1) * target.stripe_height >= g.height)
                throw std::invalid_argument("orientation dimensions do not cover supplied rows");
            rows::validate(orientation.rows);
            bool nonempty = false;
            for (const auto& row : orientation.rows) {
                nonempty = nonempty || !row.empty();
                if (!row.empty() && double(row.back().end) / rows::units > g.width)
                    throw std::invalid_argument("orientation width does not cover supplied intervals");
            }
            if (!nonempty) throw std::invalid_argument("empty raster island");
        }
    }
    frame_units_ = static_cast<rows::Coordinate>(std::floor(target_.width * rows::units));
    sub_row_cache_.resize(rasters_.size());
    for (std::size_t r = 0; r < rasters_.size(); ++r) {
        sub_row_cache_[r].resize(rasters_[r].orientations.size());
        for (std::size_t o = 0; o < rasters_[r].orientations.size(); ++o) {
            const auto& orientation = rasters_[r].orientations[o];
            sub_row_cache_[r][o].resize(orientation.source && orientation.sub_row_count > 1 ? orientation.sub_row_count - 1 : 0);
        }
    }
    occupancy_.reset(frame_units_);
    orientation_index_.resize(rasters_.size());
    for (std::size_t r = 0; r < rasters_.size(); ++r) {
        orientation_index_[r].resize(rasters_[r].orientations.size());
        for (std::size_t o = 0; o < rasters_[r].orientations.size(); ++o) {
            auto& index = orientation_index_[r][o];
            for (const auto& row : rasters_[r].orientations[o].rows) {
                rows::Coordinate longest = 0;
                for (const auto iv : row) longest = std::max(longest, iv.end - iv.begin);
                index.max_segment.push_back(longest);
                if (row.empty()) index.empty_rows.push_back(static_cast<std::uint32_t>(index.max_segment.size() - 1));
                else { index.sum_begin += row.front().begin; ++index.nonempty; }
            }
            index.flat.build(rasters_[r].orientations[o].rows);
            // Worst row first: the free-gap rejection then usually fails on its first probe.
            for (std::uint32_t i = 0; i < index.max_segment.size(); ++i)
                if (index.max_segment[i] > 0) index.gap_order.push_back(i);
            std::sort(index.gap_order.begin(), index.gap_order.end(), [&](std::uint32_t a, std::uint32_t b) {
                return index.max_segment[a] > index.max_segment[b];
            });
        }
    }
}

PreparedProblem StripePlacer::prepare(const Input& input) {
    ready_ = false;
    prepared_ = {};
    raster_indices_.clear();
    group_ids_.clear();
    group_slot_.clear();
    group_box_.clear();
    group_used_.clear();
    std::set<std::uint64_t> seen;
    for (const auto& island : input.islands) {
        if (!seen.insert(island.id).second) throw std::invalid_argument("duplicate input island id");
        const auto found = std::find_if(rasters_.begin(), rasters_.end(), [&](const auto& r) { return r.id == island.id; });
        if (found == rasters_.end()) throw std::invalid_argument("missing prepared rows for input island");
        const auto measures = measure_island(island);
        if (measures.area <= 0) throw std::invalid_argument("degenerate input polygon");
        PreparedIsland prepared{island.id, island.group, measures.area, found->hull_area, measures.perimeter, {}};
        for (const auto& orientation : found->orientations) prepared.orientations.push_back(orientation.geometry);
        prepared_.islands.push_back(std::move(prepared));
        raster_indices_.push_back(static_cast<std::size_t>(found - rasters_.begin()));
    }
    prepared_.cluster_count = 1;
    // Dense group slots (group_adjacent), so the per-trial group boxes are a
    // flat array. Built unconditionally; only the weight decides whether it is read.
    for (const auto& island : prepared_.islands) group_ids_.push_back(island.group);
    std::sort(group_ids_.begin(), group_ids_.end());
    group_ids_.erase(std::unique(group_ids_.begin(), group_ids_.end()), group_ids_.end());
    for (const auto& island : prepared_.islands) group_slot_.push_back(slot_of(island.group));
    group_box_.assign(group_ids_.size(), {0, 0, 0, 0});
    group_used_.assign(group_ids_.size(), 0);
    ready_ = true;
    return prepared_;
}

std::size_t StripePlacer::slot_of(std::size_t group) const {
    const auto it = std::lower_bound(group_ids_.begin(), group_ids_.end(), group);
    return it == group_ids_.end() || *it != group ? group_ids_.size()
                                                  : static_cast<std::size_t>(it - group_ids_.begin());
}

std::optional<std::array<double, 4>> StripePlacer::group_box(std::size_t group) const {
    const auto slot = slot_of(group);
    if (slot >= group_used_.size() || !group_used_[slot]) return std::nullopt;
    return group_box_[slot];
}

double StripePlacer::group_box_area() const {
    double total = 0;
    for (std::size_t slot = 0; slot < group_used_.size(); ++slot)
        if (group_used_[slot])
            total += (group_box_[slot][2] - group_box_[slot][0]) * (group_box_[slot][3] - group_box_[slot][1]);
    return total;
}

// Pull the island towards the box its group already occupies. Two nonnegative parts,
// both zero when the island lands inside that box:
//   - the distance from the candidate's bounding-box centre to the group box, and
//   - the half-perimeter the group box would gain by swallowing the candidate.
// The second keeps a gradient after the box has grown, which is what a union-bounding-
// box compactness measure actually rewards. Like the rest of the cost this is a length
// expressed as a fraction of the frame and multiplied through by the frame's area, so
// one unit of group spread and one unit of canvas growth stay comparable at any scale.
// Nonnegative is required: row_cost_lower_bound() leaves this term out, so the greedy
// loop's early break is only exact while the term cannot be negative.
double StripePlacer::group_cost(std::size_t island, const ClearLocation& p) const {
    if (island >= group_slot_.size()) return 0.0;
    const auto slot = group_slot_[island];
    if (slot >= group_used_.size() || !group_used_[slot]) return 0.0;
    const auto& b = group_box_[slot];
    const double cx = p.x + 0.5 * p.width, cy = p.y + 0.5 * p.height;
    const double dx = std::max({0.0, b[0] - cx, cx - b[2]});
    const double dy = std::max({0.0, b[1] - cy, cy - b[3]});
    const double grown_x = std::max(b[2], p.x + p.width) - std::min(b[0], p.x);
    const double grown_y = std::max(b[3], p.y + p.height) - std::min(b[1], p.y);
    const double expansion = (grown_x - (b[2] - b[0])) + (grown_y - (b[3] - b[1]));
    return group_weight_ * target_.height * (std::sqrt(dx * dx + dy * dy) + expansion);
}

const RasterOrientation& StripePlacer::raster(std::size_t island, std::size_t orientation) const {
    return rasters_.at(raster_indices_.at(island)).orientations.at(orientation);
}

const rows::Shape& StripePlacer::rows_for(std::size_t island, std::size_t orientation, std::size_t sub_row) {
    const auto& r = raster(island, orientation);
    if (sub_row == 0) return r.rows;
    auto& slot = sub_row_cache_.at(raster_indices_.at(island)).at(orientation).at(sub_row - 1);
    if (slot.empty()) {
        const Bounds frame{r.frame[0], r.frame[1], r.frame[2], r.frame[3]};
        slot = stripe_sub_rows(*r.source, frame, target_.stripe_height, sub_row, r.sub_row_count, target_.max_candidate_rows);
        for (auto& row : slot) // Keep sub-row rasters inside the declared orientation width.
            for (auto& iv : row) iv.end = std::min(iv.end, static_cast<rows::Coordinate>(std::ceil(r.geometry.width * rows::units)));
    }
    return slot;
}

ClearLocation StripePlacer::refine(std::size_t island, std::size_t orientation, const ClearLocation& location) {
    const auto& shape = raster(island, orientation);
    const auto sub_rows = shape.source ? shape.sub_row_count : 1;
    const auto row = static_cast<std::int64_t>(std::llround(location.y / target_.stripe_height));
    if (sub_rows <= 1 || row <= 0) return location;
    auto cost_of = [&](const ClearLocation& p) {
        const double base = placement_cost(p.x, p.y, p.width, p.height, p.occupied_width, p.occupied_height,
                                           p.target_width, p.target_height, p.left_margin_sum, p.stripe_height, true);
        // The sub-row drop has to rank by the same cost the greedy loop used.
        return group_weight_ > 0 && std::isfinite(base) ? base + group_cost(island, p) : base;
    };
    const double best_cost = cost_of(location);
    const auto maximum = static_cast<rows::Coordinate>(std::floor((target_.width - shape.geometry.width) * rows::units));
    // The island can only drop into the previous stripe above the highest
    // piece under its footprint there, so start at that sub-row instead of 1. Each
    // sub-row costs a rasterization, and this makes the first try the usual winner.
    std::size_t k_min = 1;
    {
        const auto& shape_rows = occupancy_.shape();
        const auto below = static_cast<std::size_t>(row - 1);
        if (below < shape_rows.size()) {
            const auto left = static_cast<rows::Coordinate>(std::llround(location.x * rows::units));
            const auto right = static_cast<rows::Coordinate>(std::llround((location.x + location.width) * rows::units));
            float top = 0.0f;
            for (const auto iv : shape_rows[below]) {
                if (iv.end <= left) continue;
                if (iv.begin >= right) break;
                top = std::max(top, iv.y_high);
            }
            const double needed = std::ceil(double(top) * double(sub_rows) - 1e-9);
            if (needed > 1) k_min = static_cast<std::size_t>(needed);
        }
    }
    // Lowest sub-row position first: sub_row k sits at (row-1)*w + k*w/sub_rows.
    for (std::size_t k = k_min; k < sub_rows; ++k) {
        const double y = double(row - 1) * target_.stripe_height + target_.stripe_height * double(k) / double(sub_rows);
        if (y + shape.geometry.height > target_.height) continue;
        const auto& rows = rows_for(island, orientation, k);
        const auto found = rows::find_location_unchecked(occupancy_.shape(), rows, static_cast<std::size_t>(row - 1), maximum);
        if (!found) continue;
        ClearLocation candidate{double(found->offset_x) / rows::units, y, shape.geometry.width, shape.geometry.height,
            occupied_width_, occupied_height_, target_.width, target_.height, found->left_margin_sum, target_.stripe_height, true, k};
        candidate.tile = location.tile;
        candidate.island = island;
        if (cost_of(candidate) < best_cost) return candidate;
    }
    return location;
}

void StripePlacer::begin_trial(const PreparedProblem& problem, const Parameters& parameters) {
    if (!ready_ || !parameters.values.empty())
        throw std::invalid_argument("fixed placer requires prepare() and empty Parameters");
    if (problem.cluster_count != 1 || problem.islands.size() != prepared_.islands.size())
        throw std::invalid_argument("prepared problem mismatch");
    for (std::size_t i = 0; i < problem.islands.size(); ++i) {
        const auto& actual = problem.islands[i];
        const auto& expected = prepared_.islands[i];
        if (actual.id != expected.id || actual.orientations.size() != expected.orientations.size())
            throw std::invalid_argument("prepared island mismatch");
    }
    occupancy_.reset(frame_units_);
    occupied_width_ = target_.initial_occupied_width;
    occupied_height_ = target_.initial_occupied_height;
    std::fill(group_used_.begin(), group_used_.end(), char(0));
    for (const auto& fixed : fixed_) {
        double width = 0;
        for (const auto& row : fixed.rows) if (!row.empty()) width = std::max(width, double(row.back().end) / rows::units);
        commit_rows(fixed.rows, fixed.first_row, 0, 0.0, double(fixed.first_row) * target_.stripe_height,
                    width, double(fixed.rows.size()) * target_.stripe_height);
    }
}

void StripePlacer::add_fixed_occupancy(rows::Shape rows, std::size_t first_row) {
    rows::validate(rows);
    fixed_.push_back({std::move(rows), first_row});
}

const std::vector<std::int64_t>& StripePlacer::candidate_rows(std::size_t island, std::size_t orientation) {
    const auto& g = raster(island, orientation).geometry;
    row_scratch_.clear();
    if (g.width > target_.width || g.height > target_.height) return row_scratch_;
    const auto last = static_cast<std::int64_t>(std::floor((target_.height - g.height) / target_.stripe_height));
    if (static_cast<std::uint64_t>(last) >= target_.max_candidate_rows)
        throw std::length_error("candidate-row limit exceeded");
    row_scratch_.reserve(static_cast<std::size_t>(last) + 1);
    for (std::int64_t row = 0; row <= last; ++row) row_scratch_.push_back(row);
    return row_scratch_;
}

std::optional<ClearLocation> StripePlacer::find_first_clear(std::size_t island,
    std::size_t orientation, std::int64_t row, double cost_ceiling) {
    const auto& shape = raster(island, orientation);
    const auto& index = index_for(island, orientation);
    const auto first = static_cast<std::size_t>(row);
    // Free-gap rejection, worst island row first so it usually fails on its first probe.
    // Only the hungriest rows are worth testing: the rest reject almost nothing and the
    // relaxed row searches below rule out what slips through.
    const std::size_t checks = std::min<std::size_t>(index.gap_order.size(), gap_checks);
    for (std::size_t c = 0; c < checks; ++c) {
        const auto i = index.gap_order[c];
        if (index.max_segment[i] > occupancy_.row_gap(first + i)) return std::nullopt;
    }
    const auto maximum = static_cast<rows::Coordinate>(std::floor((target_.width - shape.geometry.width) * rows::units));
    if (!target_.fast_search) {
        const auto location = rows::find_location_unchecked(occupancy_.shape(), shape.rows, first, maximum);
        if (!location) return std::nullopt;
        ClearLocation found{double(location->offset_x) / rows::units, row * target_.stripe_height,
            shape.geometry.width, shape.geometry.height, occupied_width_, occupied_height_,
            target_.width, target_.height, location->left_margin_sum, target_.stripe_height, true};
        found.island = island; // The group-cohesion term reads per-island state.
        return found;
    }
    // The clearance sum is the only cost term the search does not already know,
    // and the occupancy bounds it below in constant time from the per-row last ends.
    // A candidate row whose best conceivable cost cannot beat the incumbent is dropped
    // before its exact cost, and before the rest of its search, is ever paid for. The
    // bound only grows with the offset, so it also stops a search already under way.
    const bool bounded = cost_ceiling != std::numeric_limits<double>::infinity();
    const auto hopeless = [&](rows::Coordinate x) {
        if (!bounded) return false;
        const double bound = placement_cost(double(x) / rows::units, row * target_.stripe_height,
            shape.geometry.width, shape.geometry.height, occupied_width_, occupied_height_,
            target_.width, target_.height,
            occupancy_.left_margin_window_bound(index.sum_begin, index.nonempty, index.empty_rows,
                                                first, shape.rows.size(), x),
            target_.stripe_height, true);
        // One part in a million of slack absorbs the disagreement between this double
        // evaluation and the single-precision one score_location() uses.
        return !(bound * (1 - 1e-6) - 1e-9 < cost_ceiling);
    };
    // The row that needs the most room is a relaxation of the whole island: its own
    // first clear offset bounds the island's from below, and a row with nowhere to go
    // rules the candidate row out outright. That one probe prunes most candidate rows
    // before the full search runs, and warm-starts the search that does run.
    rows::Coordinate lower = 0;
    const std::size_t probes = std::min<std::size_t>(index.gap_order.size(), probe_rows);
    for (std::size_t p = 0; p < probes; ++p) {
        const auto i = index.gap_order[p];
        const auto clear = occupancy_.row_first_clear(index.flat, i, first, lower, maximum);
        if (!clear) return std::nullopt;
        lower = *clear;
    }
    if (hopeless(lower)) return std::nullopt;
    const auto offset = occupancy_.first_clear(index.flat, first, maximum,
                                               search_scratch_, lower, hopeless);
    if (!offset) return std::nullopt;
    if (hopeless(*offset)) return std::nullopt;
    ClearLocation found{double(*offset) / rows::units, row * target_.stripe_height,
        shape.geometry.width, shape.geometry.height, occupied_width_, occupied_height_,
        target_.width, target_.height, occupancy_.left_margin_sum(index.flat, first, *offset),
        target_.stripe_height, true};
    found.island = island; // The group-cohesion term reads per-island state.
    return found;
}

// The cost rule of placement.hpp, evaluated in single precision so the CPU and the GPU
// kernel rank the same candidates the same way (the kernel has nothing wider). Every
// term is a plain multiply or add, written in the order the kernel writes them.
double StripePlacer::score_location(const ClearLocation& p) const {
    const float height = float(p.height);
    const float frame_w = float(p.target_width), frame_h = float(p.target_height);
    const float right = float(p.x) + float(p.width);
    const float top = float(p.y) + height;
    if (p.fixed_scale && (right > frame_w || top > frame_h))
        return std::numeric_limits<double>::infinity();
    const float dx = std::fmax(right - float(p.occupied_width), 0.0f);
    const float dy = std::fmax(top - float(p.occupied_height), 0.0f);
    const float growth = std::fmax(dx * frame_h, dy * frame_w);
    const float mean_clearance = float(p.left_margin_sum) * float(p.stripe_height) / height;
    const float cost = float(growth_weight) * growth + (float(p.y) * frame_w + mean_clearance * frame_h);
    // A nonnegative pull towards the group's box; nonnegative is what keeps
    // row_cost_lower_bound() a valid bound and the greedy loop's early break exact.
    return group_weight_ > 0 ? double(cost) + group_cost(p.island, p) : double(cost);
}

double StripePlacer::row_cost_lower_bound(std::size_t island, std::size_t orientation, std::int64_t row) const {
    const auto& g = raster(island, orientation).geometry;
    return row_cost_floor(double(row) * target_.stripe_height, g.height, occupied_height_, target_.width);
}

void StripePlacer::commit(std::size_t island, std::size_t orientation, const ClearLocation& location) {
    if (group_weight_ > 0 && island < group_slot_.size()) { // group_adjacent: grow the group's box.
        const auto slot = group_slot_[island];
        auto& b = group_box_[slot];
        const double x1 = location.x + location.width, y1 = location.y + location.height;
        if (!group_used_[slot]) { b = {location.x, location.y, x1, y1}; group_used_[slot] = 1; }
        else {
            b[0] = std::min(b[0], location.x); b[1] = std::min(b[1], location.y);
            b[2] = std::max(b[2], x1);         b[3] = std::max(b[3], y1);
        }
    }
    const auto& rows = rows_for(island, orientation, location.sub_row);
    const auto first = static_cast<std::size_t>(std::floor(location.y / target_.stripe_height + 1e-9));
    commit_rows(rows, first, static_cast<rows::Coordinate>(std::llround(location.x * rows::units)),
                location.x, location.y, location.width, location.height);
}

void StripePlacer::commit_rows(const rows::Shape& rows, std::size_t first, rows::Coordinate offset_units,
                                       double x, double y, double width, double height) {
    occupancy_.commit(rows, first, offset_units);
    occupied_width_ = std::max(occupied_width_, x + width);
    occupied_height_ = std::max(occupied_height_, y + height);
}

Evaluation StripePlacer::finish_trial(const std::vector<Placement>& placements, bool all_placed) {
    Evaluation result;
    result.valid = all_placed && placements.size() == prepared_.islands.size();
    if (result.valid) {
        std::vector<ClusterMeasures> contributions;
        for (const auto& island : prepared_.islands)
            contributions.push_back({ClusterScoreMode::PreScale, island.polygon_area, island.perimeter, 0});
        result.score = aggregate_score(contributions);
    }
    return result;
}
} // namespace uvpack
