#pragma once
// The placement layer: what a packing placer is handed, what it gives back, and the
// rule that decides where one island goes.
//
// A placer is given a prepared problem (one entry per island, with the orientations
// that island may be packed in) and a trial plan (an order to place them in and a mask
// of orientations to ignore this time). It returns one offset and one orientation per
// island. Everything above this header - the scale search, the tiles, the compaction -
// only ever sees those two structures.
//
// `RowPlacer` implements the greedy half of that contract once: for every island
// in turn, walk the candidate rows of every allowed orientation bottom-up, ask the
// placer for the leftmost clear x in each, score it, and commit the cheapest. The
// placer supplies the occupancy; the scoring rule lives here.
#include "uvpack/geometry.hpp"
#include "uvpack/cancellation.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace uvpack {

// One orientation an island may be packed in: the angle its geometry was turned by
// and the size of the bounding box that turn produced.
struct Orientation { double angle, width, height; };

struct PreparedIsland {
    std::uint64_t id;
    std::size_t group;
    double polygon_area, hull_area, perimeter;
    std::vector<Orientation> orientations;
    // The occupancy rows behind each orientation stay with the placer, keyed by id.
};

struct PreparedProblem {
    std::vector<PreparedIsland> islands;
    std::size_t cluster_count = 1;
};

// Placer-specific knobs for one pack() call. The rows placers take none and
// reject anything nonempty rather than silently ignoring it.
struct Parameters { std::vector<double> values; };

struct Placement {
    std::uint64_t island_id;
    std::size_t orientation;
    Vec2 offset;
    bool near_spanning = false; // Island nearly as large as the frame (see below).
    std::size_t tile = 0;       // Target index for multi-target packing.
};

struct Evaluation {
    bool valid = false; // Everything required was placed and every cluster is legal.
    Score score;
    std::vector<Placement> placements;
    std::vector<std::uint64_t> unplaced; // Islands skipped under allow_partial.
    bool cancelled = false;
};

struct TrialPlan {
    std::vector<std::size_t> island_order;           // A permutation of the islands.
    std::vector<std::vector<bool>> skip_orientation; // Per island, per orientation.
    bool allow_partial = false; // Skip an island that does not fit instead of failing.
    CancellationToken cancel;
};

// The geometry/placement boundary.
class Placer {
public:
    virtual ~Placer() = default;
    // Measure the islands and build whatever per-orientation occupancy it needs.
    virtual PreparedProblem prepare(const Input&) = 0;
    // Place every island of the plan, in the plan's order, and score the result.
    virtual Evaluation pack(const PreparedProblem&, const Parameters&, const TrialPlan&) = 0;
};

// An island whose width or height covers all but a few percent of the frame is what
// caps the uniform scale for everything else; the scale search watches for these.
// Three percent of the frame is a gap too small to park anything else in anyway.
inline bool nearly_spans_target(double width, double height,
                                double target_width, double target_height) {
    return width >= 0.97 * target_width || height >= 0.97 * target_height;
}

// --------------------------------------------------------------------- the cost rule
//
// Where an island goes is decided by two things, in this order.
//
// Primary: how much the placement grows the occupied canvas - the bounding box of
// everything placed so far, which starts empty and only grows. A placement that fits
// inside the canvas grows it by nothing and is free; one that sticks out past the
// canvas's right edge or its top pays for the larger of the two overhangs, each
// measured as a FRACTION of the frame it is allowed to fill. Measuring them as
// fractions is what makes them comparable: on a frame twice as wide as it is tall, a
// millimetre of extra width is worth half a millimetre of extra height, and a raw
// comparison of the two lengths would quietly prefer growing the short side.
//
// Secondary: among placements that grow the canvas equally (usually: not at all),
// prefer the one nearest the bottom-left corner. Two parts, again as frame fractions:
// the height the island sits at, and the mean width of the empty gap to the left of
// its rows. The second is what stops an island from parking at the far end of a long
// free run when a hole further left would have taken it.
//
// Both terms are multiplied through by the frame's area, which removes every division
// (the GPU kernel evaluates this same expression and division there is only promised
// to 2.5 ULP):
//   growth = max(dx * frame_h, dy * frame_w)          = frame_area * max(dx/frame_w, dy/frame_h)
//   settle = y * frame_w + mean_clearance * frame_h   = frame_area * (y/frame_h + c/frame_w)
//   cost   = growth_weight * growth + settle
//
// The weight: `settle` is bounded by twice the frame's area, and the smallest growth
// worth telling apart is around a thousandth of the frame, so 1e5 puts that smallest
// growth at a hundred times the entire range of `settle` - any real growth outranks
// every bottom-left preference - while keeping the sum well inside float32's mantissa,
// so `settle` still decides among the placements that grow the canvas by the same
// amount. Callers that add their own nonnegative terms (group cohesion) add them on
// top; the greedy loop's row bound below stays valid as long as they are nonnegative.
inline constexpr double growth_weight = 1.0e5;

// `bounded` frames reject a placement that leaves the frame outright; open frames (the
// strip and extent passes) let it spill and let the growth term price it.
inline double placement_cost(double x, double y, double width, double height,
                             double occupied_width, double occupied_height,
                             double frame_width, double frame_height,
                             double left_margin_sum, double stripe_height, bool bounded) {
    if (height <= 0.0 || (bounded &&
        (x + width > frame_width || y + height > frame_height)))
        return std::numeric_limits<double>::infinity();
    const double dx = std::max(0.0, x + width - occupied_width);
    const double dy = std::max(0.0, y + height - occupied_height);
    const double growth = std::max(dx * frame_height, dy * frame_width);
    // The island's rows each leave some empty width to their left; stripe_height / height
    // is one over the number of rows, so this is their mean.
    const double mean_clearance = left_margin_sum * stripe_height / height;
    return growth_weight * growth + (y * frame_width + mean_clearance * frame_height);
}

// The smallest cost any clear location in this row could possibly have: the width
// overhang and the clearance are both nonnegative, so only the height overhang and the
// row's own height survive. Exact whenever the island lands right of everything below
// it, and monotone in the row, which is what lets the greedy stop climbing.
inline double row_cost_floor(double y, double height, double occupied_height, double frame_width) {
    return frame_width * (growth_weight * std::max(0.0, y + height - occupied_height) + y);
}

struct ClearLocation {
    double x, y;
    double width, height; // Orientation size after this trial's scaling and margin.
    double occupied_width, occupied_height;
    double target_width, target_height;
    double left_margin_sum, stripe_height;
    bool fixed_scale;
    std::size_t sub_row = 0; // Sub-row start offset index (0 = on the row boundary).
    std::size_t tile = 0;  // Target index for multi-target placers.
    // The prepared-island index this location belongs to, so a cost hook can reach
    // per-island state (its group) without threading it through the greedy loop.
    std::size_t island = 0;
};

// The greedy half of Placer, shared by the single-target and multi-target
// placers. Occupancy, candidate rows and the leftmost-clear search are the placer's.
class RowPlacer : public Placer {
public:
    static inline const bool debug_placements = std::getenv("UVPACK_DEBUG") != nullptr;
    Evaluation pack(const PreparedProblem& problem, const Parameters& parameters,
                    const TrialPlan& plan) final {
        auto cancelled=[] {Evaluation e;e.cancelled=true;return e;};
        if(plan.cancel.requested())return cancelled();
        if (plan.island_order.size() != problem.islands.size() ||
            plan.skip_orientation.size() != problem.islands.size())
            throw std::invalid_argument("trial plan must cover every prepared island");
        std::vector<bool> seen(problem.islands.size(), false);
        for (auto index : plan.island_order) {
            if (index >= seen.size() || seen[index])
                throw std::invalid_argument("trial order must be a permutation");
            seen[index] = true;
            if (plan.skip_orientation[index].size() != problem.islands[index].orientations.size())
                throw std::invalid_argument("orientation-mask size mismatch");
        }
        // 1. Rebuild the trial's occupancy and whatever the placer caches per trial.
        begin_trial(problem, parameters);
        std::vector<Placement> placements;
        std::vector<std::uint64_t> unplaced;
        for (const auto index : plan.island_order) {
            if(plan.cancel.requested())return cancelled();
            const auto& island = problem.islands.at(index);
            std::optional<ClearLocation> winner;
            std::size_t winning_orientation = 0;
            double best_cost = std::numeric_limits<double>::infinity();

            // 2. Every allowed orientation, every candidate row, bottom-up.
            for (std::size_t o = 0; o < island.orientations.size(); ++o) {
                if (plan.skip_orientation.at(index).at(o)) continue;
                for (const auto row : candidate_rows(index, o)) {
                    if(plan.cancel.requested())return cancelled();
                    // Rows come in nondecreasing height and the cost has an exact lower
                    // bound per row, so once no row can win, none above it can either.
                    if (row_cost_lower_bound(index, o, row) >= best_cost) break;
                    // 3. Leftmost x in this row at which nothing overlaps. The
                    //    incumbent cost lets a placer give up on a row early.
                    auto location = find_first_clear(index, o, row, best_cost);
                    if (!location) continue;
                    const double cost = score_location(*location);
                    // 4. First win on a tie: rows and orientations are enumerated in a
                    //    fixed order, so this is reproducible.
                    if (cost < best_cost) {
                        winner = location;
                        winning_orientation = o;
                        best_cost = cost;
                    }
                }
            }
            if(plan.cancel.requested())return cancelled();
            if (!winner) {
                if (plan.allow_partial) { unplaced.push_back(island.id); continue; }
                if (debug_placements) std::fprintf(stderr, "FAIL island=%zu after %zu placed\n", index, placements.size());
                auto failed = finish_trial(placements, false);
                failed.valid = false;
                failed.placements = std::move(placements);
                return failed;
            }
            // 5. Try to drop the winner into the stripe below it at a sub-row offset.
            if(plan.cancel.requested())return cancelled();
            *winner = refine(index, winning_orientation, *winner);
            if (debug_placements) std::fprintf(stderr, "place island=%zu o=%zu x=%.5f y=%.5f cost=%.3f\n", index, winning_orientation, winner->x, winner->y, best_cost);
            commit(index, winning_orientation, *winner);
            placements.push_back({island.id, winning_orientation, {winner->x, winner->y},
                nearly_spans_target(winner->width, winner->height,
                                    winner->target_width, winner->target_height), winner->tile});
        }
        // 6. Score the finished layout.
        auto result = finish_trial(placements, unplaced.empty());
        result.valid = result.valid && unplaced.empty();
        result.placements = std::move(placements);
        result.unplaced = std::move(unplaced);
        return result;
    }

protected:
    // Exact lower bound of score_location for any clear location in this row;
    // -infinity disables the early break. Rows must be enumerated in nondecreasing y.
    virtual double row_cost_lower_bound(std::size_t, std::size_t, std::int64_t) const {
        return -std::numeric_limits<double>::infinity();
    }
    // Optional post-selection refinement (sub-row placement). Default: identity.
    virtual ClearLocation refine(std::size_t, std::size_t, const ClearLocation& location) { return location; }
    virtual double score_location(const ClearLocation& p) const {
        return placement_cost(p.x, p.y, p.width, p.height,
            p.occupied_width, p.occupied_height, p.target_width, p.target_height,
            p.left_margin_sum, p.stripe_height, p.fixed_scale);
    }
    virtual void begin_trial(const PreparedProblem&, const Parameters&) = 0;
    // The returned rows stay valid until the next call on the same placer.
    virtual const std::vector<std::int64_t>& candidate_rows(std::size_t island,
                                                           std::size_t orientation) = 0;
    // `cost_ceiling` is the incumbent cost: a placer may also return nullopt for a
    // clear location it can prove scores at or above it.
    virtual std::optional<ClearLocation> find_first_clear(std::size_t island,
        std::size_t orientation, std::int64_t row, double cost_ceiling) = 0;
    virtual void commit(std::size_t island, std::size_t orientation, const ClearLocation&) = 0;
    virtual Evaluation finish_trial(const std::vector<Placement>&, bool all_placed) = 0;
};

} // namespace uvpack
