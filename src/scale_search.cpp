#include "thread_group.hpp"
// The uniform max-scale search and the public packing entry points.
//
// Stages: `bracket` narrows a feasible scale by k-section (or bisection without
// parallel rounds), `refine` spends the time budget on adaptive upward steps with fresh
// trial rounds, and `compact` (src/compaction.cpp) grows the incumbent in place. The
// pieces this drives live next door: src/single_pass.cpp, src/overlay_groups.cpp,
// src/clusters.cpp, src/obstacles.cpp and src/finish.cpp. The public surface is
// include/uvpack/packer.hpp; what the layers underneath do is described in README.md.
#include "search_internal.hpp"
#include "uvpack/geometry.hpp"
#include "uvpack/gpu.hpp"
#include "uvpack/offset.hpp"
#include "uvpack/stripes.hpp"
#include "uvpack/tile_placer.hpp"
#include "uvpack/validator.hpp"
#include "uvpack/compaction.hpp"
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cmath>
#include <functional>
#include <list>
#include <memory>
#include <limits>
#include <numeric>
#include <thread>

namespace uvpack {
namespace {
using Clock = std::chrono::steady_clock;

CancellationToken g_cancel;

// Packing-space geometry: pre-scale, uniform scale (unless locked), target aspect on x.
Island scaled_island(const Island& island, double scale, double target_aspect) {
    Island out = island;
    const double s = island_scale_for(island, scale);
    auto map = [&](Vec2 p) { return Vec2{p.x * target_aspect * s, p.y * s}; };
    auto scale_polygon = [&](Polygon& poly) {
        for (auto& p : poly.outer) p = map(p);
        for (auto& hole : poly.holes) for (auto& p : hole) p = map(p);
    };
    scale_polygon(out.polygon);
    for (auto& part : out.extra_parts) scale_polygon(part);
    return out;
}

// 3x3 row-major product.
std::array<double, 9> mul(const std::array<double, 9>& a, const std::array<double, 9>& b) {
    std::array<double, 9> r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
}

struct PreparedSet {
    std::vector<Island> scaled;
    std::vector<RasterIsland> rasters;
    std::vector<std::vector<OrientationTransform>> transforms; // Per island, aligned with raster orientations.
};

PreparedSet prepare_all(const std::vector<Island>& islands, double scale, const PackOptions& options) {
    PreparedSet set;
    for (const auto& island : islands) {
        options.cancel.throw_if_requested();
        OrientationSettings settings;settings.cancel=options.cancel;
        settings.allow_rotation = options.allow_rotation;
        settings.prealign = options.prealign;
        settings.step_degrees = options.rotation_step;
        settings.stripe_height = options.stripe_height;
        settings.sub_rows = std::max<std::size_t>(1, options.sub_rows);
        settings.mirror = options.allow_mirroring;
        settings.dedup_orientations = options.dedup_orientations;
        if (island.rotation_mode >= 0) settings.allow_rotation = island.rotation_mode == 1;
        if (island.rotation_step > 0) settings.step_degrees = island.rotation_step;
        if (island.flip_mode >= 0) settings.mirror = island.flip_mode == 1;
        auto scaled = scaled_island(island, scale, options.target_aspect);
        auto prepared = prepare_polygon_orientations(scaled, settings, options.margin);
        set.rasters.push_back(std::move(prepared.raster));
        set.transforms.push_back(std::move(prepared.transforms));
        set.scaled.push_back(std::move(scaled));
    }
    return set;
}

// --- Trial plans: the ordering strategies a greedy attempt can be built from. ---
enum class OrderKey { HullArea, PolygonArea, Height, Width, Perimeter, LongestSide, Random, Count };

double order_measure(const PreparedIsland& island, OrderKey key) {
    auto dims = [&](bool height) {
        double best = 0;
        for (const auto& o : island.orientations) best = std::max(best, height ? o.height : o.width);
        return best;
    };
    switch (key) {
    case OrderKey::HullArea: return island.hull_area;
    case OrderKey::PolygonArea: return island.polygon_area;
    case OrderKey::Height: return dims(true);
    case OrderKey::Width: return dims(false);
    case OrderKey::Perimeter: return island.perimeter;
    case OrderKey::LongestSide: return std::max(dims(true), dims(false));
    default: return 0;
    }
}

// group_adjacent: island index -> group id, plus the contiguous blocks of the
// current order. Empty group_of means the feature is off and every helper is a no-op.
struct GroupInfo {
    std::vector<std::size_t> group_of; // Per prepared-island index.
    bool on() const { return !group_of.empty(); }
    std::size_t of(std::size_t island) const { return group_of[island]; }
};

// Contiguous [begin, end) runs of equal group in `order`. Requires a grouped order.
std::vector<std::pair<std::size_t, std::size_t>> group_blocks(const std::vector<std::size_t>& order,
                                                              const GroupInfo& groups) {
    std::vector<std::pair<std::size_t, std::size_t>> blocks;
    for (std::size_t i = 0; i < order.size();) {
        std::size_t j = i + 1;
        while (j < order.size() && groups.of(order[j]) == groups.of(order[i])) ++j;
        blocks.emplace_back(i, j);
        i = j;
    }
    return blocks;
}

// Mild adjacent perturbation: with probability p pick the second remaining item.
void perturb_order(std::vector<std::size_t>& order, std::mt19937& rng, double p) {
    std::vector<std::size_t> out;
    out.reserve(order.size());
    std::list<std::size_t> remaining(order.begin(), order.end());
    std::uniform_real_distribution<double> u(0, 1);
    while (!remaining.empty()) {
        auto it = remaining.begin();
        if (remaining.size() > 1 && u(rng) < p) ++it;
        out.push_back(*it);
        remaining.erase(it);
    }
    order = std::move(out);
}

TrialPlan make_plan(const PreparedProblem& problem, std::size_t strategy, std::mt19937& rng,
                    const GroupInfo& groups = {}) {
    const auto key = static_cast<OrderKey>(strategy % static_cast<std::size_t>(OrderKey::Count));
    TrialPlan plan;
    plan.island_order.resize(problem.islands.size());
    std::iota(plan.island_order.begin(), plan.island_order.end(), 0);
    if (key == OrderKey::Random) {
        std::shuffle(plan.island_order.begin(), plan.island_order.end(), rng);
    } else {
        std::vector<double> measure(problem.islands.size());
        for (std::size_t i = 0; i < measure.size(); ++i) measure[i] = order_measure(problem.islands[i], key);
        std::stable_sort(plan.island_order.begin(), plan.island_order.end(),
                         [&](std::size_t a, std::size_t b) { return measure[a] > measure[b]; });
        // First strategy of each key is the pure order; later repeats add noise.
        if (strategy >= static_cast<std::size_t>(OrderKey::Count)) perturb_order(plan.island_order, rng, 0.3);
    }
    // group_adjacent: keep every strategy above, then make the order
    // group-contiguous. Groups go by summed polygon area descending (shuffled for the
    // random strategy); islands keep the strategy's relative order inside their group.
    if (groups.on()) {
        std::map<std::size_t, double> area;
        for (std::size_t i = 0; i < problem.islands.size(); ++i) area[groups.of(i)] += problem.islands[i].polygon_area;
        std::vector<std::pair<std::size_t, double>> ranked(area.begin(), area.end());
        std::stable_sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
        if (key == OrderKey::Random) std::shuffle(ranked.begin(), ranked.end(), rng);
        std::map<std::size_t, std::size_t> rank;
        for (std::size_t r = 0; r < ranked.size(); ++r) rank[ranked[r].first] = r;
        std::stable_sort(plan.island_order.begin(), plan.island_order.end(),
                         [&](std::size_t a, std::size_t b) { return rank[groups.of(a)] < rank[groups.of(b)]; });
    }
    for (const auto& island : problem.islands) plan.skip_orientation.emplace_back(island.orientations.size(), false);
    return plan;
}

// Move k random islands to random positions (insertion moves).
void insertion_moves(std::vector<std::size_t>& order, std::mt19937& rng, std::size_t k) {
    if (order.size() < 2) return;
    std::uniform_int_distribution<std::size_t> pick(0, order.size() - 1);
    for (std::size_t i = 0; i < k; ++i) {
        const auto from = pick(rng), to = pick(rng);
        if (from == to) continue;
        const auto value = order[from];
        order.erase(order.begin() + static_cast<std::ptrdiff_t>(from));
        order.insert(order.begin() + static_cast<std::ptrdiff_t>(to), value);
    }
}

// group_adjacent: the same diversification restricted to moves that preserve
// group contiguity - reinsert an island inside its own block, or move a whole block.
void grouped_insertion_moves(std::vector<std::size_t>& order, std::mt19937& rng, std::size_t k,
                             const GroupInfo& groups) {
    if (order.size() < 2) return;
    std::uniform_real_distribution<double> coin(0, 1);
    for (std::size_t i = 0; i < k; ++i) {
        auto blocks = group_blocks(order, groups);
        if (blocks.empty()) return;
        std::uniform_int_distribution<std::size_t> pick_block(0, blocks.size() - 1);
        if (blocks.size() > 1 && coin(rng) < 0.5) {
            // Whole group to another group boundary.
            const auto b = pick_block(rng);
            std::vector<std::size_t> moved(order.begin() + std::ptrdiff_t(blocks[b].first),
                                           order.begin() + std::ptrdiff_t(blocks[b].second));
            order.erase(order.begin() + std::ptrdiff_t(blocks[b].first),
                        order.begin() + std::ptrdiff_t(blocks[b].second));
            auto rest = group_blocks(order, groups);
            const auto at = std::uniform_int_distribution<std::size_t>(0, rest.size())(rng);
            const auto pos = at < rest.size() ? rest[at].first : order.size();
            order.insert(order.begin() + std::ptrdiff_t(pos), moved.begin(), moved.end());
        } else {
            const auto block = blocks[pick_block(rng)];
            const auto span = block.second - block.first;
            if (span < 2) continue;
            std::uniform_int_distribution<std::size_t> inside(0, span - 1);
            const auto from = block.first + inside(rng), to = block.first + inside(rng);
            if (from == to) continue;
            const auto value = order[from];
            order.erase(order.begin() + std::ptrdiff_t(from));
            order.insert(order.begin() + std::ptrdiff_t(to), value);
        }
    }
}

// Randomly forbid some orientations per island (keeps at least one), so trials
// explore different orientation choices instead of always taking the greedy best.
void random_masks(TrialPlan& plan, std::mt19937& rng, double probability) {
    std::uniform_real_distribution<double> u(0, 1);
    for (auto& mask : plan.skip_orientation) {
        std::size_t kept = 0;
        for (std::size_t o = 0; o < mask.size(); ++o) {
            mask[o] = u(rng) < probability;
            if (!mask[o]) ++kept;
        }
        if (kept == 0) mask[std::uniform_int_distribution<std::size_t>(0, mask.size() - 1)(rng)] = false;
    }
}

void move_to_front(std::vector<std::size_t>& order, std::size_t island) {
    auto it = std::find(order.begin(), order.end(), island);
    if (it == order.end() || it == order.begin()) return;
    std::rotate(order.begin(), it, it + 1);
}

// group_adjacent: failure repair that keeps the order group-contiguous. The
// island that did not fit goes to the front of its own block, and its block to the front.
void move_to_front_grouped(std::vector<std::size_t>& order, std::size_t island, const GroupInfo& groups) {
    auto it = std::find(order.begin(), order.end(), island);
    if (it == order.end()) return;
    const auto at = static_cast<std::size_t>(it - order.begin());
    for (const auto& block : group_blocks(order, groups)) {
        if (at < block.first || at >= block.second) continue;
        std::rotate(order.begin() + std::ptrdiff_t(block.first), order.begin() + std::ptrdiff_t(at),
                    order.begin() + std::ptrdiff_t(at) + 1);
        std::rotate(order.begin(), order.begin() + std::ptrdiff_t(block.first),
                    order.begin() + std::ptrdiff_t(block.second));
        return;
    }
}

// Group cohesion: compactness in [0, 1] -> the weight of the group term added to the
// location cost. The placer multiplies that term by the frame's height, exactly as the
// cost's own clearance term is, so w / growth_weight is the exchange rate between "keep
// the group tight" and "do not grow the canvas", and the whole thing is scale invariant.
//   c = 0   -> 1e2:  outranks the bottom-left preference but loses to any real growth,
//                    i.e. a tie-break on top of the group-contiguous order.
//   c = 0.5 -> ~3e3: worth about 3 percent of canvas growth.
//   c = 1   -> 1e5:  a unit of group spread costs as much as a unit of canvas growth.
// Measured on the `grouped` fixture, going beyond parity stops helping and starts to
// make the greedy erratic, so the ramp tops out there.
double group_weight_for(double compactness) {
    return std::pow(10.0, 2.0 + 3.0 * std::min(1.0, std::max(0.0, compactness)));
}

// A CPU trial plan in the device's flattened shape. Island indices are raster
// indices (prepare() keeps the two in step), and the per-island orientation mask
// becomes one bit per orientation.
gpu::TrialSpec to_trial_spec(const TrialPlan& plan, std::size_t island_count) {
    gpu::TrialSpec spec;
    spec.island_order.reserve(plan.island_order.size());
    for (const auto index : plan.island_order) spec.island_order.push_back(static_cast<std::uint32_t>(index));
    spec.skip_mask.assign(island_count, 0u);
    for (std::size_t i = 0; i < plan.skip_orientation.size() && i < island_count; ++i)
        for (std::size_t o = 0; o < plan.skip_orientation[i].size() && o < gpu::max_orientations; ++o)
            if (plan.skip_orientation[i][o]) spec.skip_mask[i] |= 1u << o;
    return spec;
}

// One dispatch of extra whole trials, run on a helper thread so the device works
// while the CPU threads work. Nothing here can fail the pack: every path that goes
// wrong records a reason in `skipped` and leaves `ran` false.
struct GpuRound {
    std::vector<gpu::TrialSpec> plans;
    std::vector<std::vector<std::size_t>> orders;
    std::vector<gpu::TrialResult> results;
    gpu::WorkerStats device_stats;
    std::string device_name;
    std::string skipped;
    double estimate = 0;   // Seconds this problem's batch is expected to take.
    bool retry = false;    // Skipped only for lack of time; the device itself is fine.
    bool ran = false;
    double wait_seconds = 0;
    CancellationToken cancel;
    std::thread thread;

    void launch(const std::vector<RasterIsland>& rasters, const FixedTarget& target) {
        if (plans.empty()) {
            if (skipped.empty()) skipped = "no device trials requested";
            return;
        }
        thread = std::thread([this, &rasters, &target] {
            if(cancel.requested()){skipped="packing cancelled";return;}
            gpu::WorkerLease lease;
            if (!lease) { skipped = gpu::lease_diagnostic(); return; }
            device_name = lease->device().name;
            const auto before = lease->stats();
            if(cancel.requested()){skipped="packing cancelled";return;}
            if (!lease->upload(rasters, target)) { skipped = lease->error(); return; }
            if (plans.size() > lease->trial_capacity()) plans.resize(lease->trial_capacity());
            if (plans.empty()) { skipped = "device capacity is below one trial"; return; }
            if(cancel.requested()){skipped="packing cancelled";return;}
            if (!lease->run(plans, results)) { skipped = lease->error(); return; }
            const auto after = lease->stats();
            device_stats.dispatches = after.dispatches - before.dispatches;
            device_stats.trials = after.trials - before.trials;
            device_stats.valid_trials = after.valid_trials - before.valid_trials;
            device_stats.upload_seconds = after.upload_seconds - before.upload_seconds;
            device_stats.compute_seconds = after.compute_seconds - before.compute_seconds;
            ran = true;
        });
    }

    void join() {
        if (!thread.joinable()) return;
        const auto start = Clock::now();
        thread.join();
        wait_seconds += std::chrono::duration<double>(Clock::now() - start).count();
    }

    ~GpuRound() { if (thread.joinable()) thread.join(); }
};

// One device batch at one scale, start to finish, on the caller's thread.
// Returns candidate layouts in trial order, UNVALIDATED - the caller runs the exact
// validator before using any of them.
//
// It repeats pack_impl's preparation rather than sharing it because it runs on its own
// thread, at its own scale, while the CPU rounds are using theirs.
// What a batch of this problem will cost the device, from the size of the work
// it actually has to do. Calibrated on an Apple M2 at stripe height 0.002: mixed100
// (22k island rows) took 4.4 s, mixed300 (38k) 8.5 s, mixed1000 (69k) 21.8 s, for
// batches of 32 to 48 - the cost is one workgroup's latency, near enough independent
// of how many trials ride along. A single rate cannot fit all three (the per-row cost
// grows with how full the canvas gets), so this one fits the small end and is
// optimistic at the large end by about 1.4x; the large end is nowhere near fitting a
// short budget anyway, which is the decision this actually has to get right.
// Before this guard existed, mixed1000 with a 5 s budget ran for 23 s.
double estimated_batch_seconds(const std::vector<RasterIsland>& rasters) {
    std::size_t rows = 0;
    for (const auto& island : rasters)
        for (const auto& orientation : island.orientations) rows += orientation.rows.size();
    return 2.2e-4 * double(rows);
}

std::vector<Layout> gpu_batch(const std::vector<Island>& islands, const PackTarget& target,
                              double scale, double reach, const PackOptions& options,
                              std::size_t count, double deadline_seconds, GpuRound& round) {
    std::vector<Layout> candidates;
    if (count == 0) { round.skipped = "no device trials requested"; return candidates; }
    try {
        auto set = prepare_all(islands, scale, options);
        const double border = options.border_margin < 0 ? options.margin : options.border_margin;
        const double inset = std::max(0.0, border - 0.5 * options.margin);
        const double frame_w = (target.max_x - target.min_x) * options.target_aspect - 2 * inset;
        const double frame_h = target.max_y - target.min_y - 2 * inset;
        if (!(frame_w > 0) || !(frame_h > 0)) { round.skipped = "border margin leaves no frame"; return candidates; }
        FixedTarget fixed{frame_w, frame_h, options.stripe_height};
        fixed.fast_search = options.fast_search;
        round.skipped = gpu::unsupported_reason(set.rasters, fixed, 1, false, options.group_adjacent,
                                                options.allow_partial);
        if (!round.skipped.empty()) return candidates;
        // Now that the real size is known, check the batch against the time it was
        // given. Nothing can cancel a dispatch once it is submitted, so this is the
        // last point at which the budget can still be honoured.
        round.estimate = estimated_batch_seconds(set.rasters);
        if (deadline_seconds > 0 && round.estimate > deadline_seconds) {
            round.skipped = "a batch needs about " + std::to_string(round.estimate) +
                            " s and only " + std::to_string(deadline_seconds) + " s remain";
            round.retry = true;
            return candidates;
        }

        // The ordering strategies the CPU workers would have used, continued past them.
        PreparedProblem prepared;
        for (std::size_t i = 0; i < islands.size(); ++i) {
            const auto measures = measure_island(set.scaled[i]);
            PreparedIsland island{islands[i].id, islands[i].group, measures.area,
                                  set.rasters[i].hull_area, measures.perimeter, {}};
            for (const auto& orientation : set.rasters[i].orientations)
                island.orientations.push_back(orientation.geometry);
            prepared.islands.push_back(std::move(island));
        }
        std::vector<std::size_t> warm;
        for (const auto id : options.warm_order) {
            std::size_t index = 0;
            while (index < islands.size() && islands[index].id != id) ++index;
            if (index < islands.size()) warm.push_back(index);
        }
        if (warm.size() != islands.size()) warm.clear();
        // The device's own bracket. The rasters are uploaded once, at `scale`, and
        // each trial packs them into a frame shrunk by its own factor; scaling that
        // layout back up to the target multiplies the scale by the same factor and grows
        // every gap, so one dispatch spans the whole ladder [scale, scale * (1 + reach)].
        // A single fixed guess at where the CPU will finish almost always misses, so the
        // batch spreads its aim across the ladder instead.
        const std::size_t steps = std::min<std::size_t>(count, 8);
        const std::size_t per_step = std::max<std::size_t>(1, count / steps);
        for (std::size_t g = 0; g < count; ++g) {
            std::mt19937 rng(options.seed * 7919u + static_cast<std::uint32_t>(g) * 104729u + 17u);
            TrialPlan plan = make_plan(prepared, g + options.seed, rng);
            const std::size_t rung = std::min(steps - 1, g / per_step);
            // One warm start from the incumbent's order per rung, then noisy variants.
            if (!warm.empty() && g % per_step == 0) {
                plan.island_order = warm;
            } else {
                if (g % 3 == 1) insertion_moves(plan.island_order, rng, 1 + plan.island_order.size() / 10);
                if (g % 3 == 2) random_masks(plan, rng, 0.3);
            }
            auto spec = to_trial_spec(plan, islands.size());
            spec.frame_shrink = 1.0 + reach * double(rung) / double(std::max<std::size_t>(1, steps - 1));
            round.plans.push_back(std::move(spec));
            round.orders.push_back(plan.island_order);
        }

        gpu::WorkerLease lease;
        if (!lease) { round.skipped = gpu::lease_diagnostic(); return candidates; }
        round.device_name = lease->device().name;
        const auto before = lease->stats();
        options.cancel.throw_if_requested();
        if (!lease->upload(set.rasters, fixed)) { round.skipped = lease->error(); return candidates; }
        if (round.plans.size() > lease->trial_capacity()) round.plans.resize(lease->trial_capacity());
        if (round.plans.empty()) { round.skipped = "device capacity is below one trial"; return candidates; }
        options.cancel.throw_if_requested();
        if (!lease->run(round.plans, round.results)) { round.skipped = lease->error(); return candidates; }
        const auto after = lease->stats();
        round.device_stats.dispatches = after.dispatches - before.dispatches;
        round.device_stats.trials = after.trials - before.trials;
        round.device_stats.valid_trials = after.valid_trials - before.valid_trials;
        round.device_stats.upload_seconds = after.upload_seconds - before.upload_seconds;
        round.device_stats.compute_seconds = after.compute_seconds - before.compute_seconds;
        round.ran = true;

        for (std::size_t t = 0; t < round.results.size(); ++t) {
            const auto& result = round.results[t];
            if (!result.valid || result.placements.size() != islands.size()) continue;
            // Bring the shrunken-frame layout back to the target: everything scales by
            // the trial's shrink factor about the frame origin, which is the same
            // rescale the CPU fast pass does. Gaps grow by that factor, so a layout that
            // honoured the margin in the small frame honours it here too.
            const double f = result.frame_shrink;
            Layout layout;
            layout.valid = true;
            layout.scale = scale * f;
            layout.occupied_width = result.occupied_width * f;
            layout.occupied_height = result.occupied_height * f;
            if (t < round.orders.size())
                for (const auto index : round.orders[t]) layout.order.push_back(islands[index].id);
            for (const auto& placement : result.placements) {
                std::size_t index = 0;
                while (index < islands.size() && islands[index].id != placement.island_id) ++index;
                if (index >= islands.size()) { layout.valid = false; break; }
                auto transform = set.transforms.at(index).at(placement.orientation);
                transform.source_origin.x *= f;
                transform.source_origin.y *= f;
                layout.placements.push_back({placement.island_id, transform.angle, transform.source_origin,
                    {placement.offset.x * f + inset, placement.offset.y * f + inset}, transform.mirrored,
                    island_scale_for(islands[index], scale) * f, 0});
            }
            if (layout.valid) candidates.push_back(std::move(layout));
        }
        // Largest scale first, then trial order: the caller validates in this order and
        // takes the first that passes, so the batch's answer is its best valid rung.
        std::stable_sort(candidates.begin(), candidates.end(),
                         [](const Layout& a, const Layout& b) { return a.scale > b.scale; });
    } catch (const std::exception& ex) {
        round.skipped = ex.what();
    }
    return candidates;
}

// The device works its own scale ladder while the CPU improvement rounds run.
// Nothing ever waits on it mid-round: a finished batch is collected when the CPU comes
// back round, its layouts go through the exact validator, and one becomes the incumbent
// only if it validates AND its scale beats the incumbent. The CPU rounds therefore run
// exactly as they would with no device at all, and the device can only add scale.
class GpuImprover {
public:
    GpuImprover(const std::vector<Island>& islands, const PackTarget& target, const PackOptions& options)
        : islands_(islands), target_(target), options_(options) {}
    ~GpuImprover() { join(); }

    // Seconds the next batch is expected to take. Before anything has been measured
    // this is only a floor: the batch itself re-checks against the real problem size
    // once it has prepared the rasters, which is the check that actually binds.
    double expected_seconds() const { return measured_ > 0 ? measured_ : estimate_; }
    bool busy() const { return thread_.joinable(); }
    bool ready() const { return thread_.joinable() && done_.load(std::memory_order_acquire); }
    // A reason the device is out for good. Running out of time for one batch is not one.
    const std::string& skipped() const { return skipped_; }

    void start(double scale, double reach, std::uint32_t seed, std::vector<std::uint64_t> warm,
               std::size_t count, double deadline_seconds) {
        if (thread_.joinable() || count == 0) return;
        PackOptions batch = options_;
        batch.seed = seed;
        batch.warm_order = std::move(warm);
        batch.trials = 1;
        scale_ = scale;
        reach_ = reach;
        round_.plans.clear();
        round_.orders.clear();
        round_.results.clear();
        round_.device_stats = {};
        round_.skipped.clear();
        round_.ran = false;
        candidates_.clear();
        done_.store(false, std::memory_order_release);
        const auto start = Clock::now();
        thread_ = std::thread([this, batch, count, start, deadline_seconds] {
            candidates_ = gpu_batch(islands_, target_, scale_, reach_, batch, count,
                                    deadline_seconds, round_);
            if (round_.ran) measured_ = std::chrono::duration<double>(Clock::now() - start).count();
            done_.store(true, std::memory_order_release);
        });
    }

    // Join the batch and hand back its candidates; the caller validates them.
    std::vector<Layout> collect(SearchStats& stats) {
        join();
        stats.gpu_dispatches += round_.device_stats.dispatches;
        stats.gpu_trials += round_.device_stats.trials;
        stats.gpu_evaluations += round_.device_stats.valid_trials;
        stats.gpu_upload_seconds += round_.device_stats.upload_seconds;
        stats.gpu_compute_seconds += round_.device_stats.compute_seconds;
        if (!round_.device_name.empty()) stats.gpu_device = round_.device_name;
        if (round_.estimate > 0) estimate_ = round_.estimate;
        if (!round_.skipped.empty()) {
            stats.gpu_skipped = round_.skipped;
            // Out of time for this batch says nothing about the next one; anything else
            // is a standing reason not to try again.
            if (!round_.retry) skipped_ = round_.skipped;
        }
        return std::move(candidates_);
    }

    void join() { if (thread_.joinable()) thread_.join(); }
    double scale() const { return scale_; }

private:
    const std::vector<Island>& islands_;
    const PackTarget& target_;
    PackOptions options_;
    std::thread thread_;
    std::atomic<bool> done_{false};
    std::vector<Layout> candidates_;
    GpuRound round_;
    double scale_ = 0;
    double reach_ = 0;
    double measured_ = 0;
    double estimate_ = 0.5;  // Floor until the first batch reports the real one.
    std::string skipped_;
};

void merge_gpu_stats(SearchStats& into, const SearchStats& from) {
    into.gpu_dispatches += from.gpu_dispatches;
    into.gpu_trials += from.gpu_trials;
    into.gpu_evaluations += from.gpu_evaluations;
    into.gpu_accepted += from.gpu_accepted;
    into.gpu_used += from.gpu_used;
    into.gpu_upload_seconds += from.gpu_upload_seconds;
    into.gpu_compute_seconds += from.gpu_compute_seconds;
    into.gpu_wait_seconds += from.gpu_wait_seconds;
    if (!from.gpu_device.empty()) into.gpu_device = from.gpu_device;
    if (!from.gpu_skipped.empty()) into.gpu_skipped = from.gpu_skipped;
}
} // namespace

double island_scale_for(const Island& island, double scale) {
    return island.pre_scale * (island.scale_locked ? 1.0 : scale);
}

std::size_t thread_count(const PackOptions& options) {
    if (options.threads > 0) return options.threads;
    const auto hw = std::thread::hardware_concurrency();
    return hw == 0 ? 4 : hw;
}

std::vector<PackTarget> effective_targets(const PackTarget& target, const PackOptions& options) {
    return options.targets.empty() ? std::vector<PackTarget>{target} : options.targets;
}

std::array<double, 9> IslandPlacement::transform(double, const PackTarget& target, double target_aspect) const {
    const double c = std::cos(angle), sn = std::sin(angle), s = island_scale, m = mirrored ? -1.0 : 1.0;
    // D(1/ratio) * Tr * R * S * M * D(ratio)
    const std::array<double, 9> ratio_in{target_aspect, 0, 0, 0, 1, 0, 0, 0, 1};
    const std::array<double, 9> rsm{c * s * m, -sn * s, 0, sn * s * m, c * s, 0, 0, 0, 1};
    const std::array<double, 9> tr{1, 0, -source_origin.x + offset.x + target.min_x * target_aspect,
                                   0, 1, -source_origin.y + offset.y + target.min_y, 0, 0, 1};
    const std::array<double, 9> ratio_out{1.0 / target_aspect, 0, 0, 0, 1, 0, 0, 0, 1};
    return mul(ratio_out, mul(tr, mul(rsm, ratio_in)));
}

Vec2 Layout::map_vertex(const IslandPlacement& p, Vec2 original, const PackTarget& target) const {
    const auto t = p.transform(scale, target);
    return {t[0] * original.x + t[1] * original.y + t[2], t[3] * original.x + t[4] * original.y + t[5]};
}

std::size_t snap_to_texels(Layout& layout, const std::vector<Island>& islands, const PackTarget& target,
                           double tex_size, double target_aspect) {
    if (!(tex_size > 0)) return 0;
    std::size_t moved = 0;
    for (auto& p : layout.placements) {
        auto it = std::find_if(islands.begin(), islands.end(), [&](const Island& i) { return i.id == p.id; });
        if (it == islands.end()) continue;
        const auto t = p.transform(layout.scale, target, target_aspect);
        double min_x = 1e300, min_y = 1e300;
        auto visit = [&](const Polygon& poly) {
            for (const auto v : poly.outer) {
                min_x = std::min(min_x, t[0] * v.x + t[1] * v.y + t[2]);
                min_y = std::min(min_y, t[3] * v.x + t[4] * v.y + t[5]);
            }
        };
        visit(it->polygon);
        for (const auto& part : it->extra_parts) visit(part);
        const double sx = std::round(min_x * tex_size) / tex_size, sy = std::round(min_y * tex_size) / tex_size;
        const double dx = sx - min_x, dy = sy - min_y;
        if (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12) continue;
        p.offset.x += dx * target_aspect;
        p.offset.y += dy;
        ++moved;
    }
    return moved;
}

CancellationToken legacy_cancellation_token() { return g_cancel; }
void request_cancel() { g_cancel.request(); }
bool cancel_requested() { return g_cancel.requested(); }

Layout pack_impl(const std::vector<Island>& islands, const PackTarget& target,
                 double scale, const PackOptions& options, double height_factor, SearchStats* stats,
                 const std::vector<Obstacle>& obstacles, double width_factor) {
    Layout layout;
    layout.scale = scale;
    if(options.cancel.requested()){layout.cancelled=true;layout.failure="packing cancelled";return layout;}
    if (islands.empty()) { layout.valid = true; return layout; }
    try {
        const auto t_prep = Clock::now();
        auto set = prepare_all(islands, scale, options);
        const auto t_geometry = Clock::now();
        if (stats) stats->geometry_seconds += std::chrono::duration<double>(t_geometry - t_prep).count();
        const double border = options.border_margin < 0 ? options.margin : options.border_margin;
        // The margin expansion already adds margin/2 around each island, so the frame
        // shrinks by the remaining border - margin/2 on every side.
        const double inset = std::max(0.0, border - 0.5 * options.margin);
        const auto targets = effective_targets(target, options);
        std::vector<TilePlacer::Tile> tiles;
        for (const auto& t : targets) {
            const double base_w = (t.max_x - t.min_x) * options.target_aspect - 2 * inset;
            const double base_h = t.max_y - t.min_y - 2 * inset;
            const double frame_w = base_w * width_factor;
            const double frame_h = base_h * height_factor;
            if (!(frame_w > 0) || !(frame_h > 0)) throw std::invalid_argument("border margin leaves no frame");
            FixedTarget fixed_target{frame_w, frame_h, options.stripe_height};
            if (width_factor > 1.0 || height_factor > 1.0) {
                fixed_target.initial_occupied_width = base_w;
                fixed_target.initial_occupied_height = base_h;
            } else {
                // Fill modes, expressed through the same occupied-canvas seed. The cost
                // charges for growing that canvas, so seeding a dimension makes growth in it
                // free until the frame edge, while the fixed-scale bound keeps feasibility
                // intact. `Auto` seeds nothing and leaves the plain search.
                switch (options.fill) {
                case FillMode::WidthFirst:   // Fill the width, grow upward.
                    fixed_target.initial_occupied_width = base_w;
                    break;
                case FillMode::HeightFirst: // Fill the height, grow rightward.
                    fixed_target.initial_occupied_height = base_h;
                    break;
                case FillMode::Proportional:               // Grow towards the frame's own shape.
                    fixed_target.initial_occupied_width = std::max(0.0, base_w - base_h);
                    fixed_target.initial_occupied_height = std::max(0.0, base_h - base_w);
                    break;
                case FillMode::Free:
                    break;
                }
            }
            fixed_target.fast_search = options.fast_search;
            tiles.push_back({fixed_target});
        }
        std::vector<std::vector<bool>> allowed(islands.size(), std::vector<bool>(tiles.size(), true));
        for (std::size_t i = 0; i < islands.size(); ++i) {
            auto it = options.group_tiles.find(islands[i].group);
            if (it == options.group_tiles.end()) continue;
            std::fill(allowed[i].begin(), allowed[i].end(), false);
            for (const auto t : it->second) if (t < tiles.size()) allowed[i][t] = true;
            if (std::find(allowed[i].begin(), allowed[i].end(), true) == allowed[i].end())
                throw std::invalid_argument("group has no valid tile");
        }
        const double tile_penalty = options.tile_fill == TileFill::Sequential ? 1.0e8 : 0.0;
        // group_adjacent: island index -> group id; empty keeps every helper off.
        GroupInfo groups;
        if (options.group_adjacent)
            for (const auto& island : islands) groups.group_of.push_back(island.group);
        const double group_weight = group_weight_for(options.group_cohesion);
        Input input;
        input.islands = set.scaled;

        const std::size_t trials = std::max<std::size_t>(1, options.trials);
        const std::size_t workers = std::min(trials, thread_count(options));
        struct Worker {
            std::unique_ptr<TilePlacer> placer;
            PreparedProblem prepared;
            TrialPlan plan;
            std::mt19937 rng;
            Evaluation result;
            std::string error;
            std::size_t attempts = 0;
        };
        std::vector<Worker> workers_state(workers);
        // The incumbent's order, resolved once: the CPU workers and the device batch
        // both warm-start from it.
        std::vector<std::size_t> warm;
        if (!options.warm_order.empty()) {
            for (const auto id : options.warm_order) {
                std::size_t index = 0;
                while (index < islands.size() && islands[index].id != id) ++index;
                if (index < islands.size()) warm.push_back(index);
            }
            if (warm.size() != islands.size()) warm.clear();
        }
        for (std::size_t w = 0; w < workers; ++w) {
            auto& st = workers_state[w];
            st.placer = std::make_unique<TilePlacer>(tiles, set.rasters, allowed, tile_penalty,
                                                             options.group_adjacent, group_weight);
            for (std::size_t t = 0; t < targets.size(); ++t)
                for (const auto& obstacle : obstacles)
                    add_obstacle(*st.placer, t, obstacle, targets[t], inset, tiles[t].target.width, tiles[t].target.height, options);
            st.prepared = st.placer->prepare(input);
            st.rng.seed(options.seed * 7919u + static_cast<std::uint32_t>(w) * 104729u + 17u);
            const bool improving = options.seed >= 1000; // Marks improvement rounds.
            const std::size_t warm_workers = warm.empty() ? 0 : std::min<std::size_t>(workers, 5);
            if (w < warm_workers) {
                // Warm start: the incumbent's order, then noisy variants of it.
                st.plan = make_plan(st.prepared, 0, st.rng, groups);
                st.plan.island_order = warm;
                if (w >= 1 && w <= 3) {
                    const std::size_t k = 1 + w * st.plan.island_order.size() / 25;
                    if (groups.on()) grouped_insertion_moves(st.plan.island_order, st.rng, k, groups);
                    else insertion_moves(st.plan.island_order, st.rng, k);
                }
                if (w == 4) random_masks(st.plan, st.rng, 0.3);
            } else {
                const std::size_t strategy = improving ? (w - warm_workers + options.seed) : (w - warm_workers);
                st.plan = make_plan(st.prepared, strategy, st.rng, groups);
                if (improving) {
                    const std::size_t k = 1 + st.plan.island_order.size() / 10;
                    if (w % 3 == 1) {
                        if (groups.on()) grouped_insertion_moves(st.plan.island_order, st.rng, k, groups);
                        else insertion_moves(st.plan.island_order, st.rng, k);
                    }
                    if (w % 3 == 2) random_masks(st.plan, st.rng, 0.35);
                }
            }
            st.plan.allow_partial = options.allow_partial;
        }
        for (auto& st : workers_state) {st.plan.allow_partial = options.allow_partial;st.plan.cancel=options.cancel;}
        const auto t_place = Clock::now();
        if (stats) {
            stats->prepare_seconds += std::chrono::duration<double>(t_place - t_prep).count();
            stats->setup_seconds += std::chrono::duration<double>(t_place - t_geometry).count();
        }

        // --- The GPU portfolio ----------------------------------------------------
        // A batch of whole extra trials runs on the device WHILE the CPU threads run
        // theirs, with the same diversification (strategies, insertion moves, masks,
        // warm order) continued past the CPU workers' indices. A device trial is only
        // consulted when the CPU round found nothing, and only after the exact
        // validator accepts its layout, so the CPU result is never displaced and a
        // GPU run can only turn a failure into a success.
        //
        // Open and strip frames are excluded: their layouts deliberately spill past
        // the target, so `validate_layout` cannot judge them and the rule above would
        // have no teeth.
        // Placer placements -> Layout placements. Shared by the CPU winner and by a
        // device candidate, so both go through exactly the same mapping.
        auto emit = [&](Layout& into, const std::vector<Placement>& placements) {
            for (const auto& placement : placements) {
                std::size_t index = 0;
                while (index < islands.size() && islands[index].id != placement.island_id) ++index;
                const auto& transform = set.transforms.at(index).at(placement.orientation);
                const auto tile = placement.tile;
                // Offsets are expressed relative to the primary target so transform() can add target.min.
                const Vec2 shift{(targets[tile].min_x - target.min_x) * options.target_aspect,
                                 targets[tile].min_y - target.min_y};
                into.placements.push_back({placement.island_id, transform.angle, transform.source_origin,
                    {placement.offset.x + inset + shift.x, placement.offset.y + inset + shift.y},
                    transform.mirrored, island_scale_for(islands[index], scale), tile});
            }
        };

        GpuRound gpu_round;
        if (options.gpu && options.gpu_in_round && options.gpu_trials > 0 &&
            width_factor == 1.0 && height_factor == 1.0) {
            gpu_round.skipped = gpu::unsupported_reason(set.rasters, tiles[0].target, tiles.size(),
                                                        !obstacles.empty(), options.group_adjacent,
                                                        options.allow_partial);
            if (gpu_round.skipped.empty() && !gpu::available())
                gpu_round.skipped = gpu::diagnostic();
            if (gpu_round.skipped.empty()) {
                for (std::size_t g = 0; g < options.gpu_trials; ++g) {
                    // Continue the CPU workers' diversification rather than repeating it.
                    const std::size_t w = workers + g;
                    std::mt19937 rng(options.seed * 7919u + static_cast<std::uint32_t>(w) * 104729u + 17u);
                    TrialPlan plan = make_plan(workers_state[0].prepared,
                                               (options.seed >= 1000 ? w + options.seed : w), rng, groups);
                    if (!warm.empty() && g == 0) plan.island_order = warm;
                    if (g % 3 == 1) insertion_moves(plan.island_order, rng, 1 + plan.island_order.size() / 10);
                    if (g % 3 == 2) random_masks(plan, rng, 0.3);
                    gpu_round.plans.push_back(to_trial_spec(plan, islands.size()));
                    gpu_round.orders.push_back(plan.island_order);
                }
            }
        }
        gpu_round.cancel=options.cancel;
        if(!options.cancel.requested())gpu_round.launch(set.rasters, tiles[0].target);

        std::size_t launched = 0;
        std::size_t total_evaluations = 0;
        const Worker* winner = nullptr;
        std::string last_storage_failure;
        for (std::size_t round = 0; launched < trials && !winner && !options.cancel.requested(); ++round) {
            const std::size_t active = std::min(workers, trials - launched);
            ThreadGroup pool;
            auto run = [&](Worker& st) {
                st.plan.cancel=options.cancel;
                try {
                    st.result = st.placer->pack(st.prepared, {}, st.plan);
                } catch (const std::exception& ex) {
                    st.result = {};
                    st.error = ex.what();
                }
                ++st.attempts;
            };
            for (std::size_t w = 1; w < active; ++w) pool.emplace_back(run, std::ref(workers_state[w]));
            run(workers_state[0]);
            pool.join();
            launched += active;
            total_evaluations += active;
            // group_adjacent: the attempts of a round pack the same islands at the
            // same scale, so among the valid ones keep the most compact rather than the
            // first. Without groups the first valid attempt wins.
            const Worker* most_compact = nullptr;
            double best_measure = std::numeric_limits<double>::infinity();
            for (std::size_t w = 0; w < active; ++w) {
                auto& worker=workers_state[w];
                if (!worker.result.valid) continue;
                if (height_factor==1 && width_factor==1 &&
                    (options.storage_policy || options.output_precision=="float32")) {
                    Layout candidate;candidate.valid=true;candidate.scale=scale;
                    emit(candidate,worker.result.placements);
                    if (!storage_candidate_valid(candidate,islands,target,options,obstacles)) {
                        // A geometric fit is not an incumbent until it survives
                        // the requested storage precision. Exclude it from the
                        // partial fallback too, then diversify the next trial.
                        worker.error=candidate.failure;
                        last_storage_failure=candidate.failure;
                        continue;
                    }
                }
                if (!groups.on()) { winner = &worker; break; }
                const double measure = workers_state[w].placer->group_box_area();
                if (measure < best_measure) { best_measure = measure; most_compact = &workers_state[w]; }
            }
            if (!winner) winner = most_compact;
            if (!winner && options.allow_partial) {
                // Partial mode: keep the attempt that left the fewest islands out.
                const Worker* fewest = nullptr;
                for (std::size_t w = 0; w < active; ++w) {
                    const auto& cand = workers_state[w];
                    if (cand.error.empty() && (!fewest || cand.result.unplaced.size() < fewest->result.unplaced.size())) fewest = &cand;
                }
                if (fewest) { winner = fewest; }
            }
            // The device batch ran alongside this round; collect it once, here, whether
            // or not it is needed, so the wall time it costs is always accounted for.
            if (round == 0 && !gpu_round.plans.empty()) {
                gpu_round.join();
                if (stats) {
                    stats->gpu_dispatches += gpu_round.device_stats.dispatches;
                    stats->gpu_trials += gpu_round.device_stats.trials;
                    stats->gpu_evaluations += gpu_round.device_stats.valid_trials;
                    stats->gpu_upload_seconds += gpu_round.device_stats.upload_seconds;
                    stats->gpu_compute_seconds += gpu_round.device_stats.compute_seconds;
                    stats->gpu_wait_seconds += gpu_round.wait_seconds;
                    if (!gpu_round.device_name.empty()) stats->gpu_device = gpu_round.device_name;
                    if (!gpu_round.skipped.empty()) stats->gpu_skipped = gpu_round.skipped;
                }
                // Extra candidates only: consulted just when the CPU round found
                // nothing, in trial order, and only if the exact validator accepts.
                if (!winner && gpu_round.ran) {
                    for (std::size_t t = 0; t < gpu_round.results.size(); ++t) {
                        const auto& result = gpu_round.results[t];
                        if (!result.valid || result.placements.size() != islands.size()) continue;
                        Layout candidate;
                        candidate.valid = true;
                        candidate.scale = scale;
                        candidate.occupied_width = result.occupied_width;
                        candidate.occupied_height = result.occupied_height;
                        if (t < gpu_round.orders.size())
                            for (const auto index : gpu_round.orders[t]) candidate.order.push_back(islands[index].id);
                        emit(candidate, result.placements);
                        const auto report = validate_layout(islands, candidate, target, options.margin,
                                                            -1, obstacles, options.target_aspect, options.targets,false,border);
                        if (!report.ok || !storage_candidate_valid(candidate,islands,target,options,obstacles)) continue;
                        if (stats) ++stats->gpu_accepted;
                        if (stats) {
                            ++stats->gpu_used;
                            stats->evaluations += total_evaluations;
                            stats->place_seconds += std::chrono::duration<double>(Clock::now() - t_place).count();
                        }
                        return candidate;
                    }
                }
            }
            if (winner) break;
            // Failure repair: bring the island that did not fit to the front, and add
            // fresh noise so repeated rounds explore different orders.
            for (std::size_t w = 0; w < active; ++w) {
                auto& st = workers_state[w];
                if (!st.error.empty()) { st.error.clear(); st.plan = make_plan(st.prepared, w + round * workers + 1, st.rng, groups); st.plan.allow_partial = options.allow_partial; continue; }
                const auto placed = st.result.placements.size();
                if (placed < st.plan.island_order.size()) {
                    const auto stuck = st.plan.island_order[placed];
                    if (groups.on()) move_to_front_grouped(st.plan.island_order, stuck, groups);
                    else move_to_front(st.plan.island_order, stuck);
                }
                if (round % 2 == 1) {
                    if (groups.on()) grouped_insertion_moves(st.plan.island_order, st.rng, 2, groups);
                    else insertion_moves(st.plan.island_order, st.rng, 2);
                }
            }
        }
        if (stats) {
            stats->evaluations += total_evaluations;
            stats->place_seconds += std::chrono::duration<double>(Clock::now() - t_place).count();
        }
        if (!winner) {
            layout.failure = "no valid greedy placement in " + std::to_string(launched) + " attempts";
            if (!last_storage_failure.empty()) layout.failure += ": " + last_storage_failure;
            return layout;
        }
        layout.valid = winner->result.valid;
        layout.unplaced = winner->result.unplaced;
        if (!layout.valid) layout.failure = std::to_string(layout.unplaced.size()) + " islands did not fit";
        layout.occupied_width = winner->placer->tile(0).occupied_width();
        layout.occupied_height = winner->placer->tile(0).occupied_height();
        for (std::size_t t = 1; t < winner->placer->tile_count(); ++t) {
            layout.occupied_width = std::max(layout.occupied_width, winner->placer->tile(t).occupied_width());
            layout.occupied_height = std::max(layout.occupied_height, winner->placer->tile(t).occupied_height());
        }
        for (const auto index : winner->plan.island_order) layout.order.push_back(islands[index].id);
        emit(layout, winner->result.placements);
        if(height_factor==1 && width_factor==1)
            storage_candidate_valid(layout,islands,target,options,obstacles);
    } catch (const std::exception& ex) {
        layout.failure = ex.what();
    }
    return layout;
}

Layout pack_fixed_scale(const std::vector<Island>& islands, const PackTarget& target,
                        double scale, const PackOptions& options, SearchStats* stats,
                        const std::vector<Obstacle>& obstacles) {
    if(options.cancel.requested()){Layout l;l.cancelled=true;l.failure="packing cancelled";return l;}
    const auto prepared=storage_search_options(islands,target,options,obstacles);
    auto layout = pack_impl(islands, target, scale, prepared, 1.0, stats, obstacles);
    finish_layout(layout, islands, target, prepared);
    return finish_result(std::move(layout),islands,target,options,obstacles);
}

Layout pack_strip(const std::vector<Island>& islands, const PackTarget& target, double scale,
                  const PackOptions& options, double height_factor, SearchStats* stats) {
    if(!options.cancel.requested() && options.output_precision!="float64")
        throw std::invalid_argument("storage precision requires a bounded packing entry point");
    auto result=pack_impl(islands,target,scale,options,height_factor,stats,{});
    result.cancelled=options.cancel.requested();return result;
}

Layout pack_open(const std::vector<Island>& islands, const PackTarget& target, double scale,
                 const PackOptions& options, double factor, SearchStats* stats) {
    if(!options.cancel.requested() && options.output_precision!="float64")
        throw std::invalid_argument("storage precision requires a bounded packing entry point");
    auto result=pack_impl(islands,target,scale,options,factor,stats,{},factor);
    result.cancelled=options.cancel.requested();return result;
}

static Layout max_scale_impl(const std::vector<Island>& islands, const PackTarget& target,
                      const PackOptions& options, SearchStats* stats, const std::vector<Obstacle>& obstacles) {
    const auto start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };
    auto out_of_time = [&] { return options.cancel.requested() || (options.time_budget_seconds > 0 && elapsed() > options.time_budget_seconds); };
    SearchStats local;
    SearchStats& st = stats ? *stats : local;
    // Anything but one uniform scale is driven cluster by cluster (see above).
    if (options.scale_policy != ScalePolicy::Shared && !islands.empty()) {
        auto out = pack_clusters(islands, target, options, st, obstacles);
        st.seconds = elapsed();
        return out;
    }
    // Overlaid groups: the whole search, once per group, each over
    // the full target(s) with its own scale. The merged layout's groups overlap by design.
    if (options.overlay_groups) {
        auto merged = pack_overlay_groups(islands, target, options, st, obstacles);
        st.seconds = elapsed();
        return merged;
    }

    double area = 0;
    for (const auto& island : islands) {
        const double s = island.pre_scale * (island.scale_locked ? 1.0 : 1.0);
        area += measure_island(island).area * s * s * options.target_aspect;
    }
    const auto bound_targets = effective_targets(target, options);
    // Texel-density clusters: only the tiles this island set is allowed into
    // count towards the area bound. Off by default, so the bound spans every tile.
    std::vector<bool> reachable(bound_targets.size(), !options.cluster_tile_bound);
    if (options.cluster_tile_bound) {
        for (const auto& island : islands) {
            auto it = options.group_tiles.find(island.group);
            if (it == options.group_tiles.end()) { reachable.assign(bound_targets.size(), true); break; }
            for (const auto t : it->second) if (t < reachable.size()) reachable[t] = true;
        }
    }
    double target_area = 0;
    for (std::size_t t = 0; t < bound_targets.size(); ++t)
        if (reachable[t]) target_area += (bound_targets[t].max_x - bound_targets[t].min_x) * options.target_aspect * (bound_targets[t].max_y - bound_targets[t].min_y);
    Layout best;
    // Realtime solution stream: hand every committed improvement to the caller
    // as it happens. Called only from this thread, only when the incumbent grew.
    double streamed_scale = 0;
    auto stream_best = [&] {
        if (!options.on_improved || !best.valid || !(best.scale > streamed_scale)) return;
        streamed_scale = best.scale;
        // What goes out must look like the final result: the public entry applies the
        // start-corner reflection after the search, so the streamed copy gets it too.
        Layout shown = best;
        finish_layout(shown, islands, target, options);
        options.on_improved(shown);
    };
    if (!(area > 0) || !(target_area > 0)) { best.failure = "empty input or target"; return best; }
    const double upper = std::sqrt(target_area / area);
    st.upper_bound = upper;

    // shrink_spanning: the island set the search packs. It stays a pointer to the
    // caller's vector until the mixed-scale feedback shrinks something, so a run that
    // never shrinks anything never copies.
    const std::vector<Island>* active = &islands;
    std::vector<Island> work;
    std::vector<double> multipliers;
    // The multipliers shrink as the stage goes on, so the incumbent may have come from
    // an earlier, larger set: keep that set with it for the compact stage.
    std::vector<Island> best_islands;

    auto attempt = [&](double scale, const PackOptions& opts) {
        auto layout = pack_impl(*active, target, scale, opts, 1.0, &st, obstacles);
        ++st.scales_tried;
        st.history.emplace_back(scale, layout.valid);
        if (layout.valid && (!best.valid || scale > best.scale)) {
            best = layout;
            if (options.shrink_spanning) best_islands = *active;
            stream_best();
        }
        return layout.valid;
    };

    // The device portfolio. It runs whole extra trials of its own, at its own
    // scale, from here to the end of the search - alongside the bracket rounds and
    // alongside the improvement rounds. The CPU never waits for it: a batch is
    // collected on a round boundary once it has finished, its layouts go through the
    // exact validator, and one replaces the incumbent only if it validates AND its
    // scale is larger. So the CPU search is bit for bit the search it would have run
    // alone, and the device can only add scale.
    //
    // Only with a time budget. A device batch takes seconds where a CPU round takes
    // milliseconds, so in an unbudgeted run - which returns as soon as the bracket
    // closes - waiting for one at the end would cost far more time than it can win.
    // Not under shrink_spanning either: the device packs and validates the caller's
    // island set, while the feedback shrinks a working copy the incumbent then refers to.
    const bool use_device = options.gpu && options.gpu_trials > 0 && options.time_budget_seconds > 0 &&
                            obstacles.empty() && !options.group_adjacent && !options.allow_partial &&
                            !options.shrink_spanning &&
                            effective_targets(target, options).size() == 1 && gpu::available();
    PackOptions device_options = options;
    device_options.gpu_in_round = false;
    GpuImprover improver(islands, target, device_options);
    std::uint32_t device_seed = 500u + options.seed * 17u;
    auto time_left = [&] { return options.time_budget_seconds - elapsed(); };
    auto launch_device = [&](double scale, double reach) {
        if (!use_device || improver.busy() || !improver.skipped().empty()) return;
        const double left = time_left();
        if (!(scale > 0) || left < improver.expected_seconds()) return;
        improver.start(scale, reach, device_seed, best.order, options.gpu_trials, left);
        device_seed += 7919u;
    };
    // Take a finished batch. `wait` joins one still running; only the very end does that.
    auto collect_device = [&](bool wait) {
        if (!improver.busy() || (!wait && !improver.ready())) return;
        auto candidates = improver.collect(st);
        // Largest rung first. A candidate at or below the incumbent cannot help, and
        // neither can anything after it, so the scan stops there.
        for (auto& candidate : candidates) {
            if (!(candidate.scale > best.scale)) break;
            const auto report = validate_layout(islands, candidate, target, options.margin,
                                                -1, obstacles, options.target_aspect, options.targets,false,
                                                options.border_margin<0?options.margin:options.border_margin);
            ++st.scales_tried;
            const bool valid=report.ok && storage_candidate_valid(candidate,islands,target,options,obstacles);
            st.history.emplace_back(candidate.scale, valid);
            if (!valid) continue;
            ++st.gpu_accepted;
            ++st.gpu_used;
            if (options.progress)
                std::fprintf(stderr, "[uvpack] device batch: scale %.5f accepted, was %.5f (%.1fs)\n",
                             candidate.scale, best.scale, elapsed());
            best = std::move(candidate);
            stream_best();
            break;
        }
    };

    PackOptions quick = options;
    quick.trials = std::min<std::size_t>(options.trials, thread_count(options));
    if (options.parallel_bracket && thread_count(options) > 1) {
        // Bracket stage: k-section. Each round evaluates `k` scales in parallel, one
        // greedy trial each with a different ordering strategy, then narrows the
        // bracket to (largest valid, smallest invalid above it). ~3 bits per round.
        const std::size_t k = thread_count(options);
        double low = 0, high = upper;
        std::size_t rounds = 0;
        // Extent estimate: one open
        // pack at the area bound grows a compact layout whose extent E says how far the
        // scale is off; s/E is a close estimate, so the bracket starts one round away.
        double estimate = 0;
        if ((options.extent_estimate || options.single_pass) && obstacles.empty() && !out_of_time()) {
            const auto probe = open_probe(islands, target, options, k, upper, st, obstacles);
            estimate = probe.estimate;
            // Single pass: the rescaled open layout is the answer, already validated.
            if (probe.accepted) {
                best = probe.layout;
                stream_best();
                st.best_scale = best.scale;
                st.seconds = elapsed();
                if (options.time_budget_seconds <= 0) { finish_layout(best, islands, target, options); return best; }
                low = best.scale; high = std::min(upper, best.scale * 1.02); // Continue into the refine stage with a tight bracket.
            }
        }
        auto evaluate_round = [&](const std::vector<double>& scales) {
            const auto t_round = Clock::now();
            std::vector<Layout> layouts(scales.size());
            std::vector<SearchStats> stats_per(scales.size());
            ThreadGroup pool;
            for (std::size_t i = 0; i < scales.size(); ++i) {
                pool.emplace_back([&, i] {
                    PackOptions single = options;
                    single.trials = 1;
                    single.threads = 1;
                    single.seed = options.seed + static_cast<std::uint32_t>(i) * 7919u + static_cast<std::uint32_t>(rounds) * 104729u;
                    single.warm_order.clear();
                    single.gpu_in_round = false; // The device is driven from the refine stage, not here.
                    layouts[i] = pack_impl(islands, target, scales[i], single, 1.0, &stats_per[i], obstacles);
                });
            }
            pool.join();
            for (std::size_t i = 0; i < scales.size(); ++i) {
                merge_gpu_stats(st, stats_per[i]);
                st.evaluations += stats_per[i].evaluations;
                st.prepare_seconds = std::max(st.prepare_seconds, stats_per[i].prepare_seconds);
                st.geometry_seconds = std::max(st.geometry_seconds, stats_per[i].geometry_seconds);
                st.setup_seconds = std::max(st.setup_seconds, stats_per[i].setup_seconds);
                st.place_seconds = std::max(st.place_seconds, stats_per[i].place_seconds);
                ++st.scales_tried;
                st.history.emplace_back(scales[i], layouts[i].valid);
                if (layouts[i].valid) {
                    if (!best.valid || scales[i] > best.scale) best = layouts[i];
                    low = std::max(low, scales[i]);
                }
            }
            stream_best();
            for (std::size_t i = 0; i < scales.size(); ++i)
                if (!layouts[i].valid && scales[i] > low && scales[i] < high) high = scales[i];
            if (options.progress) {
                double slowest = 0, slowest_prep = 0;
                double slowest_geom = 0, slowest_setup = 0;
                for (const auto& sp : stats_per) { slowest = std::max(slowest, sp.prepare_seconds + sp.place_seconds); slowest_prep = std::max(slowest_prep, sp.prepare_seconds); slowest_geom = std::max(slowest_geom, sp.geometry_seconds); slowest_setup = std::max(slowest_setup, sp.setup_seconds); }
                std::fprintf(stderr, "[uvpack] bracket round %zu: wall %.3fs, slowest worker %.3fs (prep %.3fs = geometry %.3fs + setup %.3fs), bracket [%.4f, %.4f]\n",
                             rounds + 1, std::chrono::duration<double>(Clock::now() - t_round).count(), slowest, slowest_prep, slowest_geom, slowest_setup, low, high);
            }
            ++rounds;
        };
        // Initial round: tight spread around the extent estimate when there is one
        // (bounded packs are a little denser than open ones, so reach slightly above
        // it), otherwise a geometric spread from the area bound downward.
        {
            std::vector<double> scales;
            if (estimate > 0) {
                const double lo = estimate * 0.97, hi = std::min(upper, estimate * 1.005);
                for (std::size_t i = 0; i < k; ++i) scales.push_back(lo + (hi - lo) * double(i) / double(std::max<std::size_t>(1, k - 1)));
                high = std::min(high, upper);
            } else {
                const double lo_frac = std::min(options.bracket_start, options.initial_fraction * 0.99);
                for (std::size_t i = 0; i < k; ++i)
                    scales.push_back(upper * options.initial_fraction * std::pow(lo_frac / options.initial_fraction, double(i) / double(std::max<std::size_t>(1, k - 1))));
            }
            evaluate_round(scales);
        }
        std::size_t halvings = 0;
        while (!best.valid && halvings < options.max_halvings && !out_of_time()) {
            std::vector<double> scales;
            for (std::size_t i = 0; i < k; ++i) scales.push_back(high * std::pow(0.5, double(i + 1) / double(k)));
            evaluate_round(scales);
            ++halvings;
        }
        if (!best.valid) {
            best.failure = "no feasible scale found down to " + std::to_string(high);
            st.seconds = elapsed();
            return best;
        }
        const double tolerance = estimate > 0 ? std::max(options.bracket_tolerance, options.estimate_tolerance) : options.bracket_tolerance;
        while (high / low > 1.0 + tolerance && rounds < options.bracket_rounds + 4 && !out_of_time()) {
            std::vector<double> scales;
            for (std::size_t i = 0; i < k; ++i) scales.push_back(low + (high - low) * double(i + 1) / double(k + 1));
            evaluate_round(scales);
        }
    } else {
        // Bracket stage: one parallel round per scale, then bisect.
        double high = upper;
        double low = 0;
        double probe = upper * options.initial_fraction;
        bool feasible = false;
        for (std::size_t i = 0; i < options.max_halvings && !out_of_time(); ++i) {
            if (attempt(probe, quick)) { feasible = true; low = probe; break; }
            high = probe;
            probe *= 0.5;
        }
        if (!feasible) {
            best.valid = false;
            best.failure = "no feasible scale found down to " + std::to_string(probe);
            st.seconds = elapsed();
            return best;
        }
        for (std::size_t i = 0; i < options.bracket_rounds && !out_of_time(); ++i) {
            const double mid = 0.5 * (low + high);
            if (attempt(mid, quick)) low = mid; else high = mid;
        }
    }

    // Mixed scales. After a valid
    // improvement round, islands whose extent nearly spans the frame get their
    // per-island scale cut by 0.9 for the following rounds, so one dominant island
    // stops holding the uniform scale of every other one down. Only the uniform scale
    // decides whether a round is kept.
    const auto mixed_targets = effective_targets(target, options);
    auto mixed_feedback = [&](const Layout& layout) {
        if (!options.shrink_spanning || layout.placements.empty()) return;
        if (multipliers.empty()) multipliers.assign(islands.size(), 1.0);
        const double border = options.border_margin < 0 ? options.margin : options.border_margin;
        const double inset = std::max(0.0, border - 0.5 * options.margin);
        std::vector<bool> near_spanning(islands.size(), false);
        for (const auto& p : layout.placements) {
            std::size_t index = 0;
            while (index < islands.size() && islands[index].id != p.id) ++index;
            if (index >= islands.size()) continue;
            const auto t = p.transform(layout.scale, target, options.target_aspect);
            double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
            auto visit = [&](const Polygon& poly) {
                for (const auto v : poly.outer) {
                    // Back to packing space: transform() divides x by the target aspect.
                    const double x = (t[0] * v.x + t[1] * v.y + t[2]) * options.target_aspect;
                    const double y = t[3] * v.x + t[4] * v.y + t[5];
                    min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                    min_y = std::min(min_y, y); max_y = std::max(max_y, y);
                }
            };
            visit(islands[index].polygon);
            for (const auto& part : islands[index].extra_parts) visit(part);
            if (!(max_x >= min_x)) continue;
            const auto& box = mixed_targets[std::min(p.tile, mixed_targets.size() - 1)];
            const double frame_w = (box.max_x - box.min_x) * options.target_aspect - 2 * inset;
            const double frame_h = (box.max_y - box.min_y) - 2 * inset;
            // The placed extent includes the margin expansion (margin/2 per side).
            near_spanning[index] = nearly_spans_target(max_x - min_x + options.margin,
                                                      max_y - min_y + options.margin, frame_w, frame_h);
        }
        // Only act when SOME island spans and some does not: that is the case where one
        // island alone is holding the uniform scale down. Each such island loses a
        // sixth of its scale per round, with a floor so nothing can vanish; a sixth
        // converges inside a handful of rounds without overshooting past the point
        // where the island stops being the one that binds.
        if (std::find(near_spanning.begin(), near_spanning.end(), false) == near_spanning.end()) return;
        const auto before = multipliers;
        for (std::size_t i = 0; i < multipliers.size(); ++i)
            if (near_spanning[i]) multipliers[i] = std::max(0.01, multipliers[i] * (5.0 / 6.0));
        if (multipliers == before) return;
        work = islands;
        std::size_t shrunk = 0;
        for (std::size_t i = 0; i < work.size(); ++i) {
            work[i].pre_scale = islands[i].pre_scale * multipliers[i];
            if (multipliers[i] < 1.0) ++shrunk;
        }
        active = &work;
        st.shrunk_islands = shrunk;
        if (options.progress) std::fprintf(stderr, "[uvpack] mixed scales: %zu island(s) shrunk\n", shrunk);
    };

    // Refine stage: budgeted improvement with adaptive relative steps and full trial rounds.
    if (options.time_budget_seconds > 0) {
        double step = options.initial_step;
        unsigned successes = 0;
        // Stall timeout: give up once the incumbent has stood
        // still for that long, instead of burning the rest of the budget on it.
        auto last_improvement = Clock::now();
        double stagnant_from = best.scale;
        mixed_feedback(best);
        // How far the device's own bracket reaches above the incumbent. While a batch
        // runs, the CPU rounds keep raising the incumbent, so a batch aimed at the
        // incumbent as it stands when the batch is launched lands already beaten. The
        // rungs span [incumbent, incumbent * (1 + reach)], wide enough to cover what the
        // CPU can still gain in the remaining budget and no wider.
        auto device_reach = [&] {
            const double ahead = std::min(1.0, std::max(0.0, time_left() / options.time_budget_seconds));
            return std::min(0.08, options.initial_step * (1.0 + 2.5 * ahead));
        };
        launch_device(best.scale, device_reach());
        while (!out_of_time()) {
            if (options.stall_timeout > 0 &&
                std::chrono::duration<double>(Clock::now() - last_improvement).count() > options.stall_timeout) {
                if (options.progress)
                    std::fprintf(stderr, "[uvpack] no improvement for %.1fs: stopping at scale %.5f\n",
                                 options.stall_timeout, best.scale);
                break;
            }
            const double candidate = best.scale * (1.0 + step);
            ++st.improve_rounds;
            PackOptions round_options = options;
            round_options.seed = 1000u + options.seed * 131u + static_cast<std::uint32_t>(st.improve_rounds) * 7u;
            round_options.trials = std::max<std::size_t>(options.trials, 2 * thread_count(options)); // Allow one repair round.
            round_options.warm_order = best.order;
            round_options.gpu_in_round = false;
            if (attempt(candidate, round_options)) {
                if (options.progress) std::fprintf(stderr, "[uvpack] round %zu: scale %.5f valid (%.1fs)\n", st.improve_rounds, best.scale, elapsed());
                mixed_feedback(best);
                if (++successes >= 2) { step = std::min(step * 2.0, 0.25); successes = 0; }
            } else {
                successes = 0;
                step *= 0.5;
                if (step < options.min_step) {
                    // Re-arm at a moderate step so later rounds keep exploring with new noise.
                    step = options.initial_step * 0.5;
                }
            }
            collect_device(false);
            launch_device(best.scale, device_reach());
            if (best.scale > stagnant_from) { stagnant_from = best.scale; last_improvement = Clock::now(); }
        }
    }
    // The search is over. A batch still in flight is joined here - it was only started
    // when the budget had room for it - and its layouts get the same validation.
    collect_device(true);
    // Compact stage: overlap-tolerant compaction. Grows the incumbent in place and
    // repairs the overlaps with a guided local search; only exactly-validated
    // layouts are accepted, so this can only improve the result.
    if (options.compaction_time > 0 && best.valid && !options.cancel.requested()) {
        CompactionStats compaction;
        const auto& compact_islands = best_islands.empty() ? islands : best_islands;
        PackOptions compaction_options = options;
        if (options.on_improved)
            compaction_options.on_improved = [&](const Layout& accepted) {
                Layout shown = accepted;
                finish_layout(shown, islands, target, options);
                options.on_improved(shown);
            };
        best = compact_layout(compact_islands, target, best, compaction_options, obstacles, &compaction);
        st.compaction_attempts = compaction.attempts;
        st.compaction_accepted = compaction.accepted;
        st.compaction_seconds = compaction.seconds;
        st.compaction_rotations = compaction.rotations_applied;
        st.compaction_threads = compaction.threads;
        st.compaction_rounds = compaction.rounds;
        st.compaction_start_scale = compaction.start_scale;
        st.compaction_skipped = compaction.skipped;
    }
    finish_layout(best, islands, target, options);
    st.best_scale = best.scale;
    st.seconds = elapsed();
    return best;
}

static Layout max_margin_impl(const std::vector<Island>& islands, const PackTarget& target, double scale,
                       const PackOptions& options, double& margin, SearchStats* stats,
                       const std::vector<Obstacle>& obstacles) {
    const auto start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };
    auto out_of_time = [&] { return options.cancel.requested() || (options.time_budget_seconds > 0 && elapsed() > options.time_budget_seconds); };
    SearchStats local;
    SearchStats& st = stats ? *stats : local;
    Layout best;
    double best_margin = -1;
    auto attempt = [&](double m) {
        PackOptions opts = options;
        opts.margin = m;
        auto layout = pack_impl(islands, target, scale, opts, 1.0, &st, obstacles);
        ++st.scales_tried;
        st.history.emplace_back(m, layout.valid);
        if (layout.valid && m > best_margin) { best = layout; best_margin = m; }
        return layout.valid;
    };
    // Bracket: start at the requested margin (or a small one) and double until it fails.
    double low = std::max(options.margin, 0.0);
    if (!attempt(low)) {
        best.valid = false;
        best.failure = "islands do not fit at the fixed scale even with margin " + std::to_string(low);
        margin = low;
        st.seconds = elapsed();
        return best;
    }
    double high = std::max(low * 2.0, 0.005);
    std::size_t doublings = 0;
    while (attempt(high) && doublings < 12 && !out_of_time()) { low = high; high *= 2.0; ++doublings; }
    for (std::size_t i = 0; i < options.bracket_rounds && !out_of_time(); ++i) {
        const double mid = 0.5 * (low + high);
        if (attempt(mid)) low = mid; else high = mid;
    }
    margin = best_margin;
    PackOptions achieved = options;
    achieved.margin = best_margin;
    finish_layout(best, islands, target, achieved);
    st.best_scale = scale;
    st.seconds = elapsed();
    return best;
}

Layout pack_max_scale(const std::vector<Island>& islands,const PackTarget& target,
                      const PackOptions& options,SearchStats* stats,const std::vector<Obstacle>& obstacles) {
    if(options.cancel.requested()){Layout l;l.cancelled=true;l.failure="packing cancelled";return l;}
    const auto prepared=storage_search_options(islands,target,options,obstacles);
    return finish_result(max_scale_impl(islands,target,prepared,stats,obstacles),islands,target,options,obstacles);
}
Layout pack_max_margin(const std::vector<Island>& islands,const PackTarget& target,double scale,
                       const PackOptions& options,double& margin,SearchStats* stats,const std::vector<Obstacle>& obstacles) {
    if(options.cancel.requested()){Layout l;l.cancelled=true;l.failure="packing cancelled";margin=options.margin;return l;}
    const auto prepared=storage_search_options(islands,target,options,obstacles);
    const auto history_start=stats?stats->history.size():0;
    auto result=max_margin_impl(islands,target,scale,prepared,margin,stats,obstacles);
    // A nested call receives already-reserved options; only the outer public call
    // translates achieved margins back into the caller's original units/policy.
    if(!options.storage_policy && prepared.storage_policy) {
        margin-=prepared.storage_policy->gap_reserve;
        if(stats)for(std::size_t i=history_start;i<stats->history.size();++i)
            stats->history[i].first-=prepared.storage_policy->gap_reserve;
    }
    auto achieved=options;achieved.margin=margin;
    return finish_result(std::move(result),islands,target,achieved,obstacles);
}
} // namespace uvpack
