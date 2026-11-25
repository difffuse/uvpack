#include "thread_group.hpp"
#include "uvpack/compaction.hpp"
#include "cancellation_internal.hpp"

#include "uvpack/geometry.hpp"
#include "uvpack/bounding.hpp"
#include "uvpack/offset.hpp"
#include "uvpack/validator.hpp"
#include "clipper.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <thread>

namespace uvpack {
namespace {

using Clock = std::chrono::steady_clock;
using ClipperLib::cInt;

// The same integer grid offset.cpp uses for offsetting (2^24). Packing-space
// coordinates are of order 1, so the grid resolves ~6e-8 and stays far inside
// Clipper's fast range; areas stay far inside a double's exact integer range.
constexpr double kUnits = 16777216.0;

struct DBox {
    double min_x = 1e300, min_y = 1e300, max_x = -1e300, max_y = -1e300;
    void add(double x, double y) {
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    }
};

struct IBox { cInt min_x = 0, min_y = 0, max_x = 0, max_y = 0; };

inline bool boxes_overlap(const IBox& a, const IBox& b) {
    return a.max_x > b.min_x && b.max_x > a.min_x && a.max_y > b.min_y && b.max_y > a.min_y;
}

using Quad = std::array<Vec2, 4>;

// Minimum-area enclosing box of a ring, by rotating calipers over its hull edges.
// Used as a tight cheap prefilter: an axis-aligned box around a long island rotated
// 45 degrees is mostly empty, and in a dense pack that costs a polygon clip per
// neighbour for nothing. Falls back to the axis-aligned box on degenerate input.
Quad minimum_area_box(const Ring& ring) {
    Ring hull;
    try {
        hull = convex_boundary(ring);
    } catch (const std::exception&) {
        hull.clear();
    }
    if (hull.size() > 1 && hull.front().x == hull.back().x && hull.front().y == hull.back().y) hull.pop_back();
    DBox aabb;
    for (const auto p : ring) aabb.add(p.x, p.y);
    Quad best{Vec2{aabb.min_x, aabb.min_y}, Vec2{aabb.max_x, aabb.min_y},
              Vec2{aabb.max_x, aabb.max_y}, Vec2{aabb.min_x, aabb.max_y}};
    if (hull.size() < 3) return best;
    double best_area = (aabb.max_x - aabb.min_x) * (aabb.max_y - aabb.min_y);
    const std::size_t n = hull.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = hull[i], b = hull[(i + 1) % n];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len = std::hypot(dx, dy);
        if (!(len > 0)) continue;
        const double ux = dx / len, uy = dy / len; // Box axis along this edge.
        double min_u = 1e300, max_u = -1e300, min_v = 1e300, max_v = -1e300;
        for (const auto p : hull) {
            const double u = p.x * ux + p.y * uy;
            const double v = -p.x * uy + p.y * ux;
            min_u = std::min(min_u, u); max_u = std::max(max_u, u);
            min_v = std::min(min_v, v); max_v = std::max(max_v, v);
        }
        const double area = (max_u - min_u) * (max_v - min_v);
        if (!(area < best_area)) continue;
        best_area = area;
        auto corner = [&](double u, double v) { return Vec2{u * ux - v * uy, u * uy + v * ux}; };
        best = {corner(min_u, min_v), corner(max_u, min_v), corner(max_u, max_v), corner(min_u, max_v)};
    }
    return best;
}

// Separating-axis test on the four axes of two boxes. True when they are disjoint.
inline bool quads_separated(const Quad& a, const Quad& b) {
    for (int s = 0; s < 2; ++s) {
        const Quad& p = s == 0 ? a : b;
        for (int e = 0; e < 2; ++e) {
            const double nx = -(p[e + 1].y - p[e].y), ny = p[e + 1].x - p[e].x;
            double a_lo = 1e300, a_hi = -1e300, b_lo = 1e300, b_hi = -1e300;
            for (int k = 0; k < 4; ++k) {
                const double pa = a[k].x * nx + a[k].y * ny;
                a_lo = std::min(a_lo, pa); a_hi = std::max(a_hi, pa);
                const double pb = b[k].x * nx + b[k].y * ny;
                b_lo = std::min(b_lo, pb); b_hi = std::max(b_hi, pb);
            }
            if (a_hi <= b_lo || b_hi <= a_lo) return true;
        }
    }
    return false;
}

// Overlaps below this (in packing-space area units) count as zero. Clipper's
// integer arithmetic returns an exact zero for disjoint polygons; the convex fast
// path below works in doubles, where two polygons that just touch can produce a
// denormal sliver. A 1e-12 area is a penetration of picometres against a margin of
// millimetres, and every accepted layout still has to pass the exact validator.
constexpr double kAreaEpsilon = 1e-12;

// Signed area, positive for counter-clockwise.
double ring_signed_area(const std::vector<Vec2>& ring) {
    double sum = 0;
    for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
        const Vec2 a = ring[i], b = ring[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    return 0.5 * sum;
}

bool ring_is_convex(const std::vector<Vec2>& ring) {
    const std::size_t n = ring.size();
    if (n < 3) return false;
    int sign = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = ring[i], b = ring[(i + 1) % n], c = ring[(i + 2) % n];
        const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (cross > 0) { if (sign < 0) return false; sign = 1; }
        else if (cross < 0) { if (sign > 0) return false; sign = -1; }
    }
    return sign != 0;
}

// One island (or one static obstacle) in the compaction's working representation:
// margin-expanded rings in a local frame whose origin is the shape's own pivot,
// plus the current pose and the integer paths cached for it.
struct Shape {
    std::uint64_t id = 0;
    bool fixed = false;             // Obstacles never move.
    bool mirrored = false;
    double island_scale = 1;
    Vec2 pivot{0, 0};               // Pivot expressed in V space (see to_layout).
    std::vector<std::vector<Ring>> parts; // Local, angle 0: outer ring then holes, per part.
    double angle = 0;
    Vec2 pos{0, 0};                 // World position of the local origin.
    std::vector<Quad> obb;          // Per part, minimum-area box in local coordinates.
    std::vector<char> convex;       // Per part: hole-free and convex, so the fast path applies.
    std::vector<std::vector<Vec2>> convex_local; // Per convex part, CCW, local coordinates.
    std::vector<ClipperLib::Paths> rotated; // Integer, pivot at origin, for `angle`.
    std::vector<Quad> obb_rotated;  // Per part, rotated to `angle`.
    std::vector<std::vector<Vec2>> convex_rotated;
    DBox local_box;                 // Bounds of the rotated local geometry.
    std::vector<ClipperLib::Paths> world;   // Integer, for the current pose.
    std::vector<Quad> obb_world;    // Per part, at the current pose.
    std::vector<std::vector<Vec2>> convex_world;
    std::vector<IBox> part_box;
    IBox box{};
    int effort = 0;                 // How hard the last relocations had to work.
};

struct Pose { double angle = 0; Vec2 pos{0, 0}; };

double island_scale_for(const Island& island, double scale) {
    return island.pre_scale * (island.scale_locked ? 1.0 : scale);
}

Ring open_ring(const Ring& ring) {
    Ring out(ring.begin(), ring.end());
    if (out.size() > 1 && out.front().x == out.back().x && out.front().y == out.back().y) out.pop_back();
    return out;
}

class Compactor {
public:
    Compactor(const std::vector<Island>& islands, const PackTarget& target,
              const PackOptions& options, const std::vector<Obstacle>& obstacles)
        : islands_(islands), target_(target), options_(options), obstacles_(obstacles),
          ratio_(options.target_aspect) {
        const double border = options_.border_margin < 0 ? options_.margin : options_.border_margin;
        const double inset = std::max(0.0, border - 0.5 * options_.margin);
        frame_.min_x = target_.min_x * ratio_ + inset;
        frame_.max_x = target_.max_x * ratio_ - inset;
        frame_.min_y = target_.min_y + inset;
        frame_.max_y = target_.max_y - inset;
    }

    // Single-threaded driver (the pre-parallel implementation, kept selectable).
    Layout run(const Layout& start, double seconds, CompactionStats& stats);

    // ---- portfolio interface ---------------------------------------------------
    // One Compactor per thread; nothing is shared between them, Clipper instances
    // included. The driver below keeps them on the same incumbent by copying the
    // winner's committed poses into the others (`adopt`), which is one geometry
    // rebuild, the same cost an attempt pays anyway.

    // Build the working representation of `start`. False (with `stats.skipped` set)
    // when the stage declines the problem.
    bool load(const Layout& start, CompactionStats& stats);
    // Repair an overlapping input and run the optional zero-growth polish. True when
    // the incumbent changed, in which case `best` carries the new one.
    bool prepare_incumbent(double seconds, const Clock::time_point& t0, std::mt19937& rng, Layout& best);
    // One growth attempt from the committed incumbent. On success `out` is the
    // validated layout and the shapes are left at its poses, so commit() adopts it;
    // on failure the shapes are rolled back to the incumbent.
    bool try_scale(double candidate_scale, double deadline_seconds, const Clock::time_point& t0,
                   std::mt19937& rng, bool rotation, Layout& out);
    // Take over another attempt's committed incumbent.
    bool adopt(const std::vector<Pose>& poses, double scale, Vec2 center);
    void commit_current(double scale) { commit(scale); }
    const std::vector<Pose>& accepted() const { return accepted_; }
    // Realtime solution stream: every accept the single-threaded driver and the
    // portfolio preamble make is handed to the caller straight away. Both run on the
    // thread that called compact_layout, never on a portfolio worker, so no locking.
    void stream(const Layout& layout) const { if (options_.on_improved) options_.on_improved(layout); }

    double accepted_scale() const { return accepted_scale_; }
    Vec2 accepted_center() const { return accepted_center_; }
    std::size_t movable() const { return movable_; }
    double initial_delta() const;
    void report(const CompactionStats& stats) const;
    void set_stats(CompactionStats* stats) { stats_ = stats; }
    // Cooperative cancel: an attempt whose rank is not above the best rank that has
    // already separated this round is dominated (its scale would be smaller), so it
    // stops instead of burning the rest of the slice.
    void set_rank(const std::atomic<int>* best_rank, int rank) { best_rank_ = best_rank; rank_ = rank; }

private:
    bool aborted() const {
        return best_rank_ && best_rank_->load(std::memory_order_relaxed) >= rank_;
    }
    // ---- geometry construction -------------------------------------------------
    // Margin-expanded rings of one island at `scale`, in V space (mirror, uniform
    // scale and target aspect applied), recentred on their bounding-box centre.
    bool build_shape(const Island& island, bool mirrored, double scale, Shape& out) const {
        std::vector<Polygon> polys;
        const double s = island_scale_for(island, scale);
        auto scale_polygon = [&](Polygon poly) {
            auto map = [&](Vec2 p) { return Vec2{p.x * ratio_ * s, p.y * s}; };
            for (auto& p : poly.outer) p = map(p);
            for (auto& hole : poly.holes) for (auto& p : hole) p = map(p);
            return poly;
        };
        try {
            polys.push_back(expand_packing_margin(scale_polygon(island.polygon), options_.margin));
            for (const auto& part : island.extra_parts)
                polys.push_back(expand_packing_margin(scale_polygon(part), options_.margin));
        } catch (const std::exception&) {
            return false;
        }
        DBox bounds;
        out.parts.clear();
        for (auto& poly : polys) {
            std::vector<Ring> rings;
            rings.push_back(open_ring(poly.outer));
            for (const auto& hole : poly.holes) rings.push_back(open_ring(hole));
            if (mirrored) {
                for (auto& ring : rings) {
                    for (auto& p : ring) p.x = -p.x;
                    std::reverse(ring.begin(), ring.end());
                }
            }
            for (const auto& ring : rings) for (const auto p : ring) bounds.add(p.x, p.y);
            out.parts.push_back(std::move(rings));
        }
        if (out.parts.empty() || bounds.max_x < bounds.min_x) return false;
        out.pivot = {0.5 * (bounds.min_x + bounds.max_x), 0.5 * (bounds.min_y + bounds.max_y)};
        for (auto& part : out.parts) for (auto& ring : part) for (auto& p : ring) {
            p.x -= out.pivot.x; p.y -= out.pivot.y;
        }
        set_prefilters(out);
        out.id = island.id;
        out.mirrored = mirrored;
        out.island_scale = s;
        out.fixed = false;
        return true;
    }

    bool build_obstacle(const Obstacle& obstacle, Shape& out) const {
        Polygon poly = obstacle.polygon;
        for (auto& p : poly.outer) p.x *= ratio_;
        for (auto& hole : poly.holes) for (auto& p : hole) p.x *= ratio_;
        try {
            poly = expand_packing_margin(poly, options_.margin + 2.0 * obstacle.extra_margin);
        } catch (const std::exception&) {
            return false;
        }
        std::vector<Ring> rings;
        rings.push_back(open_ring(poly.outer));
        for (const auto& hole : poly.holes) rings.push_back(open_ring(hole));
        out.parts.assign(1, std::move(rings));
        set_prefilters(out);
        out.pivot = {0, 0};
        out.fixed = true;
        out.island_scale = 1;
        out.id = std::numeric_limits<std::uint64_t>::max();
        return true;
    }

    // Per-part prefilters: the minimum-area box, and a counter-clockwise copy of the
    // outer ring for the parts a convex clip can handle exactly.
    static void set_prefilters(Shape& out) {
        out.obb.clear();
        out.convex.clear();
        out.convex_local.clear();
        for (const auto& part : out.parts) {
            out.obb.push_back(minimum_area_box(part.front()));
            std::vector<Vec2> ring(part.front().begin(), part.front().end());
            const bool convex = part.size() == 1 && ring_is_convex(ring);
            if (convex && ring_signed_area(ring) < 0) std::reverse(ring.begin(), ring.end());
            out.convex.push_back(convex ? 1 : 0);
            out.convex_local.push_back(convex ? std::move(ring) : std::vector<Vec2>{});
        }
    }

    void set_angle(Shape& s, double angle) {
        s.angle = angle;
        const double c = std::cos(angle), sn = std::sin(angle);
        s.local_box = DBox{};
        s.rotated.resize(s.parts.size());
        s.obb_rotated.resize(s.obb.size());
        for (std::size_t p = 0; p < s.obb.size(); ++p)
            for (int k = 0; k < 4; ++k)
                s.obb_rotated[p][k] = {c * s.obb[p][k].x - sn * s.obb[p][k].y,
                                       sn * s.obb[p][k].x + c * s.obb[p][k].y};
        s.convex_rotated.resize(s.convex_local.size());
        for (std::size_t p = 0; p < s.convex_local.size(); ++p) {
            s.convex_rotated[p].resize(s.convex_local[p].size());
            for (std::size_t k = 0; k < s.convex_local[p].size(); ++k)
                s.convex_rotated[p][k] = {c * s.convex_local[p][k].x - sn * s.convex_local[p][k].y,
                                          sn * s.convex_local[p][k].x + c * s.convex_local[p][k].y};
        }
        for (std::size_t p = 0; p < s.parts.size(); ++p) {
            s.rotated[p].resize(s.parts[p].size());
            for (std::size_t r = 0; r < s.parts[p].size(); ++r) {
                const auto& ring = s.parts[p][r];
                auto& path = s.rotated[p][r];
                path.resize(ring.size());
                for (std::size_t i = 0; i < ring.size(); ++i) {
                    const double x = c * ring[i].x - sn * ring[i].y;
                    const double y = sn * ring[i].x + c * ring[i].y;
                    s.local_box.add(x, y);
                    path[i] = ClipperLib::IntPoint(static_cast<cInt>(std::llround(x * kUnits)),
                                                  static_cast<cInt>(std::llround(y * kUnits)));
                }
            }
        }
    }

    void set_pos(Shape& s, Vec2 p) {
        s.pos = p;
        const cInt ox = static_cast<cInt>(std::llround(p.x * kUnits));
        const cInt oy = static_cast<cInt>(std::llround(p.y * kUnits));
        s.world.resize(s.rotated.size());
        s.part_box.resize(s.rotated.size());
        s.obb_world.resize(s.obb_rotated.size());
        for (std::size_t i = 0; i < s.obb_rotated.size(); ++i)
            for (int k = 0; k < 4; ++k)
                s.obb_world[i][k] = {s.obb_rotated[i][k].x + p.x, s.obb_rotated[i][k].y + p.y};
        s.convex_world.resize(s.convex_rotated.size());
        for (std::size_t i = 0; i < s.convex_rotated.size(); ++i) {
            s.convex_world[i].resize(s.convex_rotated[i].size());
            for (std::size_t k = 0; k < s.convex_rotated[i].size(); ++k)
                s.convex_world[i][k] = {s.convex_rotated[i][k].x + p.x, s.convex_rotated[i][k].y + p.y};
        }
        IBox all{std::numeric_limits<cInt>::max(), std::numeric_limits<cInt>::max(),
                 std::numeric_limits<cInt>::min(), std::numeric_limits<cInt>::min()};
        for (std::size_t i = 0; i < s.rotated.size(); ++i) {
            s.world[i].resize(s.rotated[i].size());
            IBox pb{std::numeric_limits<cInt>::max(), std::numeric_limits<cInt>::max(),
                    std::numeric_limits<cInt>::min(), std::numeric_limits<cInt>::min()};
            for (std::size_t r = 0; r < s.rotated[i].size(); ++r) {
                const auto& src = s.rotated[i][r];
                auto& dst = s.world[i][r];
                dst.resize(src.size());
                for (std::size_t k = 0; k < src.size(); ++k) {
                    dst[k].X = src[k].X + ox;
                    dst[k].Y = src[k].Y + oy;
                }
                if (r == 0) { // Outer ring bounds the whole part.
                    for (const auto& v : dst) {
                        pb.min_x = std::min(pb.min_x, v.X); pb.max_x = std::max(pb.max_x, v.X);
                        pb.min_y = std::min(pb.min_y, v.Y); pb.max_y = std::max(pb.max_y, v.Y);
                    }
                }
            }
            s.part_box[i] = pb;
            all.min_x = std::min(all.min_x, pb.min_x); all.max_x = std::max(all.max_x, pb.max_x);
            all.min_y = std::min(all.min_y, pb.min_y); all.max_y = std::max(all.max_y, pb.max_y);
        }
        s.box = all;
    }

    // ---- overlap measure -------------------------------------------------------
    double intersection_area(const ClipperLib::Paths& a, const ClipperLib::Paths& b) {
        ++stats_->overlap_tests;
        clip_.Clear();
        clip_.AddPaths(a, ClipperLib::ptSubject, true);
        clip_.AddPaths(b, ClipperLib::ptClip, true);
        solution_.clear();
        if (!clip_.Execute(ClipperLib::ctIntersection, solution_, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd))
            return 0;
        double area = 0;
        for (const auto& path : solution_) area += ClipperLib::Area(path);
        area = std::abs(area) / (kUnits * kUnits);
        return area < kAreaEpsilon ? 0 : area;
    }

    // Exact intersection area of two convex rings by successive half-plane clipping.
    // Both rings are counter-clockwise. About five times cheaper than the general
    // clipper, and most UV islands (and every margin-expanded convex one) qualify.
    double convex_intersection_area(const std::vector<Vec2>& a, const std::vector<Vec2>& b) {
        ++stats_->overlap_tests;
        clip_in_ = a;
        const std::size_t m = b.size();
        for (std::size_t e = 0; e < m; ++e) {
            const Vec2 p = b[e], q = b[(e + 1) % m];
            const double ex = q.x - p.x, ey = q.y - p.y;
            const std::size_t n = clip_in_.size();
            if (n < 3) return 0;
            clip_out_.clear();
            Vec2 prev = clip_in_[n - 1];
            double prev_side = ex * (prev.y - p.y) - ey * (prev.x - p.x);
            for (std::size_t k = 0; k < n; ++k) {
                const Vec2 cur = clip_in_[k];
                const double side = ex * (cur.y - p.y) - ey * (cur.x - p.x);
                if (side >= 0) {
                    if (prev_side < 0) {
                        const double t = prev_side / (prev_side - side);
                        clip_out_.push_back({prev.x + t * (cur.x - prev.x), prev.y + t * (cur.y - prev.y)});
                    }
                    clip_out_.push_back(cur);
                } else if (prev_side >= 0) {
                    const double t = prev_side / (prev_side - side);
                    clip_out_.push_back({prev.x + t * (cur.x - prev.x), prev.y + t * (cur.y - prev.y)});
                }
                prev = cur;
                prev_side = side;
            }
            clip_in_.swap(clip_out_);
        }
        if (clip_in_.size() < 3) return 0;
        const double area = std::abs(ring_signed_area(clip_in_));
        return area < kAreaEpsilon ? 0 : area;
    }

    double pair_overlap(std::size_t i, std::size_t j) {
        const Shape& a = shapes_[i];
        const Shape& b = shapes_[j];
        if (!boxes_overlap(a.box, b.box)) return 0;
        double sum = 0;
        for (std::size_t pa = 0; pa < a.world.size(); ++pa)
            for (std::size_t pb = 0; pb < b.world.size(); ++pb) {
                if (!boxes_overlap(a.part_box[pa], b.part_box[pb])) continue;
                if (quads_separated(a.obb_world[pa], b.obb_world[pb])) continue;
                if (a.convex[pa] && b.convex[pb])
                    sum += convex_intersection_area(a.convex_world[pa], b.convex_world[pb]);
                else
                    sum += intersection_area(a.world[pa], b.world[pb]);
            }
        return sum;
    }

    // Weighted (guided local search) overlap of one island against everything else.
    double weighted_cost(std::size_t i) {
        double cost = 0;
        const std::size_t n = shapes_.size();
        const float* row = &weights_[i * n];
        for (std::size_t j = 0; j < n; ++j) {
            if (j == i) continue;
            const double o = pair_overlap(i, j);
            if (o > 0) cost += double(row[j]) * o;
        }
        return cost;
    }

    double raw_overlap(std::size_t i) {
        double sum = 0;
        for (std::size_t j = 0; j < shapes_.size(); ++j) if (j != i) sum += pair_overlap(i, j);
        return sum;
    }

    double total_overlap() {
        double sum = 0;
        for (std::size_t i = 0; i < movable_; ++i)
            for (std::size_t j = i + 1; j < shapes_.size(); ++j) sum += pair_overlap(i, j);
        return sum;
    }

    // ---- feasible translations -------------------------------------------------
    bool translation_range(const Shape& s, double& lo_x, double& hi_x, double& lo_y, double& hi_y) const {
        lo_x = frame_.min_x - s.local_box.min_x;
        hi_x = frame_.max_x - s.local_box.max_x;
        lo_y = frame_.min_y - s.local_box.min_y;
        hi_y = frame_.max_y - s.local_box.max_y;
        return hi_x >= lo_x && hi_y >= lo_y;
    }

    Vec2 clamp_pos(const Shape& s, Vec2 p) const {
        double lo_x, hi_x, lo_y, hi_y;
        if (!translation_range(s, lo_x, hi_x, lo_y, hi_y)) return p;
        return {std::min(std::max(p.x, lo_x), hi_x), std::min(std::max(p.y, lo_y), hi_y)};
    }

    // ---- local search ----------------------------------------------------------
    // Coordinate descent on the translation with halving steps. `cost_fn` is either
    // the weighted overlap (separation) or overlap + attraction (polish).
    template <class CostFn>
    void descend(std::size_t i, double& cost, const CostFn& cost_fn, double step, double step_min,
                 std::size_t max_evals) {
        Shape& s = shapes_[i];
        double lo_x, hi_x, lo_y, hi_y;
        if (!translation_range(s, lo_x, hi_x, lo_y, hi_y)) return;
        static const double dx[4] = {1, -1, 0, 0};
        static const double dy[4] = {0, 0, 1, -1};
        std::size_t evals = 0;
        while (step > step_min && evals < max_evals) {
            bool improved = false;
            for (int d = 0; d < 4; ++d) {
                Vec2 cand{std::min(std::max(s.pos.x + dx[d] * step, lo_x), hi_x),
                          std::min(std::max(s.pos.y + dy[d] * step, lo_y), hi_y)};
                if (cand.x == s.pos.x && cand.y == s.pos.y) continue;
                const Vec2 keep = s.pos;
                set_pos(s, cand);
                ++evals;
                const double c = cost_fn(i);
                if (c < cost - 1e-16) { cost = c; improved = true; if (c <= 0) return; }
                else set_pos(s, keep);
            }
            if (!improved) step *= 0.5;
        }
    }

    // One relocation of an overlapping island. The repertoire grows with the island's
    // `effort` counter: an island that a plain coordinate descent frees costs almost
    // nothing, and only the ones that stay stuck pay for random restarts and for the
    // continuous rotation search. In a dense pack most islands are in the first class,
    // and spending the same budget on all of them is what starves the sweep count.
    void relocate(std::size_t i, std::mt19937& rng, bool allow_rotation) {
        Shape& s = shapes_[i];
        auto cost_fn = [this](std::size_t k) { return weighted_cost(k); };
        double cost = weighted_cost(i);
        if (cost <= 0) { s.effort = 0; return; }
        ++stats_->relocations;
        const double start_angle = s.angle;
        double best_cost = cost;
        Pose best{s.angle, s.pos};
        const double span = std::max(frame_.max_x - frame_.min_x, frame_.max_y - frame_.min_y);
        const double coarse = 0.12 * span;
        const double fine = std::max(1e-5, 2e-4 * span);
        const int effort = s.effort;

        descend(i, cost, cost_fn, coarse, fine, 44);
        if (cost < best_cost) { best_cost = cost; best = {s.angle, s.pos}; }
        if (best_cost <= 0) { apply(i, best, start_angle); s.effort = 0; return; }

        double lo_x, hi_x, lo_y, hi_y;
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        const int restarts = effort >= 4 ? 3 : (effort >= 2 ? 1 : 0);
        for (int r = 0; r < restarts && best_cost > 0; ++r) {
            if (!translation_range(s, lo_x, hi_x, lo_y, hi_y)) break;
            Vec2 cand;
            if (r % 2 == 0) { // Uniform in the target.
                cand = {lo_x + (hi_x - lo_x) * unit(rng), lo_y + (hi_y - lo_y) * unit(rng)};
            } else {          // Near the current position.
                const double radius = 0.25 * span;
                cand = clamp_pos(s, {best.pos.x + radius * (2 * unit(rng) - 1),
                                     best.pos.y + radius * (2 * unit(rng) - 1)});
            }
            set_pos(s, cand);
            double c = weighted_cost(i);
            descend(i, c, cost_fn, 0.25 * coarse, fine, 36);
            if (c < best_cost) { best_cost = c; best = {s.angle, s.pos}; }
        }

        if (allow_rotation && best_cost > 0 && effort >= 1) {
            // Continuous rotation: a coarse fan, then successive refinement down to a
            // tenth of a degree around whichever coarse angle worked best.
            static const double wide_fan[8] = {12, -12, 30, -30, 60, -60, 90, 180};
            const int fan_size = effort >= 3 ? 8 : 4;
            const double deg = 3.14159265358979323846 / 180.0;
            double best_rot_cost = best_cost;
            Pose best_rot = best;
            for (int k = 0; k < fan_size && best_rot_cost > 0; ++k) {
                set_angle(s, start_angle + wide_fan[k] * deg);
                if (!translation_range(s, lo_x, hi_x, lo_y, hi_y)) continue;
                set_pos(s, clamp_pos(s, best.pos));
                double c = weighted_cost(i);
                descend(i, c, cost_fn, 0.2 * coarse, fine, 28);
                if (c < best_rot_cost) { best_rot_cost = c; best_rot = {s.angle, s.pos}; }
            }
            for (double width : {6.0, 1.5, 0.4, 0.1}) {
                if (best_rot_cost <= 0) break;
                for (int sign = -1; sign <= 1; sign += 2) {
                    set_angle(s, best_rot.angle + sign * width * deg);
                    if (!translation_range(s, lo_x, hi_x, lo_y, hi_y)) continue;
                    set_pos(s, clamp_pos(s, best_rot.pos));
                    double c = weighted_cost(i);
                    descend(i, c, cost_fn, 0.08 * coarse, fine, 20);
                    if (c < best_rot_cost) { best_rot_cost = c; best_rot = {s.angle, s.pos}; }
                }
            }
            if (best_rot_cost < best_cost) { best_cost = best_rot_cost; best = best_rot; }
        }
        apply(i, best, start_angle);
        if (best.angle != start_angle) ++stats_->rotations_applied;
        s.effort = best_cost > 0 ? std::min(6, s.effort + 1) : 0;
    }

    void apply(std::size_t i, const Pose& pose, double current_angle) {
        Shape& s = shapes_[i];
        if (s.angle != pose.angle || current_angle != pose.angle) set_angle(s, pose.angle);
        set_pos(s, pose.pos);
    }

    // Exchange the positions of two islands, then let both settle. Accepted when the
    // weighted cost of the pair strictly improves.
    void try_swap(std::size_t i, std::size_t j) {
        if (i == j || shapes_[i].fixed || shapes_[j].fixed) return;
        auto cost_fn = [this](std::size_t k) { return weighted_cost(k); };
        const double before = weighted_cost(i) + weighted_cost(j);
        if (before <= 0) return;
        const Pose keep_i{shapes_[i].angle, shapes_[i].pos};
        const Pose keep_j{shapes_[j].angle, shapes_[j].pos};
        set_pos(shapes_[i], clamp_pos(shapes_[i], keep_j.pos));
        set_pos(shapes_[j], clamp_pos(shapes_[j], keep_i.pos));
        const double span = std::max(frame_.max_x - frame_.min_x, frame_.max_y - frame_.min_y);
        double ci = weighted_cost(i);
        descend(i, ci, cost_fn, 0.04 * span, std::max(1e-5, 2e-4 * span), 40);
        double cj = weighted_cost(j);
        descend(j, cj, cost_fn, 0.04 * span, std::max(1e-5, 2e-4 * span), 40);
        if (weighted_cost(i) + weighted_cost(j) >= before - 1e-16) {
            set_pos(shapes_[i], keep_i.pos);
            set_pos(shapes_[j], keep_j.pos);
        }
    }

    // ---- stages ------------------------------------------------------------------
    // Returns the residual total overlap: 0 means the layout is separated.
    double separate(double deadline_seconds, const Clock::time_point& t0, std::mt19937& rng, bool rotation);
    void polish(double deadline_seconds, const Clock::time_point& t0, bool rotation);

    bool build_at(double scale);
    void commit(double scale);
    void snapshot(std::vector<Pose>& poses) const {
        poses.resize(movable_);
        for (std::size_t i = 0; i < movable_; ++i) poses[i] = {shapes_[i].angle, shapes_[i].pos};
    }
    void restore(const std::vector<Pose>& poses) {
        for (std::size_t i = 0; i < movable_ && i < poses.size(); ++i) {
            if (shapes_[i].angle != poses[i].angle) set_angle(shapes_[i], poses[i].angle);
            set_pos(shapes_[i], poses[i].pos);
        }
    }
    Layout to_layout(double scale) const;
    bool validate(const Layout& layout) const;

    const std::vector<Island>& islands_;
    PackTarget target_;
    PackOptions options_;
    std::vector<Obstacle> obstacles_;
    double ratio_ = 1;
    DBox frame_{};
    std::vector<Shape> shapes_;
    std::size_t movable_ = 0;
    std::vector<std::size_t> island_index_; // shapes_[i] -> index into islands_
    std::vector<float> weights_;
    std::vector<Pose> accepted_;
    double accepted_scale_ = 1;
    Vec2 accepted_center_{0, 0};
    std::vector<std::uint64_t> order_;
    CompactionStats* stats_ = nullptr;
    ClipperLib::Clipper clip_;
    ClipperLib::Paths solution_;
    std::vector<Vec2> clip_in_, clip_out_;
    const std::atomic<int>* best_rank_ = nullptr;
    int rank_ = -1;
};

bool Compactor::build_at(double scale) {
    const double k = scale / accepted_scale_;
    for (std::size_t i = 0; i < movable_; ++i) {
        Shape next;
        if (!build_shape(islands_[island_index_[i]], shapes_[i].mirrored, scale, next)) return false;
        // Every piece of cached geometry has to come across: the prefilter boxes and
        // the convex rings are the ones the overlap measure actually reads, and a
        // stale copy of them silently under-reports overlap at the new scale.
        shapes_[i].parts = std::move(next.parts);
        shapes_[i].obb = std::move(next.obb);
        shapes_[i].convex = std::move(next.convex);
        shapes_[i].convex_local = std::move(next.convex_local);
        shapes_[i].pivot = next.pivot;
        shapes_[i].island_scale = next.island_scale;
        set_angle(shapes_[i], accepted_[i].angle);
        const Vec2 target_pos{accepted_center_.x + (accepted_[i].pos.x - accepted_center_.x) * k,
                              accepted_center_.y + (accepted_[i].pos.y - accepted_center_.y) * k};
        double lo_x, hi_x, lo_y, hi_y;
        if (!translation_range(shapes_[i], lo_x, hi_x, lo_y, hi_y)) {
            // The grown island no longer fits the frame at this angle. Containment is a
            // hard constraint and the local search only ever moves islands that overlap
            // something, so nothing downstream would turn it: look for an angle that
            // fits here, nearest first. This is the case where a long island has to go
            // diagonally, which no schedule of 90 degree steps can express.
            if (!options_.compaction_rotation) return false;
            bool fitted = false;
            for (int step = 2; step <= 180 && !fitted; ++step) {
                const double offset = (step / 2) * (step % 2 ? 1.0 : -1.0) * (3.14159265358979323846 / 180.0);
                set_angle(shapes_[i], accepted_[i].angle + offset);
                fitted = translation_range(shapes_[i], lo_x, hi_x, lo_y, hi_y);
            }
            if (!fitted) return false; // Too big for the target at any angle.
        }
        set_pos(shapes_[i], clamp_pos(shapes_[i], target_pos));
    }
    return true;
}

void Compactor::commit(double scale) {
    accepted_.resize(movable_);
    DBox content;
    for (std::size_t i = 0; i < movable_; ++i) {
        accepted_[i] = {shapes_[i].angle, shapes_[i].pos};
        content.add(shapes_[i].pos.x + shapes_[i].local_box.min_x, shapes_[i].pos.y + shapes_[i].local_box.min_y);
        content.add(shapes_[i].pos.x + shapes_[i].local_box.max_x, shapes_[i].pos.y + shapes_[i].local_box.max_y);
    }
    accepted_scale_ = scale;
    accepted_center_ = {0.5 * (content.min_x + content.max_x), 0.5 * (content.min_y + content.max_y)};
}

Layout Compactor::to_layout(double scale) const {
    Layout out;
    out.valid = true;
    out.scale = scale;
    out.order = order_;
    DBox content;
    for (std::size_t i = 0; i < movable_; ++i) {
        const Shape& s = shapes_[i];
        IslandPlacement p;
        p.id = s.id;
        p.angle = s.angle;
        p.source_origin = {0, 0};
        const double c = std::cos(s.angle), sn = std::sin(s.angle);
        const Vec2 rotated_pivot{c * s.pivot.x - sn * s.pivot.y, sn * s.pivot.x + c * s.pivot.y};
        p.offset = {s.pos.x - rotated_pivot.x - target_.min_x * ratio_,
                    s.pos.y - rotated_pivot.y - target_.min_y};
        p.mirrored = s.mirrored;
        p.island_scale = s.island_scale;
        p.tile = 0;
        out.placements.push_back(p);
        content.add(s.pos.x + s.local_box.min_x, s.pos.y + s.local_box.min_y);
        content.add(s.pos.x + s.local_box.max_x, s.pos.y + s.local_box.max_y);
    }
    out.occupied_width = content.max_x - content.min_x;
    out.occupied_height = content.max_y - content.min_y;
    return out;
}

bool Compactor::validate(const Layout& layout) const {
    // Grid discretization cannot reduce the caller's requested clearance.
    const auto report = validate_layout(islands_, layout, target_, options_.margin, -1,
                                        obstacles_, ratio_, options_.targets,false,
                                        options_.border_margin<0?options_.margin:options_.border_margin);
    if(!report.ok)return false;
    auto candidate=layout;
    const bool finished=!options_.storage_policy || !options_.storage_policy->finish_compaction_candidates;
    return storage_candidate_valid(candidate,islands_,target_,options_,obstacles_,finished);
}

// Guided local search: sweep the overlapping islands, relocating each to the pose
// that minimises its weighted overlap. When a sweep cannot reduce the total any
// further, the weights of the still-overlapping pairs grow, which reshapes the
// landscape and pushes the search out of the local minimum.
double Compactor::separate(double deadline_seconds, const Clock::time_point& t0, std::mt19937& rng, bool rotation) {
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    const std::size_t n = shapes_.size();
    std::fill(weights_.begin(), weights_.end(), 1.0f);
    double total = total_overlap();
    if (total <= 0) return 0.0;
    double best_total = total;
    std::size_t stale = 0;
    std::vector<std::size_t> active;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    // The penalty weights deliberately push the search away from the current minimum,
    // so the configuration in hand at any moment can be much worse than the best one
    // seen. Keep that one and hand it back at the end.
    std::vector<Pose> best_poses;
    snapshot(best_poses);
    // Reproducible runs cap the sweeps instead of the wall clock (see PackOptions).
    const std::size_t sweep_budget = options_.compaction_attempt_sweeps;
    std::size_t sweeps_done = 0;
    while (elapsed() < deadline_seconds && !options_.cancel.requested() && !aborted() &&
           (sweep_budget == 0 || sweeps_done < sweep_budget)) {
        ++stats_->sweeps;
        ++sweeps_done;
        active.clear();
        for (std::size_t i = 0; i < movable_; ++i) if (raw_overlap(i) > 0) active.push_back(i);
        if (active.empty()) return 0.0;
        std::shuffle(active.begin(), active.end(), rng);
        // Ripple: relocating an island usually pushes it into a neighbour that was
        // perfectly happy, and a neighbour with no overlap has zero cost, so nothing
        // in the sweep would move it. Queueing whoever the move just disturbed lets
        // the pressure travel across the pack inside one sweep instead of one island
        // per sweep, which is what a jammed layout needs.
        std::size_t head = 0;
        std::vector<char> queued(movable_, 0);
        for (const auto i : active) queued[i] = 1;
        const std::size_t pop_limit = 3 * movable_ + 16;
        for (std::size_t pops = 0; head < active.size() && pops < pop_limit; ++pops) {
            const auto i = active[head++];
            queued[i] = 0;
            relocate(i, rng, rotation);
            if (elapsed() >= deadline_seconds || aborted()) break;
            if (unit(rng) < 0.15) { // Occasional swap with a random other island.
                const auto j = std::uniform_int_distribution<std::size_t>(0, movable_ - 1)(rng);
                try_swap(i, j);
            }
            for (std::size_t j = 0; j < movable_; ++j) {
                if (j == i || queued[j] || pair_overlap(i, j) <= 0) continue;
                queued[j] = 1;
                active.push_back(j);
            }
        }
        total = total_overlap();
        if (total <= 0) return 0.0;
        if (total < best_total * (1.0 - 1e-6)) { best_total = total; stale = 0; snapshot(best_poses); }
        else ++stale;
        // GLS penalty update: make the pairs that are still stuck more expensive.
        for (std::size_t i = 0; i < movable_; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                if (pair_overlap(i, j) <= 0) continue;
                weights_[i * n + j] = std::min(4096.0f, weights_[i * n + j] + 1.0f);
                weights_[j * n + i] = weights_[i * n + j];
            }
        if (stale >= 8) { // Kick: return to the best configuration, jitter, forget the penalties.
            stale = 0;
            std::fill(weights_.begin(), weights_.end(), 1.0f);
            restore(best_poses);
            const double span = std::max(frame_.max_x - frame_.min_x, frame_.max_y - frame_.min_y);
            for (std::size_t i = 0; i < movable_; ++i) {
                if (raw_overlap(i) <= 0 || unit(rng) > 0.5) continue;
                set_pos(shapes_[i], clamp_pos(shapes_[i], {shapes_[i].pos.x + 0.04 * span * (2 * unit(rng) - 1),
                                                           shapes_[i].pos.y + 0.04 * span * (2 * unit(rng) - 1)}));
            }
            total = total_overlap();
        }
    }
    if (total_overlap() > best_total) { restore(best_poses); total = total_overlap(); }
    return total_overlap();
}

// Zero-growth polish: keep the layout overlap-free and pull every island toward the
// frame's minimum corner, with small continuous rotations allowed. This costs no
// scale by itself; it gathers the free space into one region so the next growth
// step has somewhere to expand into.
void Compactor::polish(double deadline_seconds, const Clock::time_point& t0, bool rotation) {
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    const Vec2 attractor{frame_.min_x, frame_.min_y};
    const double span = std::max(frame_.max_x - frame_.min_x, frame_.max_y - frame_.min_y);
    const double big = 1e6 * span * span;
    auto cost_fn = [&](std::size_t k) {
        const double dx = shapes_[k].pos.x - attractor.x, dy = shapes_[k].pos.y - attractor.y;
        return big * raw_overlap(k) + dx * dx + dy * dy;
    };
    std::vector<std::size_t> order(movable_);
    std::iota(order.begin(), order.end(), 0);
    const double deg = 3.14159265358979323846 / 180.0;
    for (int sweep = 0; sweep < 8 && elapsed() < deadline_seconds && !options_.cancel.requested() && !aborted(); ++sweep) {
        // Closest to the corner first: they settle and let the far ones follow.
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
            const double da = std::hypot(shapes_[a].pos.x - attractor.x, shapes_[a].pos.y - attractor.y);
            const double db = std::hypot(shapes_[b].pos.x - attractor.x, shapes_[b].pos.y - attractor.y);
            return da < db;
        });
        double moved = 0;
        for (const auto i : order) {
            if (elapsed() >= deadline_seconds) break;
            double cost = cost_fn(i);
            const Vec2 before = shapes_[i].pos;
            descend(i, cost, cost_fn, 0.1 * span, std::max(1e-5, 2e-4 * span), 60);
            if (rotation) {
                const Pose keep{shapes_[i].angle, shapes_[i].pos};
                double best_cost = cost;
                Pose best = keep;
                for (double d : {5.0, -5.0, 15.0, -15.0, 45.0, -45.0}) {
                    set_angle(shapes_[i], keep.angle + d * deg);
                    double lo_x, hi_x, lo_y, hi_y;
                    if (!translation_range(shapes_[i], lo_x, hi_x, lo_y, hi_y)) continue;
                    set_pos(shapes_[i], clamp_pos(shapes_[i], keep.pos));
                    double c = cost_fn(i);
                    descend(i, c, cost_fn, 0.05 * span, std::max(1e-5, 2e-4 * span), 40);
                    if (c < best_cost) { best_cost = c; best = {shapes_[i].angle, shapes_[i].pos}; }
                }
                set_angle(shapes_[i], best.angle);
                set_pos(shapes_[i], best.pos);
                cost = best_cost;
            }
            moved += std::hypot(shapes_[i].pos.x - before.x, shapes_[i].pos.y - before.y);
        }

        if (moved < 1e-6 * span) break;
    }
}

bool Compactor::load(const Layout& start, CompactionStats& stats) {
    if (options_.targets.size() > 1) { stats.skipped = "multi-tile layout"; return false; }
    if (!start.unplaced.empty()) { stats.skipped = "partial layout"; return false; }
    if (start.placements.size() != islands_.size()) { stats.skipped = "placement count mismatch"; return false; }
    if (!(frame_.max_x > frame_.min_x) || !(frame_.max_y > frame_.min_y)) { stats.skipped = "empty frame"; return false; }

    order_ = start.order;
    shapes_.clear();
    island_index_.clear();
    for (const auto& placement : start.placements) {
        std::size_t index = 0;
        while (index < islands_.size() && islands_[index].id != placement.id) ++index;
        if (index == islands_.size()) { stats.skipped = "unknown island in layout"; return false; }
        Shape shape;
        if (!build_shape(islands_[index], placement.mirrored, start.scale, shape)) {
            stats.skipped = "margin expansion failed";
            return false;
        }
        set_angle(shape, placement.angle);
        // The incumbent's source_origin is the bounds minimum of exactly this rotated,
        // expanded geometry, so the pivot's packed position follows directly.
        const double c = std::cos(placement.angle), sn = std::sin(placement.angle);
        const Vec2 rotated_pivot{c * shape.pivot.x - sn * shape.pivot.y, sn * shape.pivot.x + c * shape.pivot.y};
        const Vec2 pos{rotated_pivot.x - placement.source_origin.x + placement.offset.x + target_.min_x * ratio_,
                       rotated_pivot.y - placement.source_origin.y + placement.offset.y + target_.min_y};
        set_pos(shape, pos);
        shapes_.push_back(std::move(shape));
        island_index_.push_back(index);
    }
    movable_ = shapes_.size();
    if (movable_ == 0) { stats.skipped = "no placements"; return false; }
    for (const auto& obstacle : obstacles_) {
        Shape shape;
        if (!build_obstacle(obstacle, shape)) { stats.skipped = "obstacle expansion failed"; return false; }
        set_angle(shape, 0);
        set_pos(shape, {0, 0});
        shapes_.push_back(std::move(shape));
    }
    weights_.assign(shapes_.size() * shapes_.size(), 1.0f);
    commit(start.scale);
    return true;
}

bool Compactor::adopt(const std::vector<Pose>& poses, double scale, Vec2 center) {
    if (poses.size() != movable_) return false;
    accepted_ = poses;
    accepted_scale_ = scale;
    accepted_center_ = center;
    return build_at(scale); // k == 1, so this only rebuilds the geometry at `scale`.
}

bool Compactor::prepare_incumbent(double seconds, const Clock::time_point& t0, std::mt19937& rng, Layout& best) {
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    const bool rotation = options_.compaction_rotation;
    bool changed = false;

    // The incoming layout must already be overlap-free in this representation; if it
    // is not (a different rasterization path, a pixel-aligned layout), repair it
    // first at the same scale so growth starts from a clean state.
    const double initial_overlap = total_overlap();
    if (options_.progress && initial_overlap > 0)
        std::fprintf(stderr, "[uvpack] compaction: incoming layout overlaps by %.3e\n", initial_overlap);
    if (initial_overlap > 0) {
        separate(std::min(seconds * 0.25, 0.5), t0, rng, rotation);
        if (total_overlap() <= 0) {
            auto candidate = to_layout(accepted_scale_);
            if (validate(candidate)) { best = candidate; commit(accepted_scale_); changed = true; stream(best); }
            else { build_at(accepted_scale_); }
        } else {
            build_at(accepted_scale_);
        }
    }

    // compaction_delta == 0 means "polish only": keep the scale and spend the whole
    // budget on slack and rotation. That is what fixed-scale packing asks for.
    const bool growth = options_.compaction_delta > 0;
    if ((options_.compaction_polish || !growth) && elapsed() < seconds) {
        const double polish_deadline = std::min(seconds, elapsed() + (growth ? std::max(0.1, 0.2 * seconds) : seconds));
        const double polish_start = elapsed();
        polish(polish_deadline, t0, rotation);
        stats_->polish_seconds = elapsed() - polish_start;
        auto candidate = to_layout(accepted_scale_);
        if (total_overlap() <= 0 && validate(candidate)) {
            if (candidate.scale >= best.scale) best = candidate;
            commit(accepted_scale_);
            changed = true;
            stream(best);
        } else {
            build_at(accepted_scale_); // Roll back to the committed poses.
        }
    }
    return changed;
}

bool Compactor::try_scale(double candidate_scale, double deadline_seconds, const Clock::time_point& t0,
                          std::mt19937& rng, bool rotation, Layout& out) {
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    ++stats_->attempts;
    if (!build_at(candidate_scale)) {
        build_at(accepted_scale_);
        return false;
    }
    const double start_overlap = options_.progress ? total_overlap() : 0.0;
    const double attempt_start = elapsed();
    const double residual = separate(deadline_seconds, t0, rng, rotation);
    if (options_.progress)
        std::fprintf(stderr, "[uvpack] compaction: try %.5f overlap %.3e -> %.3e in %.2fs\n",
                     candidate_scale, start_overlap, residual, elapsed() - attempt_start);
    if (residual <= 0) {
        auto candidate = to_layout(candidate_scale);
        if (validate(candidate)) {
            out = std::move(candidate);
            ++stats_->accepted;
            return true; // Leaves the shapes at the accepted poses, ready for commit().
        }
        ++stats_->rejected_by_validator;
    }
    build_at(accepted_scale_);
    return false;
}

// The first relative step. A failed attempt costs a whole time slice, and the more
// islands there are the less a single growth step is likely to separate, so start
// smaller when there are many of them. Successes grow the step back quickly.
double Compactor::initial_delta() const {
    return options_.compaction_delta * std::min(1.5, std::max(0.4, std::sqrt(40.0 / double(movable_))));
}

void Compactor::report(const CompactionStats& stats) const {
    if (!options_.progress) return;
    std::fprintf(stderr, "[uvpack] compaction: %.5f -> %.5f (%+.2f%%) in %.2fs on %zu thread(s), %zu rounds, "
                         "%zu attempts / %zu accepted / %zu rejected, %zu sweeps, %zu relocations (%zu rotated), "
                         "%zu overlap tests, polish %.2fs\n",
                 stats.start_scale, stats.best_scale, 100.0 * (stats.best_scale / stats.start_scale - 1.0),
                 stats.seconds, stats.threads, stats.rounds, stats.attempts, stats.accepted,
                 stats.rejected_by_validator, stats.sweeps, stats.relocations, stats.rotations_applied,
                 stats.overlap_tests, stats.polish_seconds);
}

Layout Compactor::run(const Layout& start, double seconds, CompactionStats& stats) {
    stats_ = &stats;
    stats.start_scale = start.scale;
    stats.best_scale = start.scale;
    stats.threads = 1;
    const auto t0 = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    Layout best = start;

    if (!load(start, stats)) return best;
    stats.ran = true;

    std::mt19937 rng(options_.seed * 2654435761u + 12345u);
    const bool rotation = options_.compaction_rotation;
    prepare_incumbent(seconds, t0, rng, best);

    double delta = initial_delta();
    const double initial = delta;
    const double min_delta = 2e-4;
    unsigned successes = 0;
    int rearms = 0;
    // Halving all the way down means the budget is spent; re-arm at a moderate step
    // instead, the way the greedy's refine stage does, so the remaining time
    // gets another run at it with fresh randomisation.
    auto shrink = [&] {
        delta *= 0.5;
        if (delta >= min_delta) return true;
        if (rearms >= 2) return false;
        ++rearms;
        delta = initial * 0.5;
        return true;
    };
    const bool growth = options_.compaction_delta > 0;
    const std::size_t max_rounds = options_.compaction_max_rounds;
    while (growth && elapsed() < seconds && !options_.cancel.requested() &&
           (max_rounds == 0 || stats.rounds < max_rounds)) {
        ++stats.rounds;
        const double candidate_scale = accepted_scale_ * (1.0 + delta);
        const double attempt_start = elapsed();
        const double remaining = seconds - attempt_start;
        // Even slices: a successful separation returns early anyway, and a failure
        // late in the budget is no easier than an early one just because time is short.
        const double slice = options_.compaction_attempt_sweeps > 0
                                 ? seconds
                                 : attempt_start + std::min(remaining, std::max(0.1, seconds / 6.0));
        Layout candidate;
        if (try_scale(candidate_scale, slice, t0, rng, rotation, candidate)) {
            best = std::move(candidate);
            commit(candidate_scale);
            stream(best);
            if (options_.progress)
                std::fprintf(stderr, "[uvpack] compaction: scale %.5f (+%.2f%%, %.1fs)\n",
                             candidate_scale, 100.0 * (candidate_scale / stats.start_scale - 1.0), elapsed());
            if (++successes >= 2) { delta = std::min(delta * 1.6, 0.05); successes = 0; }
        } else {
            successes = 0;
            if (!shrink()) break;
        }
    }
    stats.best_scale = best.scale;
    stats.seconds = elapsed();
    report(stats);
    return best;
}

// ---- portfolio driver ----------------------------------------------------------
// Multiplier applied to the current step by attempt `w` of `k`: a geometric ladder
// from half the step to three times it. One round therefore brackets the step instead
// of probing it, and the largest multiplier that separated becomes the next step.
double ladder_multiplier(std::size_t w, std::size_t k) {
    if (k <= 1) return 1.0;
    return 0.5 * std::pow(6.0, double(w) / double(k - 1));
}

struct PortfolioWorker {
    std::unique_ptr<Compactor> compactor;
    CompactionStats stats;
    std::mt19937 rng;
    Layout result;
    bool alive = false;    // Loaded, and its geometry still rebuilds.
    bool success = false;  // Separated and validated in the last round.
    double scale = 0;
    std::size_t generation = 0;
};

Layout run_portfolio(const std::vector<Island>& islands, const PackTarget& target,
                     const PackOptions& options, const std::vector<Obstacle>& obstacles,
                     const Layout& start, double seconds, std::size_t threads, CompactionStats& stats) {
    const auto t0 = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    stats.start_scale = start.scale;
    stats.best_scale = start.scale;
    stats.threads = threads;
    Layout best = start;

    std::vector<PortfolioWorker> workers(threads);
    for (std::size_t w = 0; w < threads; ++w) {
        workers[w].compactor.reset(new Compactor(islands, target, options, obstacles));
        workers[w].compactor->set_stats(&workers[w].stats);
        // Independent streams: the same layout is attacked from `threads` different
        // random restarts even where two attempts share a target delta.
        workers[w].rng.seed(options.seed * 2654435761u + 12345u + static_cast<std::uint32_t>(w) * 2246822519u);
    }
    // Worker 0 first, so a declined problem reports its reason before any thread starts.
    if (!workers[0].compactor->load(start, stats)) return best;
    workers[0].alive = true;
    {
        // Loading margin-expands every island; doing it on the threads keeps the
        // portfolio's fixed cost at one expansion pass instead of `threads` of them.
        ThreadGroup pool;
        for (std::size_t w = 1; w < threads; ++w)
            pool.emplace_back([&workers, &start, w] {
                workers[w].alive = workers[w].compactor->load(start, workers[w].stats);
            });
        pool.join();
    }
    stats.ran = true;

    const bool rotation = options.compaction_rotation;
    // The preamble (repairing an overlapping input, the zero-growth polish) changes the
    // incumbent for everyone, so one attempt runs it and the rest adopt the result.
    std::mt19937 preamble_rng(options.seed * 2654435761u + 12345u);
    std::size_t generation = 0;
    if (workers[0].compactor->prepare_incumbent(seconds, t0, preamble_rng, best)) {
        generation = 1;
        workers[0].generation = 1;
    }
    std::vector<Pose> incumbent_poses = workers[0].compactor->accepted();
    double incumbent_scale = workers[0].compactor->accepted_scale();
    Vec2 incumbent_center = workers[0].compactor->accepted_center();

    double delta = workers[0].compactor->initial_delta();
    const double initial = delta;
    const double min_delta = 2e-4;
    int rearms = 0;
    auto shrink = [&] {
        delta *= 0.5;
        if (delta >= min_delta) return true;
        if (rearms >= 2) return false;
        ++rearms;
        delta = initial * 0.5;
        return true;
    };

    // Short rounds take the easy gains quickly; when a whole round fails its attempts
    // were cut off mid-separation, so the next one gets a longer slice.
    const bool fixed_sweeps = options.compaction_attempt_sweeps > 0;
    const double base_slice = std::max(0.08, seconds / 10.0);
    const double max_slice = std::max(base_slice, seconds / 3.0);
    int consecutive_failures = 0;
    std::atomic<int> best_rank{-1};
    for (std::size_t w = 0; w < threads; ++w)
        workers[w].compactor->set_rank(fixed_sweeps ? nullptr : &best_rank, static_cast<int>(w));

    const bool growth = options.compaction_delta > 0;
    const std::size_t max_rounds = options.compaction_max_rounds;
    while (growth && elapsed() < seconds && !options.cancel.requested() &&
           (max_rounds == 0 || stats.rounds < max_rounds)) {
        ++stats.rounds;
        best_rank.store(-1, std::memory_order_relaxed);
        const double round_start = elapsed();
        const double slice = std::min(max_slice, base_slice * std::pow(1.5, double(consecutive_failures)));
        const double deadline = fixed_sweeps ? seconds : round_start + std::min(seconds - round_start, slice);
        auto attempt = [&](std::size_t w) {
            PortfolioWorker& wk = workers[w];
            wk.success = false;
            if (!wk.alive) return;
            if (wk.generation != generation) {
                if (!wk.compactor->adopt(incumbent_poses, incumbent_scale, incumbent_center)) {
                    wk.alive = false;
                    return;
                }
                wk.generation = generation;
            }
            wk.scale = incumbent_scale * (1.0 + delta * ladder_multiplier(w, threads));
            if (!wk.compactor->try_scale(wk.scale, deadline, t0, wk.rng, rotation, wk.result)) return;
            wk.success = true;
            int seen = best_rank.load(std::memory_order_relaxed);
            while (seen < static_cast<int>(w) &&
                   !best_rank.compare_exchange_weak(seen, static_cast<int>(w), std::memory_order_relaxed)) {
            }
        };
        ThreadGroup pool;
        for (std::size_t w = 1; w < threads; ++w) pool.emplace_back(attempt, w);
        attempt(0);
        pool.join();

        // Largest validated scale wins; ties go to the lowest attempt index, so the
        // round's outcome does not depend on which thread happened to finish first.
        std::size_t winner = threads;
        for (std::size_t w = 0; w < threads; ++w)
            if (workers[w].alive && workers[w].success &&
                (winner == threads || workers[w].scale > workers[winner].scale))
                winner = w;
        if (winner == threads) {
            ++consecutive_failures;
            if (!shrink()) break;
            continue;
        }
        consecutive_failures = 0;
        best = workers[winner].result;
        if (options.on_improved) options.on_improved(best);
        workers[winner].compactor->commit_current(workers[winner].scale);
        incumbent_poses = workers[winner].compactor->accepted();
        incumbent_scale = workers[winner].compactor->accepted_scale();
        incumbent_center = workers[winner].compactor->accepted_center();
        workers[winner].generation = ++generation;
        // The winner's multiplier becomes the next step: the round bracketed it, so
        // this is where the ladder should sit. Floored, so a run of small successes
        // cannot drive the step down to nothing.
        delta = std::max(min_delta, std::min(0.05, delta * ladder_multiplier(winner, threads)));
        if (options.progress)
            std::fprintf(stderr, "[uvpack] compaction: round %zu scale %.5f (+%.2f%%, attempt %zu of %zu, %.1fs)\n",
                         stats.rounds, best.scale, 100.0 * (best.scale / stats.start_scale - 1.0),
                         winner, threads, elapsed());
    }

    for (const auto& wk : workers) {
        stats.attempts += wk.stats.attempts;
        stats.accepted += wk.stats.accepted;
        stats.rejected_by_validator += wk.stats.rejected_by_validator;
        stats.sweeps += wk.stats.sweeps;
        stats.relocations += wk.stats.relocations;
        stats.overlap_tests += wk.stats.overlap_tests;
        stats.rotations_applied += wk.stats.rotations_applied;
    }
    stats.polish_seconds = workers[0].stats.polish_seconds;
    stats.best_scale = best.scale;
    stats.seconds = elapsed();
    workers[0].compactor->report(stats);
    return best;
}

} // namespace

static Layout compact_impl(const std::vector<Island>& islands, const PackTarget& target,
                      const Layout& start, const PackOptions& options,
                      const std::vector<Obstacle>& obstacles, CompactionStats* stats) {
    CompactionStats local;
    CompactionStats& st = stats ? *stats : local;
    if (!start.valid || start.placements.empty() || islands.empty()) {
        st.skipped = "no valid layout to compact";
        st.start_scale = start.scale;
        st.best_scale = start.scale;
        return start;
    }
    const double seconds = options.compaction_time;
    if (!(seconds > 0)) {
        st.skipped = "compaction disabled";
        st.start_scale = start.scale;
        st.best_scale = start.scale;
        return start;
    }
    std::size_t threads = options.compaction_threads;
    if (threads == 0) threads = options.threads;
    if (threads == 0) {
        const auto hardware = std::thread::hardware_concurrency();
        threads = hardware == 0 ? 4 : hardware;
    }
    // Polish-only runs (compaction_delta == 0, which is what fixed-scale packing asks
    // for) have no rounds to spread over threads: one attempt owns the whole budget.
    if (!(options.compaction_delta > 0)) threads = 1;
    try {
        if (threads <= 1) {
            Compactor compactor(islands, target, options, obstacles);
            return compactor.run(start, seconds, st);
        }
        return run_portfolio(islands, target, options, obstacles, start, seconds, threads, st);
    } catch (const std::exception& ex) {
        st.skipped = ex.what();
        st.best_scale = start.scale;
        return start;
    }
}

Layout compact_layout(const std::vector<Island>& islands,const PackTarget& target,
                      const Layout& start,const PackOptions& options,
                      const std::vector<Obstacle>& obstacles,CompactionStats* stats) {
    if(options.cancel.requested()) {
        if(stats){stats->skipped="packing cancelled";stats->start_scale=start.scale;stats->best_scale=start.scale;}
        return finish_result(start,islands,target,options,obstacles);
    }
    const auto prepared=storage_search_options(islands,target,options,obstacles,false);
    return finish_result(compact_impl(islands,target,start,prepared,obstacles,stats),islands,target,options,obstacles);
}
} // namespace uvpack
