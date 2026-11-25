#include "uvpack/input_check.hpp"
#include "uvpack/geometry.hpp"
#include "uvpack/bounding.hpp"
#include <set>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace uvpack {
namespace {
double cross(Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

bool proper_cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

bool on_segment(Vec2 a, Vec2 b, Vec2 p) {
    return cross(a,b,p)==0 && p.x>=std::min(a.x,b.x) && p.x<=std::max(a.x,b.x)
        && p.y>=std::min(a.y,b.y) && p.y<=std::max(a.y,b.y);
}

bool ring_self_intersects(const Ring& ring) {
    const std::size_t n = ring.size();
    if (n < 4) return false;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue; // Adjacent through the closing edge.
            const auto a=ring[i],b=ring[(i+1)%n],c=ring[j],d=ring[(j+1)%n];
            // Stored geometry rejects non-adjacent contacts too. Report these
            // during input policy evaluation so rejection/recovery retains IDs.
            if (proper_cross(a,b,c,d) || on_segment(a,b,c) || on_segment(a,b,d)
                || on_segment(c,d,a) || on_segment(c,d,b)) return true;
        }
    }
    return false;
}

bool inside(const Ring& ring, Vec2 p) {
    bool in = false;
    const std::size_t n = ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = ring[i], b = ring[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < x) in = !in;
        }
    }
    return in;
}

bool rings_cross(const Ring& a, const Ring& b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = 0; j < b.size(); ++j)
            if (proper_cross(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()])) return true;
    return false;
}

// Returns true when the ring changed.
bool tidy_ring(Ring& ring) {
    const Ring before = ring;
    if (ring.size() > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) ring.pop_back();
    Ring out;
    for (const auto p : ring) {
        if (!out.empty() && out.back().x == p.x && out.back().y == p.y) continue;
        out.push_back(p);
    }
    // Drop collinear repeats (zero-length turn backs) that Clipper would choke on.
    Ring cleaned;
    for (std::size_t i = 0; i < out.size(); ++i) {
        const Vec2 prev = out[(i + out.size() - 1) % out.size()], cur = out[i], next = out[(i + 1) % out.size()];
        if (out.size() >= 3 && cross(prev, cur, next) == 0 && ((cur.x - prev.x) * (next.x - cur.x) + (cur.y - prev.y) * (next.y - cur.y)) < 0) continue; // spike
        cleaned.push_back(cur);
    }
    ring = std::move(cleaned);
    return ring.size() != before.size() || !std::equal(ring.begin(), ring.end(), before.begin(), [](Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; });
}
} // namespace

InputReport check_and_repair(std::vector<Island>& islands, bool strict) {
    InputReport report;
    std::vector<Island> kept;
    for (auto& island : islands) {
        bool changed = false, fatal = false;
        auto note = [&](const std::string& m, bool f) { report.issues.push_back({island.id, m, f, f ? "invalid_geometry" : "input_repair"}); fatal = fatal || f || strict; };
        auto check_polygon = [&](Polygon& polygon, const std::string& label) {
            for (const auto p : polygon.outer) if (!std::isfinite(p.x) || !std::isfinite(p.y)) { note(label + ": non-finite coordinate", true); return; }
            for (const auto& hole : polygon.holes) for (const auto p : hole) if (!std::isfinite(p.x) || !std::isfinite(p.y)) { note(label + ": non-finite hole coordinate", true); return; }
            if (tidy_ring(polygon.outer)) { changed = true; note(label + ": repeated or closing vertices removed", false); }
            if (polygon.outer.size() < 3) { note(label + ": fewer than three distinct vertices", true); return; }
            const double area = clockwise_ring_area(polygon.outer);
            if (!std::isfinite(area)) { note(label + ": numerical area overflow", true); return; }
            if (area == 0) { note(label + ": zero area", true); return; }
            if (area > 0) { std::reverse(polygon.outer.begin(), polygon.outer.end()); changed = true; note(label + ": outer winding normalized", false); }
            if (ring_self_intersects(polygon.outer)) { note(label + ": self-intersecting outer ring", true); return; }
            std::vector<Ring> holes;
            for (auto hole : polygon.holes) {
                if (tidy_ring(hole)) { changed = true; note(label + ": hole vertices tidied", false); }
                if (hole.size() < 3 || clockwise_ring_area(hole) == 0) { changed = true; note(label + ": degenerate hole dropped", false); continue; }
                if (clockwise_ring_area(hole) < 0) { std::reverse(hole.begin(), hole.end()); changed = true; }
                if (ring_self_intersects(hole)) { note(label + ": self-intersecting hole", true); return; }
                bool contained = true;
                for (const auto p : hole) contained = contained && inside(polygon.outer, p);
                if (!contained || rings_cross(hole, polygon.outer)) { note(label + ": hole not inside the outer ring", true); return; }
                holes.push_back(std::move(hole));
            }
            polygon.holes = std::move(holes);
        };
        check_polygon(island.polygon, "polygon");
        for (std::size_t k = 0; k < island.extra_parts.size() && !fatal; ++k) check_polygon(island.extra_parts[k], "part " + std::to_string(k));
        if (fatal) { ++report.rejected; report.complete=false; report.rejected_ids.push_back(island.id); continue; }
        if (changed) ++report.repaired;
        kept.push_back(std::move(island));
    }
    islands = std::move(kept);
    return report;
}

InputReport check_and_repair(std::vector<Island>& islands, const InputPolicy& policy) {
    if (policy.hull_policy != "reject" && policy.hull_policy != "convex_hull")
        throw std::invalid_argument("hull_policy must be reject or convex_hull");
    std::set<std::uint64_t> ids;
    for (const auto& i : islands)
        if (!ids.insert(i.id).second) throw std::invalid_argument("duplicate island ID");
    InputReport report;
    std::vector<Island> kept;
    for (const auto& original : islands) {
        Ring points=original.envelope_points;
        auto collect=[&](const Polygon& p) {
            points.insert(points.end(),p.outer.begin(),p.outer.end());
            for(const auto& h:p.holes) points.insert(points.end(),h.begin(),h.end());
        };
        collect(original.polygon);
        for(const auto& p:original.extra_parts) collect(p);
        bool finite=std::all_of(points.begin(),points.end(),[](Vec2 p){return std::isfinite(p.x)&&std::isfinite(p.y);});
        std::vector<Island> candidate{original};
        InputReport checked;
        if (finite) checked=check_and_repair(candidate,policy.strict);
        else {
            candidate.clear();
            checked.issues.push_back({original.id,"non-finite source/envelope coordinate",true,"nonfinite_geometry"});
        }
        auto contains=[](const Ring& ring,Vec2 p) {
            for(std::size_t j=0;j<ring.size();++j) {
                auto a=ring[j],b=ring[(j+1)%ring.size()];
                if(cross(a,b,p)==0 && p.x>=std::min(a.x,b.x) && p.x<=std::max(a.x,b.x)
                   && p.y>=std::min(a.y,b.y) && p.y<=std::max(a.y,b.y)) return true;
            }
            return !ring.empty() && inside(ring,p);
        };
        if(!candidate.empty()) {
            for(auto p:original.envelope_points) {
                bool covered=contains(candidate[0].polygon.outer,p);
                for(const auto& part:candidate[0].extra_parts) covered=covered||contains(part.outer,p);
                if(!covered) {
                    checked.issues.push_back({original.id,"source corner outside packing envelope",true,"uncovered_corner"});
                    candidate.clear();break;
                }
            }
        }
        bool rescued=false;
        if(candidate.empty() && finite && !policy.strict && policy.hull_policy=="convex_hull") {
            try {
                Island hull=original;
                hull.polygon={convex_boundary(points),{}};
                hull.extra_parts.clear();
                candidate={hull};
                check_and_repair(candidate,false);
                if(!candidate.empty()) {
                    for(auto p:points) if(!contains(candidate[0].polygon.outer,p)) {
                        candidate.clear();break;
                    }
                }
                if(!candidate.empty()) {
                    rescued=true;
                    report.hull_ids.push_back(original.id);
                    for(auto& issue:checked.issues) issue.fatal=false;
                    checked.issues.push_back({original.id,"conservative convex hull substituted",false,"convex_hull"});
                }
            } catch(const std::invalid_argument& e) {
                checked.issues.push_back({original.id,e.what(),true,"degenerate_envelope"});
            }
        }
        for(auto& issue:checked.issues) {
            if(issue.reason.empty()) issue.reason=issue.fatal?"invalid_geometry":"input_repair";
            report.issues.push_back(std::move(issue));
        }
        if(candidate.empty()) {
            report.complete=false;++report.rejected;report.rejected_ids.push_back(original.id);
        } else {
            if(rescued || checked.repaired) ++report.repaired;
            kept.push_back(std::move(candidate[0]));
        }
    }
    islands=std::move(kept);
    return report;
}
} // namespace uvpack
