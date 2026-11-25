#include "uvpack/validator.hpp"
#include "interior_overlap.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <map>
#include <stdexcept>

namespace uvpack {
namespace {
struct Box { double min_x, min_y, max_x, max_y; };
struct ValidationCancelled {};
void check_cancel(const CancellationToken* cancel) {
    if(cancel && cancel->requested())throw ValidationCancelled{};
}
double checked_tolerance(const PackTarget& target,const ValidationOptions& options) {
    if(!std::isfinite(options.gap)||options.gap<0||!std::isfinite(options.border)||options.border<0
       ||!std::isfinite(options.target_aspect)||options.target_aspect<=0)
        throw std::invalid_argument("gap/border must be finite nonnegative and aspect finite positive");
    return validation_tolerance(target,options.targets,options.output_precision);
}

// Rings in packing space: output transform applied, then x scaled by the target aspect,
// so gaps are isotropic pixel distances for non-square textures.
using Part = std::vector<Ring>;      // outer ring first, then holes
using Shape = std::vector<Part>;     // one island: its polygon and extra parts

Shape transformed_shape(const Island& island, const IslandPlacement& p, double scale, const PackTarget& target, double target_aspect) {
    const auto t = p.transform(scale, target, target_aspect);
    auto map = [&](Vec2 v) { return Vec2{(t[0] * v.x + t[1] * v.y + t[2]) * target_aspect, t[3] * v.x + t[4] * v.y + t[5]}; };
    Shape shape;
    auto add_polygon = [&](const Polygon& polygon) {
        Part part;
        auto add = [&](const Ring& ring) {
            Ring out;
            std::size_t n = ring.size();
            if (n > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) --n;
            for (std::size_t i = 0; i < n; ++i) out.push_back(map(ring[i]));
            part.push_back(std::move(out));
        };
        add(polygon.outer);
        for (const auto& hole : polygon.holes) add(hole);
        shape.push_back(std::move(part));
    };
    add_polygon(island.polygon);
    for (const auto& part : island.extra_parts) add_polygon(part);
    return shape;
}

Box bounds(const Shape& shape, const CancellationToken* cancel=nullptr) {
    Box b{1e300, 1e300, -1e300, -1e300};
    for (const auto& part : shape) for (const auto& ring : part) for (const auto p : ring) {
        check_cancel(cancel);
        b.min_x = std::min(b.min_x, p.x); b.max_x = std::max(b.max_x, p.x);
        b.min_y = std::min(b.min_y, p.y); b.max_y = std::max(b.max_y, p.y);
    }
    return b;
}

// Even-odd point in polygon over one part's rings (outer + holes).
bool contains_part(const Part& rings, Vec2 p, const CancellationToken* cancel=nullptr) {
    bool in = false;
    for (const auto& ring : rings) {
        const std::size_t n = ring.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            check_cancel(cancel);
            const Vec2 a = ring[i], b = ring[j];
            if ((a.y > p.y) != (b.y > p.y)) {
                const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (p.x < x) in = !in;
            }
        }
    }
    return in;
}

double cross(Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

bool segments_cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

double point_segment_distance(Vec2 p, Vec2 a, Vec2 b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
    t = std::clamp(t, 0.0, 1.0);
    const double qx = a.x + t * dx - p.x, qy = a.y + t * dy - p.y;
    return std::sqrt(qx * qx + qy * qy);
}

double segment_distance(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    if (segments_cross(a, b, c, d)) return 0;
    return std::min({point_segment_distance(a, c, d), point_segment_distance(b, c, d),
                     point_segment_distance(c, a, b), point_segment_distance(d, a, b)});
}

// Distance between two parts (0 if they overlap): boundary distance unless one
// contains a vertex of the other (interior overlap).
double part_distance(const Part& A, const Part& B, double gap_needed, const CancellationToken* cancel=nullptr) {
    check_cancel(cancel);
    if (contains_part(A, B.front().front(),cancel) || contains_part(B, A.front().front(),cancel)) return 0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& ra : A) {
        const std::size_t n = ra.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = ra[i], b = ra[(i + 1) % n];
            const double ax0 = std::min(a.x, b.x) - gap_needed, ax1 = std::max(a.x, b.x) + gap_needed;
            const double ay0 = std::min(a.y, b.y) - gap_needed, ay1 = std::max(a.y, b.y) + gap_needed;
            for (const auto& rb : B) {
                const std::size_t m = rb.size();
                for (std::size_t j = 0; j < m; ++j) {
                    check_cancel(cancel);
                    const Vec2 c = rb[j], d = rb[(j + 1) % m];
                    if (std::max(c.x, d.x) < ax0 || std::min(c.x, d.x) > ax1 || std::max(c.y, d.y) < ay0 || std::min(c.y, d.y) > ay1) continue;
                    best = std::min(best, segment_distance(a, b, c, d));
                    if (best == 0) return 0;
                }
            }
        }
    }
    return best;
}

double polygon_distance(const Shape& A, const Shape& B, double gap_needed, const CancellationToken* cancel=nullptr) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& pa : A) for (const auto& pb : B) {
        best = std::min(best, part_distance(pa, pb, gap_needed,cancel));
        if (best == 0) return 0;
    }
    return best;
}

Shape shape_of(const std::vector<Polygon>& parts, double target_aspect, const CancellationToken* cancel=nullptr) {
    Shape shape;
    for (const auto& polygon : parts) {
        Part part;
        auto add = [&](const Ring& ring) {
            Ring out;
            std::size_t n = ring.size();
            if (n > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) --n;
            for (std::size_t i = 0; i < n; ++i) {
                check_cancel(cancel);out.push_back({ring[i].x * target_aspect, ring[i].y});
            }
            part.push_back(std::move(out));
        };
        add(polygon.outer);
        for (const auto& hole : polygon.holes) add(hole);
        shape.push_back(std::move(part));
    }
    return shape;
}

Part part_of(const Polygon& polygon) {
    Part part;
    auto add = [&](const Ring& ring) {
        Ring out(ring.begin(), ring.end());
        if (out.size() > 1 && out.front().x == out.back().x && out.front().y == out.back().y) out.pop_back();
        part.push_back(std::move(out));
    };
    add(polygon.outer);
    for (const auto& hole : polygon.holes) add(hole);
    return part;
}

// Containment, pairwise overlap and gap over shapes that are already in packing
// space. `shapes` holds `movable` island shapes followed by the obstacle shapes;
// `ids` is parallel to it. Shared by both public entry points so the two can never
// answer differently.
// `group_of` and `shelved` are parallel to `shapes` when given (empty = no such
// information): with `validate_overlaid_groups` only same-group pairs are
// checked, and shelved islands (laid out above the target on purpose) skip
// containment and only have to stay overlap-free.
constexpr std::size_t any_group = std::numeric_limits<std::size_t>::max();
ValidationReport check_shapes(const std::vector<Shape>& shapes, const std::vector<Box>& boxes,
                              const std::vector<std::uint64_t>& ids, std::size_t movable,
                              const PackTarget& target, double gap, double tolerance,
                              double target_aspect, const std::vector<PackTarget>& targets,
                              const std::vector<std::size_t>& group_of = {},
                              const std::vector<char>& shelved = {},
                              bool validate_overlaid_groups = false,
                              const CancellationToken* cancel=nullptr, double border=0,
                              const std::vector<std::optional<std::size_t>>& tiles={},
                              const std::vector<Obstacle>& obstacles={}) {
    if(tolerance<0)tolerance=validation_tolerance(target,targets);
    if(!std::isfinite(tolerance) || !std::isfinite(border) || border<0)
        throw std::invalid_argument("validation tolerance/border must be finite nonnegative");
    for(const auto& obstacle:obstacles)
        if(!std::isfinite(obstacle.extra_margin) || obstacle.extra_margin<0 ||
           !std::isfinite(gap+obstacle.extra_margin))
            throw std::invalid_argument("obstacle clearance must be finite nonnegative");
    ValidationReport report;
    report.tolerance=tolerance;
    auto note = [&](const std::string& m) { if (report.messages.size() < 12) report.messages.push_back(m); };
    auto blame = [&](std::size_t i) {
        if (i >= movable) return; // Obstacles are not islands and cannot be reverted.
        if (std::find(report.offenders.begin(), report.offenders.end(), ids[i]) == report.offenders.end())
            report.offenders.push_back(ids[i]);
    };
    const std::vector<PackTarget> boxes_to_check = targets.empty() ? std::vector<PackTarget>{target} : targets;
    for (std::size_t i = 0; i < movable; ++i) {
        check_cancel(cancel);
        if (!shelved.empty() && shelved[i]) continue; // Outside the target by design.
        const auto& b = boxes[i];
        double clearance=-std::numeric_limits<double>::infinity();
        const auto tile=tiles.empty()?std::optional<std::size_t>{}:tiles[i];
        if(tile && *tile>=boxes_to_check.size()) {
            ++report.invalid_geometry;report.ok=false;blame(i);
            note("island "+std::to_string(ids[i])+" has an unknown target index");
            continue;
        }
        const std::size_t begin=tile?*tile:0,end=tile?*tile+1:boxes_to_check.size();
        for (std::size_t k=begin;k<end;++k) {
            check_cancel(cancel);
            const auto& t=boxes_to_check[k];
            clearance=std::max(clearance,std::min({b.min_x-t.min_x*target_aspect,
                t.max_x*target_aspect-b.max_x,b.min_y-t.min_y,t.max_y-b.max_y}));
        }
        report.min_border=std::min(report.min_border,clearance);
        if (clearance < -tolerance) {
            ++report.outside_target; report.ok = false; blame(i);
            note("island " + std::to_string(ids[i]) + (tile?" outside its assigned target":" outside every target"));
        }
        if(clearance < border-tolerance) {
            ++report.border_violations;report.ok=false;blame(i);
            note("island " + std::to_string(ids[i]) + " violates border clearance");
        }
    }
    const double requested = std::max(0.0, gap);
    for (std::size_t i = 0; i < movable; ++i) {
        for (std::size_t j = i + 1; j < shapes.size(); ++j) {
            check_cancel(cancel);
            // Overlaid groups: every group is packed over the whole target on its
            // own, so only same-group pairs are meant to be disjoint. Obstacle
            // identity comes from its position, not a reserved user group ID.
            if (validate_overlaid_groups && !group_of.empty() && j<movable &&
                group_of[i] != group_of[j]) continue;
            // Shelved islands only have to not overlap: they sit outside the target on a
            // fixed 0.05 grid, which owes nothing to the requested island gap.
            const double extra=j>=movable && !obstacles.empty()?obstacles[j-movable].extra_margin:0;
            const double needed = (!shelved.empty() && (shelved[i] || shelved[j])) ? 0.0 : requested+extra;
            const auto pair=[&] { return "island "+std::to_string(ids[i])+" and "+
                (j<movable?"island "+std::to_string(ids[j]):"obstacle "+std::to_string(j-movable)); };
            const auto& a = boxes[i]; const auto& b = boxes[j];
            if (a.max_x + needed < b.min_x || b.max_x + needed < a.min_x || a.max_y + needed < b.min_y || b.max_y + needed < a.min_y) continue;
            const double d = polygon_distance(shapes[i], shapes[j], needed,cancel);
            report.min_gap = std::min(report.min_gap, d);
            if (d <= tolerance && needed >= 0) {
                bool collision=d==0;
                if(collision && needed==0) {
                    collision=false;
                    // Boundary-only boxes cannot share area. Ambiguous contacts
                    // need an interior predicate, not a clearance tolerance.
                    if(a.max_x>b.min_x && b.max_x>a.min_x && a.max_y>b.min_y && b.max_y>a.min_y) {
                        for(const auto& left:shapes[i]) {
                            for(const auto& right:shapes[j]) {
                                const auto relation=detail::part_interiors_overlap(left,right,cancel);
                                if(relation==detail::InteriorRelation::Cancelled)throw ValidationCancelled{};
                                if(relation==detail::InteriorRelation::Overlap){collision=true;break;}
                            }
                            if(collision)break;
                        }
                    }
                }
                if(collision){++report.overlaps;report.ok=false;blame(i);blame(j);note(pair()+(needed==0?" interior overlap":" overlap or touch"));}
            }
            if (needed > 0 && d < needed - tolerance) {
                ++report.gap_violations; report.ok = false; blame(i); blame(j);
                note(pair() + " gap " + std::to_string(d) + " < " + std::to_string(needed));
            }
        }
    }
    return report;
}

// Obstacles take part in the pairwise checks but not in containment.
void append_obstacles(const std::vector<Obstacle>& obstacles, double target_aspect, std::vector<Shape>& shapes,
                      std::vector<Box>& boxes, std::vector<std::uint64_t>& ids) {
    for (const auto& obstacle : obstacles) {
        Part part = part_of(obstacle.polygon);
        for (auto& ring : part) for (auto& v : ring) v.x *= target_aspect;
        shapes.push_back(Shape{std::move(part)});
        boxes.push_back(bounds(shapes.back()));
        ids.push_back(std::numeric_limits<std::uint64_t>::max());
    }
}
} // namespace

double validation_tolerance(const PackTarget& target,const std::vector<PackTarget>& targets,
                            const std::string& output_precision) {
    const bool single=output_precision=="float32";
    if(!single && output_precision!="float64")throw std::invalid_argument("unknown output_precision");
    double magnitude=1;
    const auto include=[&](const PackTarget& t) {
        if(!std::isfinite(t.min_x)||!std::isfinite(t.min_y)||!std::isfinite(t.max_x)||!std::isfinite(t.max_y)
           ||t.min_x>=t.max_x||t.min_y>=t.max_y)throw std::invalid_argument("invalid target rectangle");
        magnitude=std::max({magnitude,std::abs(t.min_x),std::abs(t.max_x),std::abs(t.min_y),std::abs(t.max_y)});
    };
    if(targets.empty())include(target);else for(const auto& t:targets)include(t);
    return 8*(single?std::numeric_limits<float>::epsilon():std::numeric_limits<double>::epsilon())*magnitude;
}

bool polygons_overlap(const Polygon& a, const Polygon& b) {
    return part_distance(part_of(a), part_of(b), 0.0) == 0.0;
}

ValidationReport validate_layout(const std::vector<Island>& islands, const Layout& layout,
                                 const PackTarget& target, double gap, double tolerance,
                                 const std::vector<Obstacle>& obstacles, double target_aspect,
                                 const std::vector<PackTarget>& targets,
                                 bool validate_overlaid_groups, double border) {
    // Problems found while building the shapes are reported before the geometric
    // ones, which is the order callers that print messages.front() expect.
    std::vector<std::string> pre;
    bool pre_ok = true;
    if (!layout.valid && layout.placements.empty()) {
        ValidationReport report;
        report.ok = false;
        report.messages.push_back("layout marked invalid: " + layout.failure);
        return report;
    }
    if (!layout.valid) pre.push_back("partial layout: " + layout.failure);
    std::map<std::uint64_t,const Island*> source;
    std::set<std::uint64_t> seen,unplaced,identity_offenders;
    auto identity_error=[&](std::uint64_t id,const char* reason) {
        pre_ok=false;identity_offenders.insert(id);
        if(pre.size()<12)pre.push_back(std::string(reason)+std::to_string(id));
    };
    for(const auto& island:islands)
        if(!source.emplace(island.id,&island).second)identity_error(island.id,"duplicate source island ");
    for(const auto id:layout.unplaced) {
        if(!source.count(id))identity_error(id,"unknown unplaced island ");
        else if(!unplaced.insert(id).second)identity_error(id,"duplicate unplaced island ");
    }
    std::vector<Shape> shapes;
    std::vector<Box> boxes;
    std::vector<std::uint64_t> ids;
    std::vector<std::size_t> group_of;  // Per shape; obstacles use the any_group sentinel.
    std::vector<char> shelved;         // Per shape; laid out above the target on purpose.
    std::vector<std::optional<std::size_t>> tiles;
    std::size_t arranged_count = 0;
    for (const auto& placement : layout.placements) {
        const auto it=source.find(placement.id);
        if(it==source.end()){identity_error(placement.id,"placement for unknown island ");continue;}
        if(!seen.insert(placement.id).second){identity_error(placement.id,"duplicate placement for island ");continue;}
        if(unplaced.count(placement.id) && !placement.shelved)identity_error(placement.id,"island both placed and unplaced ");
        shapes.push_back(transformed_shape(*it->second, placement, layout.scale, target, target_aspect));
        boxes.push_back(bounds(shapes.back()));
        ids.push_back(placement.id);
        group_of.push_back(it->second->group);
        shelved.push_back(placement.shelved ? 1 : 0);
        tiles.push_back(placement.tile);
        if (placement.shelved) ++arranged_count;
    }
    const std::size_t movable = shapes.size();
    append_obstacles(obstacles, target_aspect, shapes, boxes, ids);
    group_of.resize(shapes.size(), any_group);
    shelved.resize(shapes.size(), 0);
    const std::size_t in_target = movable - arranged_count;
    for(const auto& [id,island]:source) {
        (void)island;
        if(!seen.count(id) && !unplaced.count(id))identity_error(id,"missing original island ");
    }
    if (in_target != source.size() - unplaced.size()) {
        pre_ok = false;
        pre.push_back("placed " + std::to_string(in_target) + " of " + std::to_string(islands.size()) + " islands");
    }
    auto report = check_shapes(shapes, boxes, ids, movable, target, gap, tolerance, target_aspect, targets,
                               group_of, shelved, validate_overlaid_groups,nullptr,border,tiles,obstacles);
    report.ok = report.ok && pre_ok;
    report.invalid_geometry+=identity_offenders.size();
    for(const auto id:identity_offenders)
        if(std::find(report.offenders.begin(),report.offenders.end(),id)==report.offenders.end())report.offenders.push_back(id);
    report.messages.insert(report.messages.begin(), pre.begin(), pre.end());
    if (report.messages.size() > 12) report.messages.resize(12);
    return report;
}

static ValidationReport validate_polygons_impl(const std::vector<PlacedIsland>& placed, const PackTarget& target,
                                   double gap, double tolerance, const std::vector<Obstacle>& obstacles,
                                   double target_aspect, const std::vector<PackTarget>& targets,
                                   const CancellationToken* cancel,double border,
                                   std::vector<std::size_t> groups={},std::vector<char> shelved={},bool overlays=false) {
    std::vector<Shape> shapes;
    std::vector<Box> boxes;
    std::vector<std::uint64_t> ids;
    std::vector<std::optional<std::size_t>> tiles;
    for (const auto& island : placed) {
        check_cancel(cancel);
        shapes.push_back(shape_of(island.parts, target_aspect,cancel));
        boxes.push_back(bounds(shapes.back(),cancel));
        ids.push_back(island.id);
        tiles.push_back(island.tile);
    }
    const std::size_t movable = shapes.size();
    append_obstacles(obstacles, target_aspect, shapes, boxes, ids);
    if(!groups.empty())groups.resize(shapes.size(),any_group);
    if(!shelved.empty())shelved.resize(shapes.size(),0);
    return check_shapes(shapes, boxes, ids, movable, target, gap, tolerance, target_aspect, targets,
                        groups,shelved,overlays,cancel,border,tiles,obstacles);
}

ValidationReport validate_polygons(const std::vector<PlacedIsland>& placed, const PackTarget& target,
                                   double gap, double tolerance, const std::vector<Obstacle>& obstacles,
                                   double target_aspect, const std::vector<PackTarget>& targets,double border,bool overlays) {
    std::vector<std::size_t> groups;std::vector<char> shelved;
    for(const auto& item:placed){groups.push_back(item.group);shelved.push_back(item.shelved);}
    return validate_polygons_impl(placed,target,gap,tolerance,obstacles,target_aspect,targets,nullptr,border,groups,shelved,overlays);
}

static ValidationReport validate_stored_impl(const std::vector<PlacedIsland>& input,
                                         const PackTarget& target, const ValidationOptions& options,
                                         const std::vector<std::size_t>& groups={},
                                         const std::vector<char>& shelved={},bool overlays=false) {
    const auto* cancel=&options.cancel;
    check_cancel(cancel);
    const bool single=options.output_precision=="float32";
    const double epsilon=checked_tolerance(target,options);
    auto targets=options.targets.empty()?std::vector<PackTarget>{target}:options.targets;
    check_cancel(cancel);
    auto placed=input;
    auto obstacles=options.obstacles;
    ValidationReport report;report.tolerance=epsilon;
    std::set<std::uint64_t> ids;
    auto invalid=[&](std::uint64_t id,const std::string& why,bool obstacle=false) {
        report.ok=false;++report.invalid_geometry;
        if(!obstacle && std::find(report.offenders.begin(),report.offenders.end(),id)==report.offenders.end())
            report.offenders.push_back(id);
        if(report.messages.size()<12) report.messages.push_back(std::string(obstacle?"obstacle ":"island ")+std::to_string(id)+": "+why);
    };
    auto ring_ok=[&](Ring& ring) {
        for(auto& p:ring) {
            check_cancel(cancel);
            if(!std::isfinite(p.x)||!std::isfinite(p.y)) return false;
            if(single) {
                if(std::abs(p.x)>std::numeric_limits<float>::max() || std::abs(p.y)>std::numeric_limits<float>::max()) return false;
                // GCC 12.2 vectorization can elide narrowing/widening casts here.
                // Force binary32 storage so certification sees the host's UVs.
                volatile float stored_x=static_cast<float>(p.x);
                volatile float stored_y=static_cast<float>(p.y);
                p.x=stored_x;p.y=stored_y;
            }
        }
        if(ring.size()>1 && ring.front().x==ring.back().x && ring.front().y==ring.back().y) ring.pop_back();
        if(ring.size()<3) return false;
        long double twice=0;
        const auto origin=ring.front();
        for(std::size_t i=0;i<ring.size();++i) {
            auto a=ring[i],b=ring[(i+1)%ring.size()];
            if(a.x==b.x && a.y==b.y) return false;
            twice+=((long double)a.x-origin.x)*((long double)b.y-origin.y)-((long double)b.x-origin.x)*((long double)a.y-origin.y);
            for(std::size_t j=i+2;j<ring.size();++j) {
                check_cancel(cancel);
                if(i==0 && j==ring.size()-1) continue;
                if(segment_distance(a,b,ring[j],ring[(j+1)%ring.size()])==0) return false;
            }
        }
        return std::isfinite(twice) && twice!=0;
    };
    auto check_polygon=[&](Polygon& part,std::uint64_t id,bool obstacle=false) {
            if(!ring_ok(part.outer)) {invalid(id,"invalid outer ring at storage precision",obstacle);return;}
            std::vector<Ring> previous;
            for(auto& hole:part.holes) {
                if(!ring_ok(hole)) {invalid(id,"invalid hole at storage precision",obstacle);continue;}
                bool valid=contains_part(Part{part.outer},hole.front(),cancel);
                for(std::size_t a=0;a<hole.size();++a)
                    for(std::size_t b=0;b<part.outer.size();++b) {
                        check_cancel(cancel);
                        if(segment_distance(hole[a],hole[(a+1)%hole.size()],part.outer[b],part.outer[(b+1)%part.outer.size()])==0) valid=false;
                    }
                for(const auto& other:previous) if(part_distance(Part{hole},Part{other},0,cancel)==0) valid=false;
                if(!valid) invalid(id,"hole crosses or lies outside its outer ring, or overlaps another hole",obstacle);
                previous.push_back(hole);
            }
    };
    for(auto& island:placed) {
        check_cancel(cancel);
        if(!ids.insert(island.id).second) {invalid(island.id,"duplicate ID");continue;}
        if(island.parts.empty()) {invalid(island.id,"no polygon parts");continue;}
        for(auto& part:island.parts)check_polygon(part,island.id);
    }
    for(std::size_t k=0;k<obstacles.size();++k) {
        check_cancel(cancel);
        check_polygon(obstacles[k].polygon,k,true);
    }
    if(!report.ok) return report; // Never pass malformed rings to geometry loops.
    report=validate_polygons_impl(placed,target,options.gap,epsilon,obstacles,options.target_aspect,targets,
                                 cancel,options.border,groups,shelved,overlays);
    report.tolerance=epsilon;
    std::vector<Shape> shapes;
    for(const auto& island:placed) {
        check_cancel(cancel);
        shapes.push_back(shape_of(island.parts,options.target_aspect,cancel));
    }
    const auto movable=shapes.size();
    for(const auto& obstacle:obstacles) {
        check_cancel(cancel);
        shapes.push_back(shape_of({obstacle.polygon},options.target_aspect,cancel));
    }
    // Measured minimum, including pairs beyond the requested-gap broad phase.
    for(std::size_t i=0;i<movable;++i) for(std::size_t j=i+1;j<shapes.size();++j) {
        check_cancel(cancel);
        if(overlays && j<movable && groups[i]!=groups[j])continue;
        report.min_gap=std::min(report.min_gap,polygon_distance(shapes[i],shapes[j],std::numeric_limits<double>::infinity(),cancel));
    }
    check_cancel(cancel);
    return report;
}

namespace {
ValidationReport cancelled_report() {
    ValidationReport report;report.ok=false;report.cancelled=true;
    report.messages.push_back("validation cancelled; no geometry certificate produced");
    return report;
}
}

ValidationReport validate_stored_polygons(const std::vector<PlacedIsland>& input,
                                         const PackTarget& target, const ValidationOptions& options) {
    try {
        std::vector<std::size_t> groups;std::vector<char> shelved;
        for(const auto& item:input){check_cancel(&options.cancel);groups.push_back(item.group);shelved.push_back(item.shelved);}
        return validate_stored_impl(input,target,options,groups,shelved,options.overlaid_groups);
    } catch(const ValidationCancelled&) {
        return cancelled_report();
    }
}

ValidationReport validate_stored_layout(const std::vector<Island>& islands,const Layout& layout,
                                       const PackTarget& target,const ValidationOptions& options) {
    try {
        const auto* cancel=&options.cancel;
        check_cancel(cancel);
        ValidationReport identity;
        identity.tolerance=checked_tolerance(target,options);
        auto invalid=[&](std::uint64_t id,const char* reason) {
            identity.ok=false;++identity.invalid_geometry;
            if(std::find(identity.offenders.begin(),identity.offenders.end(),id)==identity.offenders.end())
                identity.offenders.push_back(id);
            if(identity.messages.size()<12)identity.messages.push_back(std::string(reason)+std::to_string(id));
        };
        if(!layout.valid) {
            identity.ok=false;identity.messages.push_back("layout marked invalid: "+layout.failure);
            return identity;
        }
        std::map<std::uint64_t,const Island*> source;
        std::set<std::uint64_t> placed_ids,unplaced;
        for(const auto& island:islands) {
            check_cancel(cancel);
            if(!source.emplace(island.id,&island).second)invalid(island.id,"duplicate source island ");
        }
        for(const auto id:layout.unplaced) {
            check_cancel(cancel);
            if(!source.count(id))invalid(id,"unknown unplaced island ");
            if(!unplaced.insert(id).second)invalid(id,"duplicate unplaced island ");
        }
        for(const auto& placement:layout.placements) {
            check_cancel(cancel);
            if(!source.count(placement.id))invalid(placement.id,"placement for unknown island ");
            if(!placed_ids.insert(placement.id).second)invalid(placement.id,"duplicate placement for island ");
            if(placement.shelved && !unplaced.count(placement.id))invalid(placement.id,"shelved island missing from unplaced ");
            if(!placement.shelved && unplaced.count(placement.id))invalid(placement.id,"island both placed and unplaced ");
        }
        for(const auto& [id,island]:source) {
            check_cancel(cancel);(void)island;
            if(!placed_ids.count(id) && !unplaced.count(id))invalid(id,"missing original island ");
        }
        if(!identity.ok)return identity;
        std::vector<PlacedIsland> placed;
        std::vector<std::size_t> groups;
        std::vector<char> shelved;
        placed.reserve(layout.placements.size());
        for(const auto& placement:layout.placements) {
            check_cancel(cancel);
            const auto& island=*source.at(placement.id);
            const auto t=placement.transform(layout.scale,target,options.target_aspect);
            auto map_ring=[&](const Ring& input) {
                Ring output;output.reserve(input.size());
                for(const auto p:input) {
                    check_cancel(cancel);
                    output.push_back({t[0]*p.x+t[1]*p.y+t[2],t[3]*p.x+t[4]*p.y+t[5]});
                }
                return output;
            };
            auto map_polygon=[&](const Polygon& input) {
                Polygon output;output.outer=map_ring(input.outer);
                for(const auto& hole:input.holes)output.holes.push_back(map_ring(hole));
                return output;
            };
            PlacedIsland item;item.id=island.id;item.tile=placement.tile;
            item.parts.push_back(map_polygon(island.polygon));
            for(const auto& part:island.extra_parts)item.parts.push_back(map_polygon(part));
            placed.push_back(std::move(item));groups.push_back(island.group);shelved.push_back(placement.shelved);
        }
        return validate_stored_impl(placed,target,options,groups,shelved,layout.overlaid_groups);
    } catch(const ValidationCancelled&) {
        return cancelled_report();
    }
}
} // namespace uvpack
