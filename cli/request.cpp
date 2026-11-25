// JSON in, a packable request out: islands and options, the input repair, and the
// bind-overlapping pre-pass that merges overlapping islands into rigid ones.
#include "cli.hpp"
#include "checked_input.hpp"
#include "uvpack/geometry.hpp"
#include "uvpack/input_check.hpp"
#include "uvpack/validator.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>

namespace uvpack::cli {
namespace {

static Ring read_ring(const json::Value& v) {
Ring ring;
for (const auto& p : v.as_array()) {
    const auto& xy = p.as_array();
        if (xy.size() != 2) throw std::runtime_error("vertex must be [x,y]");
        ring.push_back({xy[0].as_number(), xy[1].as_number()});
    }
    return ring;
}

} // namespace

Request parse_request(const json::Value& doc) {
    input::validate(doc);
    Request request;
    if(auto o=doc.find("options")) {
        if(auto mode=o->find("density_mode")) request.density_mode=mode->as_string();
    }
    if(request.density_mode!="per_island" && request.density_mode!="uniform_layout")
        throw std::runtime_error("density_mode must be per_island or uniform_layout");
    const bool uniform=request.density_mode=="uniform_layout";
    request.require_complete=uniform;
    PackTarget target;
    if (auto t = doc.find("target")) {
        const auto& a = t->as_array();
        if (a.size() != 4) throw std::runtime_error("target must be [minx,miny,maxx,maxy]");
        target = {a[0].as_number(), a[1].as_number(), a[2].as_number(), a[3].as_number()};
    }
    std::vector<PackTarget> targets;
    if (auto ts = doc.find("targets")) {
        for (const auto& t : ts->as_array()) {
            const auto& a = t.as_array();
            if (a.size() != 4) throw std::runtime_error("each target must be [minx,miny,maxx,maxy]");
            targets.push_back({a[0].as_number(), a[1].as_number(), a[2].as_number(), a[3].as_number()});
        }
        if (!targets.empty()) target = targets[0];
    }
    std::vector<Island> islands;
    // Per-island inputs that are not part of Island itself: the 3D areas the texel
    // density is measured against and the normalization controls. Kept aligned with
    // `islands` through the repair and lock-merge passes below.
    struct IslandExtras {
        std::optional<double> original_uv, cross_sum, cross_global;
        bool local_known=false, global_known=false;
        double area_3d = 0;          // Normalize Space LOCAL: object-space 3D area.
        double area_3d_global = 0;   // Normalize Space GLOBAL: world-space 3D area.
        long long density_group = -1;   // Normalize Groups: < 0 = no group.
        double scale_multiplier = 1; // Island Scale Multiplier (1 = none).
        long long bind_group = 0;
        bool has_cluster = false;    // Texel Density Policy CUSTOM.
        long long scale_cluster = 0;
    };
    std::vector<IslandExtras> extras;
    std::uint64_t next_id = 0;
    const auto* requested = doc.find("islands");
    if (!requested) throw std::runtime_error("islands is required");
    std::set<std::uint64_t> original_ids;
    for (const auto& item : requested->as_array()) {
        Island island;
        const auto* id_value=item.find("id");
        const double id=id_value?id_value->as_number():double(next_id);
        if(!std::isfinite(id) || id<0 || id>9007199254740991.0 || std::floor(id)!=id)
            throw std::runtime_error("island ID must be an exactly representable nonnegative integer");
        island.id = static_cast<std::uint64_t>(id);
        if(!original_ids.insert(island.id).second) throw std::runtime_error("duplicate island ID");
        request.requested_ids.push_back(island.id);
        next_id = island.id + 1;
        island.group = 0;
        if(auto outer=item.find("outer")) island.polygon.outer = read_ring(*outer);
        else if(!item.find("envelope_points")) throw std::runtime_error("island needs outer or envelope_points");
        if(auto points=item.find("envelope_points")) island.envelope_points=read_ring(*points);
        if (auto holes = item.find("holes"))
            for (const auto& h : holes->as_array()) island.polygon.holes.push_back(read_ring(h));
        island.scale_locked = item.bool_or("scale_locked", false);
        island.rotation_step = input::integer_or<int>(item, "rotation_step", -1);
        if (auto v = item.find("rotation")) island.rotation_mode = v->as_bool() ? 1 : 0;
        if (auto v = item.find("flip")) island.flip_mode = v->as_bool() ? 1 : 0;
        island.pre_scale = item.number_or("pre_scale", 1.0);
        island.group = input::integer_or<std::size_t>(item, "group", 0);
        if (auto parts = item.find("parts"))
            for (const auto& part : parts->as_array()) {
                Polygon polygon;
                const auto* outer=part.find("outer");
                if(!outer) throw std::runtime_error("part needs outer");
                polygon.outer = read_ring(*outer);
                if (auto holes = part.find("holes"))
                    for (const auto& h : holes->as_array()) polygon.holes.push_back(read_ring(h));
                island.extra_parts.push_back(std::move(polygon));
            }
        IslandExtras extra;
        if(auto value=item.find("area_uv")) extra.original_uv=value->as_number();
        if(auto value=item.find("density_cross_sum")) extra.cross_sum=value->as_number();
        if(auto value=item.find("density_cross_sum_global")) extra.cross_global=value->as_number();
        extra.local_known=item.find("area_3d")!=nullptr;
        extra.global_known=item.find("area_3d_global")!=nullptr;
        extra.bind_group = input::integer_or<long long>(item, "bind_group", 0);
        extra.area_3d = item.number_or("area_3d", 0.0);
        extra.area_3d_global = item.number_or("area_3d_global", 0.0);
        extra.density_group = input::integer_or<long long>(item, "density_group", -1);
        extra.scale_multiplier = item.number_or("scale_multiplier", 1.0);
        if (auto v = item.find("scale_cluster")) { extra.has_cluster = true; extra.scale_cluster = input::integer<long long>(*v,"scale_cluster"); }
        if(uniform && (island.scale_locked || extra.scale_multiplier!=1))
            throw std::runtime_error("uniform density conflicts with scale locks or multipliers");
        if(extra.original_uv && (!std::isfinite(*extra.original_uv)||*extra.original_uv<0))
            throw std::runtime_error("area_uv must be finite and nonnegative");
        extras.push_back(extra);
        islands.push_back(std::move(island));
    }
    // Preserve input footprints for final emitted-geometry certification and for
    // positional per-vertex snapping, before repair or rigid binding changes them.
    std::map<std::uint64_t, Island> source_islands;
    for(const auto& island:islands)source_islands[island.id]=island;
    bool per_vertex_snap=false;
    if (const auto* o = doc.find("options")) {
        const std::string mode = o->string_or("snap_mode", "bbox_min");
        per_vertex_snap=o->bool_or("snap_to_texels", false) && (mode == "outline" || mode == "all");
    }
    // Input validation and repair (before locking, so rejected islands never merge).
    json::Object input_report;
    {
        InputPolicy policy;
        if (auto o = doc.find("options")) {
            if(auto v=o->find("strict_input")) policy.strict=v->as_bool();
            if(auto v=o->find("hull_policy")) policy.hull_policy=v->as_string();
            if(auto v=o->find("require_complete")) {
                if(uniform && !v->as_bool()) throw std::runtime_error("uniform density requires complete input");
                request.require_complete=v->as_bool();
            }
        }
        std::vector<Island> checked = islands;
        const auto rep = check_and_repair(checked, policy);
        std::set<std::uint64_t> kept_ids;
        for (const auto& i : checked) kept_ids.insert(i.id);
        std::vector<IslandExtras> kept;
        for (std::size_t i = 0; i < islands.size(); ++i)
            if (kept_ids.count(islands[i].id)) kept.push_back(extras[i]);
        islands = std::move(checked); extras = std::move(kept);
        json::Array issues;
        for (const auto& issue : rep.issues) {
            json::Object it; it["island"] = double(issue.island); it["message"] = issue.message; it["fatal"] = issue.fatal; it["reason"] = issue.reason; issues.push_back(it);
        }
        json::Array rejected_ids,hull_ids;
        for(auto id:rep.rejected_ids) rejected_ids.push_back(double(id));
        for(auto id:rep.hull_ids) hull_ids.push_back(double(id));
        input_report["rejected_ids"]=rejected_ids;
        input_report["hull_ids"]=hull_ids;
        input_report["complete"]=rep.complete;
        if(!rep.hull_ids.empty() && per_vertex_snap)
            throw std::runtime_error("hull recovery with per-vertex snapping is unsupported; use rigid snapping or disable snapping");
        // Envelope-only callers explicitly supplied no original ring. Their
        // approved hull is the footprint; a supplied invalid ring is never hidden.
        for(const auto& island:islands) {
            auto& source=source_islands.at(island.id);
            if(source.polygon.outer.empty() && !source.envelope_points.empty())source=island;
        }
        input_report["issues"] = issues; input_report["repaired"] = rep.repaired; input_report["rejected"] = rep.rejected;
    }
    // Binding: overlapping islands, and islands sharing a bind group, are combined
    // into rigid multi-part islands.
    std::map<std::uint64_t, std::vector<std::uint64_t>> members; // leader id -> member ids
    {
        std::string mode = "off";
        if (auto o = doc.find("options")) mode = o->string_or("bind_overlapping", "off");
        if (mode != "off" && mode != "touching" && mode != "identical")
            throw std::runtime_error("bind_overlapping must be off, touching or identical");
        const bool any_bind_groups = std::any_of(extras.begin(), extras.end(), [](const IslandExtras& e) { return e.bind_group != 0; });
        if (mode != "off" || any_bind_groups) {
            std::vector<std::size_t> parent(islands.size());
            for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
            std::function<std::size_t(std::size_t)> find = [&](std::size_t i) { return parent[i] == i ? i : parent[i] = find(parent[i]); };
            auto unite = [&](std::size_t a, std::size_t b) { parent[find(a)] = find(b); };
            auto box_of = [](const Polygon& poly) {
                std::array<double, 4> b{1e300, 1e300, -1e300, -1e300};
                for (const auto p : poly.outer) { b[0] = std::min(b[0], p.x); b[1] = std::min(b[1], p.y); b[2] = std::max(b[2], p.x); b[3] = std::max(b[3], p.y); }
                return b;
            };
            std::vector<std::array<double, 4>> boxes;
            for (const auto& island : islands) boxes.push_back(box_of(island.polygon));
            for (std::size_t i = 0; i < islands.size(); ++i) {
                for (std::size_t j = i + 1; j < islands.size(); ++j) {
                    if (extras[i].bind_group != 0 && extras[i].bind_group == extras[j].bind_group) { unite(i, j); continue; }
                    if (mode == "off") continue;
                    const auto& a = boxes[i]; const auto& b = boxes[j];
                    if (a[2] < b[0] || b[2] < a[0] || a[3] < b[1] || b[3] < a[1]) continue;
                    if (mode == "identical") {
                        const double tol = 1e-6;
                        const bool same_box = std::abs(a[0] - b[0]) < tol && std::abs(a[1] - b[1]) < tol && std::abs(a[2] - b[2]) < tol && std::abs(a[3] - b[3]) < tol;
                        const double area_a = measure_polygon(islands[i].polygon).area, area_b = measure_polygon(islands[j].polygon).area;
                        if (same_box && std::abs(area_a - area_b) < tol * std::max(1.0, area_a)) unite(i, j);
                    } else if (polygons_overlap(islands[i].polygon, islands[j].polygon)) {
                        unite(i, j);
                    }
                }
            }
            std::vector<Island> merged;
            std::map<std::size_t, std::size_t> leader_index; // root -> index in merged
            for (std::size_t i = 0; i < islands.size(); ++i) {
                const auto root = find(i);
                auto it = leader_index.find(root);
                if (it == leader_index.end()) {
                    leader_index[root] = merged.size();
                    merged.push_back(islands[i]);
                    members[islands[i].id] = {islands[i].id};
                } else {
                    auto& leader = merged[it->second];
                    if(uniform && leader.pre_scale!=islands[i].pre_scale)
                        throw std::runtime_error("bound uniform-density islands must share pre_scale");
                    leader.extra_parts.push_back(islands[i].polygon);
                    for (const auto& part : islands[i].extra_parts) leader.extra_parts.push_back(part);
                    members[leader.id].push_back(islands[i].id);
                }
            }
            if (merged.size() != islands.size()) {
                // A rigid multi-part island covers the 3D area of all its members;
                // the normalization controls come from the leader.
                std::vector<IslandExtras> merged_extras;
                for (const auto& m : merged) {
                    IslandExtras sum;
                    sum.original_uv=0;sum.cross_sum=0;sum.cross_global=0;
                    sum.local_known=true;sum.global_known=true;
                    auto add=[](std::optional<double>& a,const std::optional<double>& b) {
                        if(a && b) *a+=*b; else a.reset();
                    };
                    bool first = true;
                    for (std::size_t i = 0; i < islands.size(); ++i) {
                        if (std::find(members[m.id].begin(), members[m.id].end(), islands[i].id) == members[m.id].end()) continue;
                        add(sum.original_uv,extras[i].original_uv);
                        add(sum.cross_sum,extras[i].cross_sum);
                        add(sum.cross_global,extras[i].cross_global);
                        sum.local_known=sum.local_known && extras[i].local_known;
                        sum.global_known=sum.global_known && extras[i].global_known;
                        sum.area_3d += extras[i].area_3d;
                        sum.area_3d_global += extras[i].area_3d_global;
                        if (first) {
                            sum.density_group = extras[i].density_group;
                            sum.scale_multiplier = extras[i].scale_multiplier;
                            sum.has_cluster = extras[i].has_cluster;
                            sum.scale_cluster = extras[i].scale_cluster;
                            sum.bind_group = extras[i].bind_group;
                            first = false;
                        }
                    }
                    merged_extras.push_back(sum);
                }
                islands = std::move(merged);
                extras = std::move(merged_extras);
            }
        }
    }
    std::vector<Obstacle> obstacles;
    if (auto obs = doc.find("obstacles")) {
        for (const auto& item : obs->as_array()) {
            Obstacle o;
            o.polygon.outer = read_ring(input::required(item,"outer"));
            if (auto holes = item.find("holes"))
                for (const auto& h : holes->as_array()) o.polygon.holes.push_back(read_ring(h));
            o.extra_margin = item.number_or("extra_margin", 0.0);
            obstacles.push_back(std::move(o));
        }
    }
    PackOptions options;
    options.targets = targets;
    bool stream = false;          // One JSON line per improved layout, then a "done" line.
    double requested_margin = -1; // Set when texel snapping reserves extra gap.
    std::string scale_mode = "max";
    double fixed_scale = 1;
    // Texel density state, filled in by the options block and reported afterwards.
    std::string density_space = "local";
    std::vector<DensityIsland> density;
    double tex_size = 0, texel_density_request = 0;
    std::size_t texel_density_islands = 0;
    {
        // An absent options block has the same defaults as {}. Per-island
        // multipliers and clusters must not depend on that unrelated presence.
        const json::Value defaults(json::Object{});
        const auto* o=doc.find("options");
        if(!o)o=&defaults;
        options.output_precision=o->string_or("output_precision","float64");
        options.margin = o->number_or("margin", options.margin);
        options.border_margin = o->number_or("border_margin", options.border_margin);
        // Pixel margins: px / texture size, overriding the UV-unit margins when given.
        const double tex = o->number_or("tex_size", 0);
        if (tex > 0) {
            if (auto v = o->find("pixel_margin")) options.margin = v->as_number() / tex;
            if (auto v = o->find("pixel_border_margin")) options.border_margin = v->as_number() / tex;
        }
        request.requested_border=options.border_margin;
        options.allow_mirroring = o->bool_or("allow_mirroring", options.allow_mirroring);
        options.target_aspect = o->number_or("target_aspect", options.target_aspect);
        options.allow_partial = o->bool_or("allow_partial", options.allow_partial);
        options.progress = o->bool_or("progress", false);
        {
            const std::string m = o->string_or("snap_mode", "bbox_min");
            if (m == "bbox") options.snap_mode = SnapMode::BBox;
            else if (m == "outline") options.snap_mode = SnapMode::Outline;
            else if (m == "all") options.snap_mode = SnapMode::All;
            else if (m != "bbox_min") throw std::runtime_error("snap_mode must be bbox_min, bbox, outline or all");
            const std::string a = o->string_or("snap_target", "corner");
            if (a == "center" || a == "centre") options.snap_target = SnapTarget::Center;
            else if (a != "corner") throw std::runtime_error("snap_target must be corner or center");
        }
        if (o->bool_or("snap_to_texels", false) && o->number_or("tex_size", 0) > 0) {
            // Snapping moves each island by up to half a texel: reserve a texel of
            // gap and half a texel of border so the snapped layout still honors them.
            options.snap_tex_size = o->number_or("tex_size", 0);
            requested_margin = options.margin;
            options.margin += 1.0 / options.snap_tex_size;
            if (options.border_margin < 0) options.border_margin = requested_margin;
            options.border_margin += 0.5 / options.snap_tex_size;
        }
        {
            const auto tile_fill = o->string_or("tile_fill", "sequential");
            if (tile_fill == "sequential") options.tile_fill = TileFill::Sequential;
            else if (tile_fill == "any") options.tile_fill = TileFill::Any;
            else throw std::runtime_error("tile_fill must be sequential or any");
        }
        if (auto g = o->find("groups")) {
            for (const auto& [key, value] : g->as_object()) {
                std::vector<std::size_t> tiles;
                if (auto t = value.find("tiles")) for (const auto& v : t->as_array()) tiles.push_back(input::integer<std::size_t>(v,"tile index"));
                if (!tiles.empty()) options.group_tiles[input::group_key(key)] = tiles;
                // Texel Density Policy CUSTOM: the cluster this group belongs to.
                if (auto c = value.find("scale_cluster"))
                    options.scale_cluster[input::group_key(key)] = input::integer<long long>(*c,"scale_cluster");
            }
        }
        options.group_adjacent = o->bool_or("group_adjacent", options.group_adjacent);
        options.group_cohesion = o->number_or("group_cohesion", options.group_cohesion);
        options.overlay_groups = o->bool_or("overlay_groups", options.overlay_groups);
        options.shelve_unplaced = o->bool_or("shelve_unplaced", options.shelve_unplaced);
        options.stall_timeout = o->number_or("stall_timeout", options.stall_timeout);
        {
            const auto mode = o->string_or("fill", "free");
            if (mode == "free") options.fill = FillMode::Free;
            else if (mode == "width_first") options.fill = FillMode::WidthFirst;
            else if (mode == "height_first") options.fill = FillMode::HeightFirst;
            else if (mode == "proportional") options.fill = FillMode::Proportional;
            else throw std::runtime_error("unknown fill: " + mode);
            const auto corner = o->string_or("origin", "bottom_left");
            if (corner == "bottom_left") options.origin = Origin::BottomLeft;
            else if (corner == "bottom_right") options.origin = Origin::BottomRight;
            else if (corner == "top_right") options.origin = Origin::TopRight;
            else if (corner == "top_left") options.origin = Origin::TopLeft;
            else throw std::runtime_error("unknown origin: " + corner);
        }
        // --- Texel density controls (all of them only move Island::pre_scale). ---
        // Normalize Space: LOCAL reads `area_3d` (object-space areas), GLOBAL reads
        // `area_3d_global` (world-space areas, object scaling folded in by the
        // caller). It is purely which number the caller measured and sent; an island
        // with no area under the chosen space falls back to the other one, and with
        // neither it is left untouched by every control here.
        density_space = o->string_or("density_space", density_space);
        if (density_space != "local" && density_space != "global")
            throw std::runtime_error("density_space must be \"local\" or \"global\"");
        const bool global_space = density_space == "global";
        for (std::size_t i = 0; i < islands.size(); ++i) {
            DensityIsland d;
            if(uniform && (!extras[i].original_uv || !(global_space?extras[i].global_known:extras[i].local_known)))
                throw std::runtime_error("uniform density requires original UV and selected-space surface areas");
            d.uv_area = extras[i].original_uv.value_or(measure_island(islands[i]).area);
            d.area_3d = global_space ? (extras[i].area_3d_global > 0 ? extras[i].area_3d_global : extras[i].area_3d)
                                     : (extras[i].area_3d > 0 ? extras[i].area_3d : extras[i].area_3d_global);
            if(uniform) {
                d.area_3d=global_space?extras[i].area_3d_global:extras[i].area_3d;
                d.density_cross_sum=global_space?extras[i].cross_global:extras[i].cross_sum;
            }
            d.pre_scale = islands[i].pre_scale;
            d.density_group = extras[i].density_group;
            d.scale_multiplier = extras[i].scale_multiplier;
            density.push_back(d);
        }
        // Texture the densities are expressed on. sqrt(tex_w * tex_h) is the scalar
        // in the density formula, so a target aspect r means a tex_size * r wide page.
        tex_size = o->number_or("tex_size", 0);
        const double tex_w = tex_size * options.target_aspect, tex_h = tex_size;
        // Order: equalize_density -> set the absolute level (texel_density) ->
        // deliberate per-island departures (scale_multiplier).
        if(uniform && o->bool_or("equalize_density",false))
            throw std::runtime_error("uniform density cannot equalize relative island densities");
        if (o->bool_or("equalize_density", false)) equalize_density(density);
        texel_density_request = o->number_or("texel_density", 0);
        if(uniform) {
            if(input_report.at("complete").as_bool())
                request.density_factor=set_uniform_layout_density(density,texel_density_request,tex_w,tex_h);
            texel_density_islands=density.size();scale_mode="fixed";fixed_scale=1;
        } else if (texel_density_request > 0) {
            if (!(tex_size > 0)) throw std::runtime_error("texel_density needs a positive tex_size");
            texel_density_islands = set_texel_density(density, texel_density_request, tex_w, tex_h);
            // Pre-scaled to the requested density: pack at a fixed scale of 1 so the
            // density survives the pack. An explicit scale_mode still wins.
            if (!o->find("scale_mode")) { scale_mode = "fixed"; fixed_scale = 1.0; }
        }
        apply_scale_multipliers(density);
        for (std::size_t i = 0; i < islands.size(); ++i) islands[i].pre_scale = density[i].pre_scale;
        // Texel Density Policy: how many uniform scales the run may produce.
        {
            const auto policy = o->string_or("scale_policy", "shared");
            if (policy == "shared") options.scale_policy = ScalePolicy::Shared;
            else if (policy == "per_group") options.scale_policy = ScalePolicy::PerGroup;
            else if (policy == "custom") options.scale_policy = ScalePolicy::Custom;
            else if (policy == "by_tiles") options.scale_policy = ScalePolicy::ByTiles;
            else throw std::runtime_error("scale_policy must be shared, per_group, custom or by_tiles");
            // Per-island `scale_cluster` is a shorthand for the group's entry; an
            // explicit `groups` entry, parsed above, wins over it.
            for (std::size_t i = 0; i < islands.size(); ++i)
                if (extras[i].has_cluster) options.scale_cluster.emplace(islands[i].group, extras[i].scale_cluster);
        }
        options.shrink_spanning = o->bool_or("shrink_spanning", options.shrink_spanning);
        options.sub_rows = input::integer_or<std::size_t>(*o, "sub_rows", options.sub_rows);
        options.fast_search = o->bool_or("fast_search", options.fast_search);
        options.parallel_bracket = o->bool_or("parallel_bracket", options.parallel_bracket);
        options.extent_estimate = o->bool_or("extent_estimate", options.extent_estimate);
        options.single_pass = o->bool_or("single_pass", options.single_pass);
        options.single_pass_margin_reserve = o->number_or("single_pass_margin_reserve", options.single_pass_margin_reserve);
        options.single_pass_start = o->number_or("single_pass_start", options.single_pass_start);
        options.estimate_tolerance = o->number_or("estimate_tolerance", options.estimate_tolerance);
        options.bracket_tolerance = o->number_or("bracket_tolerance", options.bracket_tolerance);
        options.bracket_start = o->number_or("bracket_start", options.bracket_start);
        options.max_evaluations = input::integer_or<std::size_t>(*o, "max_evaluations", options.max_evaluations);
        options.stripe_height = o->number_or("stripe_height", options.stripe_height);
        options.allow_rotation = o->bool_or("allow_rotation", options.allow_rotation);
        options.prealign = o->bool_or("prealign", options.prealign);
        options.rotation_step = input::integer_or<int>(*o, "rotation_step", options.rotation_step);
        options.dedup_orientations = o->bool_or("dedup_orientations", options.dedup_orientations);
        options.trials = input::integer_or<std::size_t>(*o, "trials", options.trials);
        options.threads = input::integer_or<std::size_t>(*o, "threads", options.threads);
        options.gpu = o->bool_or("gpu", options.gpu);
        options.gpu_trials = input::integer_or<std::size_t>(*o, "gpu_trials", options.gpu_trials);
        options.gpu_in_round = o->bool_or("gpu_in_round", options.gpu_in_round);
        options.initial_step = o->number_or("initial_step", options.initial_step);
        options.seed = input::integer_or<std::uint32_t>(*o, "seed", options.seed);
        options.bracket_rounds = input::integer_or<std::size_t>(*o, "bracket_rounds", options.bracket_rounds);
        options.time_budget_seconds = o->number_or("time_budget", 0);
        options.compaction_time = o->number_or("compaction_time", options.compaction_time);
        options.compaction_rotation = o->bool_or("compaction_rotation", options.compaction_rotation);
        options.compaction_polish = o->bool_or("compaction_polish", options.compaction_polish);
        options.compaction_delta = o->number_or("compaction_delta", options.compaction_delta);
        options.compaction_threads = input::integer_or<std::size_t>(*o, "compaction_threads", options.compaction_threads);
        options.compaction_attempt_sweeps = input::integer_or<std::size_t>(*o, "compaction_attempt_sweeps", options.compaction_attempt_sweeps);
        options.compaction_max_rounds = input::integer_or<std::size_t>(*o, "compaction_max_rounds", options.compaction_max_rounds);
        scale_mode = o->string_or("scale_mode", scale_mode);
        fixed_scale = o->number_or("scale", fixed_scale);
        stream = o->bool_or("stream", false);
        if(uniform && (scale_mode!="fixed" || fixed_scale!=1 || options.allow_partial
                      || options.shrink_spanning || options.shelve_unplaced
                      || options.scale_policy!=ScalePolicy::Shared))
            throw std::runtime_error("uniform density requires fixed scale 1, shared scaling and complete placement");
        if(uniform && options.snap_tex_size>0 && options.snap_mode!=SnapMode::BBoxMin)
            throw std::runtime_error("uniform density permits only rigid bbox_min snapping");
    }
    request.self_check = !(doc.find("options") && doc.find("options")->bool_or("validate", true) == false);
    request.target = target;
    request.islands = std::move(islands);
    request.obstacles = std::move(obstacles);
    request.options = std::move(options);
    request.scale_mode = scale_mode;
    request.fixed_scale = fixed_scale;
    request.stream = stream;
    request.requested_margin = requested_margin;
    request.members = std::move(members);
    request.source_islands = std::move(source_islands);
    request.input_report = std::move(input_report);
    request.density = std::move(density);
    request.tex_size = tex_size;
    request.texel_density_request = texel_density_request;
    request.texel_density_islands = texel_density_islands;
    return request;
}

} // namespace uvpack::cli
