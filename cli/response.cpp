// What goes back out: the per-island transforms, the streamed "improved" events, the
// texel snapping applied to the finished layout, and the result object around them.
#include "cli.hpp"
#include "uvpack/snap.hpp"
#include "uvpack/validator.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <set>

namespace uvpack::cli {
namespace {
json::Object validation_json(const ValidationReport& report) {
    json::Array messages,offenders;
    for(const auto& message:report.messages)messages.push_back(message);
    for(auto id:report.offenders)offenders.push_back(double(id));
    return {{"ok",report.ok},{"cancelled",report.cancelled},{"overlaps",report.overlaps},
        {"outside_target",report.outside_target},{"gap_violations",report.gap_violations},
        {"border_violations",report.border_violations},{"invalid_geometry",report.invalid_geometry},
        {"tolerance",report.tolerance},
        {"min_gap",std::isfinite(report.min_gap)?report.min_gap:-1.0},
        {"min_border",std::isfinite(report.min_border)?json::Value(report.min_border):json::Value()},
        {"messages",messages},{"offenders",offenders}};
}

// Check exactly what a client will apply, retaining the original ID inventory.
// Bound members form one compound shape; overlap inside that rigid shape is intentional.
ValidationReport certify_emitted(const Request& request,const Layout& layout,const json::Array& emitted,
                                 double gap,double border,bool final_certificate=true) {
    ValidationReport identity;
    identity.tolerance=validation_tolerance(request.target,request.options.targets,request.options.output_precision);
    auto invalid=[&](std::uint64_t id,const char* reason) {
        identity.ok=false;++identity.invalid_geometry;identity.offenders.push_back(id);
        if(identity.messages.size()<12)identity.messages.push_back(std::string(reason)+std::to_string(id));
    };
    std::map<std::uint64_t,const Island*> owner;
    std::map<std::uint64_t,std::vector<std::uint64_t>> members;
    for(const auto& island:request.islands) {
        const auto bound=request.members.find(island.id);
        const auto ids=bound==request.members.end()?std::vector<std::uint64_t>{island.id}:bound->second;
        members[island.id]=ids;
        for(auto id:ids)if(!owner.emplace(id,&island).second)invalid(id,"duplicate bound member ");
    }
    std::map<std::uint64_t,PlacedIsland> compounds;
    std::set<std::uint64_t> seen;
    for(const auto& value:emitted) {
        const auto& item=value.as_object();
        const auto id=static_cast<std::uint64_t>(item.at("id").as_number());
        if(!seen.insert(id).second){invalid(id,"duplicate emitted island ");continue;}
        const auto own=owner.find(id);
        const auto source=request.source_islands.find(id);
        if(own==owner.end() || source==request.source_islands.end()){invalid(id,"unknown emitted original island ");continue;}
        const auto& leader=*own->second;
        const auto tile=static_cast<std::size_t>(item.at("tile").as_number());
        const bool shelved=value.bool_or("shelved",false);
        auto [position,fresh]=compounds.try_emplace(leader.id);
        auto& placed=position->second;
        if(fresh){placed.id=leader.id;placed.group=leader.group;placed.tile=tile;placed.shelved=shelved;}
        else if(placed.tile!=tile || placed.shelved!=shelved)invalid(id,"inconsistent bound placement ");
        const auto& transform=item.at("transform").as_array();
        auto map_ring=[&](const Ring& ring) {
            Ring out;out.reserve(ring.size());
            for(auto p:ring)out.push_back({transform[0].as_number()*p.x+transform[1].as_number()*p.y+transform[2].as_number(),
                transform[3].as_number()*p.x+transform[4].as_number()*p.y+transform[5].as_number()});
            return out;
        };
        auto map_polygon=[&](const Polygon& polygon) {
            Polygon out;out.outer=map_ring(polygon.outer);
            for(const auto& hole:polygon.holes)out.holes.push_back(map_ring(hole));
            return out;
        };
        if(const auto vertices=value.find("vertices")) {
            auto read_ring=[](const json::Value& ring) {
                Ring out;for(const auto& p:ring.as_array())out.push_back({p.as_array()[0].as_number(),p.as_array()[1].as_number()});
                return out;
            };
            auto read_polygon=[&](const json::Value& polygon) {
                Polygon out;out.outer=read_ring(*polygon.find("outer"));
                if(auto holes=polygon.find("holes"))for(const auto& hole:holes->as_array())out.holes.push_back(read_ring(hole));
                return out;
            };
            placed.parts.push_back(read_polygon(*vertices));
            if(auto parts=vertices->find("parts"))for(const auto& part:parts->as_array())placed.parts.push_back(read_polygon(part));
        } else {
            placed.parts.push_back(map_polygon(source->second.polygon));
            for(const auto& part:source->second.extra_parts)placed.parts.push_back(map_polygon(part));
        }
    }
    for(const auto& [id,island]:owner){(void)island;if(!seen.count(id))invalid(id,"missing emitted original island ");}
    if(!identity.ok)return identity;
    std::vector<PlacedIsland> placed;
    for(auto& [id,compound]:compounds){(void)id;placed.push_back(std::move(compound));}
    ValidationOptions policy;policy.gap=gap;policy.border=border;policy.target_aspect=request.options.target_aspect;
    policy.targets=request.options.targets;policy.output_precision=request.options.output_precision;
    policy.obstacles=request.obstacles;policy.overlaid_groups=layout.overlaid_groups;
    if(!final_certificate)policy.cancel=request.options.cancel;
    auto report=validate_stored_polygons(placed,request.target,policy);
    std::vector<std::uint64_t> original_offenders;
    for(auto leader:report.offenders)for(auto id:members.at(leader))original_offenders.push_back(id);
    report.offenders=std::move(original_offenders);
    return report;
}
}

json::Array islands_json(const Request& request, const Layout& l) {
    const auto& target = request.target;
    const auto& options = request.options;
    const auto& islands = request.islands;
    const auto& members = request.members;
    const auto& density = request.density;
    const double tex_size = request.tex_size;
    json::Array placed;
    for (const auto& p : l.placements) {
        std::vector<std::uint64_t> ids{p.id};
        if (auto it = members.find(p.id); it != members.end()) ids = it->second;
        const auto t = p.transform(l.scale, target, options.target_aspect);
        for (const auto id : ids) {
            json::Object item;
            item["id"] = double(id);
            if (id != p.id) item["bound_to"] = double(p.id);
            item["angle"] = p.angle;
            item["mirrored"] = p.mirrored;
            item["island_scale"] = p.island_scale;
            item["tile"] = p.tile;
            if (p.shelved) item["shelved"] = true;
            if (options.scale_policy != ScalePolicy::Shared) item["cluster"] = p.cluster;
            // Achieved texel density, TD = sqrt(tex_w * tex_h) * sqrt(A_uv / A_3d) with
            // A_uv the packed UV area. Only islands with a known 3D area have one; the
            // rest were never rescaled by the density controls and report none.
            if (tex_size > 0) {
                std::size_t index = 0;
                while (index < islands.size() && islands[index].id != p.id) ++index;
                if (index < islands.size() && density[index].area_3d > 0) {
                    const double packed_area = density[index].uv_area * p.island_scale * p.island_scale;
                    item["texel_density"] = texel_density(packed_area, density[index].area_3d,
                                                          tex_size * options.target_aspect, tex_size);
                }
            }
            json::Array tr;
            for (double v : t) tr.push_back(v);
            item["transform"] = tr;
            placed.push_back(item);
        }
    }
    return placed;
}

json::Object stream_event(const Request& request, const Layout& layout, double elapsed) {
    if(!layout.valid)return {};
    auto placed=islands_json(request,layout);
    const double gap=request.requested_margin>=0?request.requested_margin:request.options.margin;
    const double border=request.requested_border>=0?request.requested_border:gap;
    const auto report=certify_emitted(request,layout,placed,gap,border,false);
    if(!report.ok)return {};
    json::Object ev;
    ev["event"] = std::string("improved");
    ev["dialect"] = std::string(dialect);
    ev["scale"] = layout.scale;
    ev["elapsed"] = elapsed;
    ev["islands"] = std::move(placed);
    ev["output_precision"]=request.options.output_precision;
    return ev;
}

json::Object result_json(const Request& request, Layout& layout, const SearchStats& stats,
                         double achieved_margin) {
    const auto& target = request.target;
    const auto& options = request.options;
    const auto& islands = request.islands;
    const auto& obstacles = request.obstacles;
    const auto& members = request.members;
    const auto& source_islands = request.source_islands;
    const auto& input_report = request.input_report;
    const double requested_margin = request.requested_margin;
    const double texel_density_request = request.texel_density_request;
    const std::size_t texel_density_islands = request.texel_density_islands;
    const bool self_check = request.self_check;
    const Layout unsnapped=layout;
    const double tolerance=validation_tolerance(target,options.targets,options.output_precision);
    const double border=request.requested_border>=0?request.requested_border:
        (requested_margin>=0?requested_margin:achieved_margin);
    json::Object out;
    out["dialect"] = std::string(dialect);
    out["valid"] = layout.valid;
    out["output_precision"]=options.output_precision;
    out["scale"] = layout.scale;
    out["margin"] = achieved_margin;
    // Overlaid groups: placements of different groups overlap by design, and
    // `scale` above is the smallest of the per-group scales listed in stats.
    if (layout.overlaid_groups) out["overlaid_groups"] = true;
    if (layout.cancelled || options.cancel.requested()) out["cancelled"] = true;
    out["input"] = input_report;
    // Texel snapping. `corners` is a rigid per-island translation and is
    // applied to the layout itself, all or nothing, exactly as it always has been.
    // The vertex modes produce geometry a transform cannot express (or, for bbox, a
    // transform this Layout cannot hold), so they are computed separately and the
    // islands below are emitted from that result instead.
    std::unique_ptr<AlignResult> aligned;
    bool align_snap_interior = false;
    if (layout.valid && options.snap_tex_size > 0) {
        const double required = requested_margin >= 0 ? requested_margin : achieved_margin;
        const bool merged = std::any_of(members.begin(), members.end(), [](const auto& kv) { return kv.second.size() > 1; });
        auto mode = options.snap_mode;
        json::Object pa;
        if (mode != SnapMode::BBoxMin && merged) {
            // A locked group's members share one placement, and their geometry lives
            // in the leader's parts, so there is no per-member vertex list to emit.
            pa["downgraded"] = std::string("locked islands share a transform");
            mode = SnapMode::BBoxMin;
        }
        pa["mode"] = std::string(mode == SnapMode::BBoxMin ? "bbox_min" : mode == SnapMode::BBox ? "bbox"
                                 : mode == SnapMode::Outline ? "outline" : "all");
        pa["target"] = std::string(options.snap_target == SnapTarget::Corner ? "corner" : "center");
        pa["tex_size"] = options.snap_tex_size;
        // The in-place snap only knows texel corners; every other mode, and every
        // centre target, goes through align_layout.
        if (mode == SnapMode::BBoxMin && options.snap_target == SnapTarget::Corner) {
            Layout snapped = layout;
            const auto moved = snap_to_texels(snapped, islands, target, options.snap_tex_size, options.target_aspect);
            const auto check = validate_layout(islands, snapped, target, required, tolerance, obstacles, options.target_aspect, options.targets, layout.overlaid_groups,border);
            pa["moved"] = moved; pa["applied"] = check.ok;
            if (check.ok) { layout = snapped; achieved_margin = required; out["margin"] = achieved_margin; }
            else pa["reason"] = check.messages.empty() ? std::string("validation failed") : check.messages.front();
        } else {
            aligned.reset(new AlignResult(align_layout(layout, islands, target, options.snap_tex_size,
                                                       options.target_aspect, mode, options.snap_target,
                                                       required, tolerance, obstacles, options.targets,border,options.output_precision)));
            pa["moved"] = aligned->moved;
            pa["fallbacks"] = aligned->fallbacks;
            // Applied when the result validates and at least one island kept its
            // snap; an island that had to be reverted is counted under "fallbacks".
            pa["applied"] = aligned->validation.ok && aligned->fallbacks < aligned->islands.size();
            pa["max_shift"] = aligned->max_shift;
            pa["vertex_output"] = aligned->vertex_output;
            // `all` also asks the client to snap the interior vertices it owns and
            // the packer never saw; the packer only ever gets island outlines.
            align_snap_interior = mode == SnapMode::All;
            if (align_snap_interior) pa["snap_interior"] = true;
            if (!aligned->validation.ok)
                pa["reason"] = aligned->validation.messages.empty() ? std::string("validation failed") : aligned->validation.messages.front();
            achieved_margin = required;
            out["margin"] = achieved_margin;
        }
        out["snap_to_texels"] = pa;
    }
    json::Array unplaced;
    for (const auto id : layout.unplaced) unplaced.push_back(double(id));
    out["unplaced"] = unplaced;
    if (!layout.failure.empty()) out["failure"] = layout.failure;
    json::Array placed = islands_json(request, layout);
    if (aligned) {
        // Overwrite the transforms with the aligned ones, and attach per-vertex UVs
        // for the islands whose vertices moved individually. The vertex arrays follow
        // the INPUT rings one for one: check_and_repair may have dropped or reversed
        // vertices, so each aligned vertex is looked up by the original coordinate it
        // came from, and anything the repair removed falls back to the transform.
        std::map<std::uint64_t, const AlignedIsland*> by_id;
        for (const auto& a : aligned->islands) by_id[a.id] = &a;
        for (auto& entry : placed) {
            auto& item = *entry.obj;
            const auto id = static_cast<std::uint64_t>(item["id"].as_number());
            const auto found = by_id.find(id);
            if (found == by_id.end()) continue;
            const AlignedIsland& a = *found->second;
            json::Array tr;
            for (double v : a.transform) tr.push_back(v);
            item["transform"] = tr;
            if (!a.vertices_moved) continue;
            const auto packed = std::find_if(islands.begin(), islands.end(), [&](const Island& i) { return i.id == id; });
            const auto source = source_islands.find(id);
            if (packed == islands.end() || source == source_islands.end()) continue;
            std::map<std::pair<double, double>, Vec2> snapped_at;
            auto record = [&](const Ring& from, const Ring& to) {
                for (std::size_t k = 0; k < from.size() && k < to.size(); ++k)
                    snapped_at[{from[k].x, from[k].y}] = to[k];
            };
            auto record_polygon = [&](const Polygon& from, const Polygon& to) {
                record(from.outer, to.outer);
                for (std::size_t h = 0; h < from.holes.size() && h < to.holes.size(); ++h) record(from.holes[h], to.holes[h]);
            };
            if (!a.parts.empty()) record_polygon(packed->polygon, a.parts[0]);
            for (std::size_t k = 0; k < packed->extra_parts.size() && k + 1 < a.parts.size(); ++k)
                record_polygon(packed->extra_parts[k], a.parts[k + 1]);
            auto emit_ring = [&](const Ring& ring) {
                json::Array out_ring;
                for (const auto v : ring) {
                    const auto hit = snapped_at.find({v.x, v.y});
                    const Vec2 q = hit != snapped_at.end()
                                       ? hit->second
                                       : Vec2{a.transform[0] * v.x + a.transform[1] * v.y + a.transform[2],
                                              a.transform[3] * v.x + a.transform[4] * v.y + a.transform[5]};
                    out_ring.push_back(json::Array{q.x, q.y});
                }
                return out_ring;
            };
            auto emit_polygon = [&](const Polygon& polygon) {
                json::Object p;
                p["outer"] = emit_ring(polygon.outer);
                json::Array holes;
                for (const auto& hole : polygon.holes) holes.push_back(emit_ring(hole));
                p["holes"] = holes;
                return p;
            };
            json::Object vertices = emit_polygon(source->second.polygon);
            json::Array parts;
            for (const auto& part : source->second.extra_parts) parts.push_back(emit_polygon(part));
            if (!parts.empty()) vertices["parts"] = parts;
            item["vertices"] = vertices;
            item["vertex_output"] = true;
            if (align_snap_interior) item["snap_interior"] = true;
        }
    }
    const double required=requested_margin>=0?requested_margin:achieved_margin;
    auto report=certify_emitted(request,layout,placed,required,border);
    if(!report.ok && unsnapped.valid && options.snap_tex_size>0) {
        const auto reason=report.messages.empty()?std::string("emitted geometry validation failed"):report.messages.front();
        layout=unsnapped;placed=islands_json(request,layout);
        report=certify_emitted(request,layout,placed,required,border);
        auto& snap=*out.at("snap_to_texels").obj;
        snap["applied"]=false;snap["moved"]=0;snap["vertex_output"]=false;
        snap["max_shift"]=0;
        snap["fallbacks"]=layout.placements.size();snap["reason"]=reason;
        snap.erase("snap_interior");
    }
    if(self_check || !report.ok)out["validation"]=validation_json(report);
    if(!report.ok) {
        out["valid"]=false;
        // An empty failed search has no emitted geometry to certify. Keep its
        // original cause; the full missing-output report remains available below.
        if (layout.valid || !layout.placements.empty() || layout.failure.empty())
            out["failure"]=report.messages.empty()?std::string("emitted geometry validation failed"):report.messages.front();
    }
    if(request.require_complete && !input_report.at("complete").as_bool()) {
        // No search ran after input rejection. Missing output is a consequence;
        // keep the actual source island/reason as the primary failure summary.
        out["failure"]=std::string("input validation rejected required islands");
        for(const auto& value:input_report.at("issues").as_array()) {
            const auto& issue=value.as_object();
            if(issue.at("fatal").as_bool()) {
                out["failure"]=std::string("input island ")+json::dump(issue.at("island"))+": "+issue.at("message").as_string();
                break;
            }
        }
    }
    std::set<std::uint64_t> returned;
    for(const auto& value:placed) {
        const auto& item=value.as_object();
        if(auto s=item.find("shelved");s!=item.end() && s->second.as_bool()) continue;
        returned.insert(static_cast<std::uint64_t>(item.at("id").as_number()));
    }
    bool complete=out.at("valid").as_bool() && returned.size()==request.requested_ids.size();
    for(auto id:request.requested_ids) complete=complete && returned.count(id)>0;
    out["complete"]=complete;
    if(request.density_mode=="uniform_layout") {
        json::Object density_report;
        density_report["mode"]=request.density_mode;
        density_report["requested"]=request.texel_density_request;
        density_report["factor"]=request.density_factor;
        density_report["achieved"]=complete?json::Value(request.texel_density_request):json::Value();
        out["density"]=density_report;
    }
    if(request.require_complete && !complete) out["valid"]=false;
    out["islands"] = placed;
    json::Object s;
    s["evaluations"] = stats.evaluations;
    s["scales_tried"] = stats.scales_tried;
    s["improve_rounds"] = stats.improve_rounds;
    s["upper_bound"] = stats.upper_bound;
    s["seconds"] = stats.seconds;
    s["prepare_seconds"] = stats.prepare_seconds;
    s["geometry_seconds"] = stats.geometry_seconds;
    s["setup_seconds"] = stats.setup_seconds;
    s["place_seconds"] = stats.place_seconds;
    if (options.compaction_time > 0) {
        json::Object cp;
        cp["attempts"] = stats.compaction_attempts;
        cp["accepted"] = stats.compaction_accepted;
        cp["rotations"] = stats.compaction_rotations;
        cp["threads"] = stats.compaction_threads;
        cp["rounds"] = stats.compaction_rounds;
        cp["start_scale"] = stats.compaction_start_scale;
        cp["seconds"] = stats.compaction_seconds;
        if (!stats.compaction_skipped.empty()) cp["skipped"] = stats.compaction_skipped;
        s["compaction"] = cp;
    }
    if (!stats.group_scales.empty()) {
        json::Object gs;
        for (const auto& [group, scale] : stats.group_scales) gs[std::to_string(group)] = scale;
        s["group_scales"] = gs;
    }
    if (!stats.cluster_scales.empty()) {
        // One uniform scale per texel-density cluster, in packing order.
        json::Array clusters;
        for (const auto& c : stats.cluster_scales) {
            json::Object item;
            item["cluster"] = c.cluster;
            item["scale"] = c.scale;
            item["islands"] = c.islands;
            item["valid"] = c.valid;
            clusters.push_back(item);
        }
        s["clusters"] = clusters;
    }
    if (options.shrink_spanning) s["shrunk_islands"] = stats.shrunk_islands;
    if (texel_density_request > 0) s["texel_density_islands"] = texel_density_islands;
    if (stats.gpu_trials > 0 || !stats.gpu_skipped.empty()) {
        json::Object g;
        g["device"] = stats.gpu_device;
        g["dispatches"] = stats.gpu_dispatches;
        g["trials"] = stats.gpu_trials;
        g["evaluations"] = stats.gpu_evaluations;   // Trials the device placed completely.
        g["accepted"] = stats.gpu_accepted;         // Of those, layouts the validator accepted.
        g["used"] = stats.gpu_used;                 // Of those, layouts that became a result.
        g["upload_seconds"] = stats.gpu_upload_seconds;
        g["compute_seconds"] = stats.gpu_compute_seconds;
        g["wait_seconds"] = stats.gpu_wait_seconds;
        if (!stats.gpu_skipped.empty()) g["skipped"] = stats.gpu_skipped;
        s["gpu"] = g;
    }
    json::Array hist;
    for (const auto& [scale, valid] : stats.history) hist.push_back(json::Array{scale, valid});
    s["history"] = hist;
    out["stats"] = s;
    return out;
}

} // namespace uvpack::cli
