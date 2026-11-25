#include "cli.hpp"
#include "checked_input.hpp"
#include "uvpack/validator.hpp"
#include <cmath>
#include <stdexcept>

namespace uvpack::cli {
namespace {
const json::Value& required(const json::Value& object,const std::string& name) {
    auto p=object.find(name);if(!p) throw std::runtime_error("missing "+name);return *p;
}
Ring ring(const json::Value& value) {
    Ring result;
    for(const auto& v:value.as_array()) {
        const auto& xy=v.as_array();if(xy.size()!=2) throw std::runtime_error("vertex must be [x,y]");
        result.push_back({xy[0].as_number(),xy[1].as_number()});
    }
    return result;
}
Polygon polygon(const json::Value& value) {
    Polygon p;p.outer=ring(required(value,"outer"));
    if(auto holes=value.find("holes")) for(const auto& h:holes->as_array()) p.holes.push_back(ring(h));
    return p;
}
PackTarget target_of(const json::Value& value) {
    const auto& a=value.as_array();if(a.size()!=4) throw std::runtime_error("target must have 4 coordinates");
    return {a[0].as_number(),a[1].as_number(),a[2].as_number(),a[3].as_number()};
}
}
json::Object validate_request(const json::Value& doc,const CancellationToken& cancel) {
    input::validate(doc,true);
    PackTarget target;
    ValidationOptions options;
    options.cancel=cancel;
    if(auto t=doc.find("target")) target=target_of(*t);
    if(auto ts=doc.find("targets")) {
        for(const auto& t:ts->as_array()) options.targets.push_back(target_of(t));
        if(options.targets.empty()) throw std::runtime_error("targets must not be empty");
    }
    if(auto o=doc.find("options")) {
        o->as_object();
        if(auto v=o->find("margin")) options.gap=v->as_number();
        options.border=options.gap;
        if(auto v=o->find("border_margin")) options.border=v->as_number();
        if(auto v=o->find("target_aspect")) options.target_aspect=v->as_number();
        if(auto v=o->find("output_precision")) options.output_precision=v->as_string();
        options.overlaid_groups=o->bool_or("overlay_groups",false);
        auto pixel_gap=o->find("pixel_margin"),pixel_border=o->find("pixel_border_margin");
        if(pixel_gap || pixel_border) {
            const auto tex=required(*o,"tex_size").as_number();
            if(!std::isfinite(tex)||tex<=0) throw std::runtime_error("pixel margins require positive tex_size");
            if(pixel_gap) options.gap=pixel_gap->as_number()/tex;
            if(pixel_border) options.border=pixel_border->as_number()/tex;
            else if(!o->find("border_margin")) options.border=options.gap;
        }
    }
    std::vector<PlacedIsland> placed;
    if(auto obstacles=doc.find("obstacles"))for(const auto& item:obstacles->as_array()) {
        if(cancel.requested())break;
        options.obstacles.push_back({polygon(item),item.number_or("extra_margin",0)});
    }
    for(const auto& i:required(doc,"islands").as_array()) {
        if(cancel.requested())break; // The native entry point returns an explicit cancelled report.
        const double id=required(i,"id").as_number();
        if(!std::isfinite(id)||id<0||id>9007199254740991.0||std::floor(id)!=id)
            throw std::runtime_error("invalid island ID");
        PlacedIsland item;item.id=static_cast<std::uint64_t>(id);item.parts.push_back(polygon(i));
        item.group=input::integer_or<std::size_t>(i,"group",0);item.shelved=i.bool_or("shelved",false);
        if(auto tile=i.find("tile"))item.tile=input::integer<std::size_t>(*tile,"tile index");
        if(auto parts=i.find("parts")) for(const auto& p:parts->as_array()) item.parts.push_back(polygon(p));
        placed.push_back(std::move(item));
    }
    const auto report=validate_stored_polygons(placed,target,options);
    json::Object v;
    v["ok"]=report.ok;v["cancelled"]=report.cancelled;v["overlaps"]=report.overlaps;v["outside_target"]=report.outside_target;
    v["gap_violations"]=report.gap_violations;v["border_violations"]=report.border_violations;
    v["invalid_geometry"]=report.invalid_geometry;v["tolerance"]=report.tolerance;
    v["min_gap"]=std::isfinite(report.min_gap)?json::Value(report.min_gap):json::Value();
    v["min_border"]=std::isfinite(report.min_border)?json::Value(report.min_border):json::Value();
    json::Array offenders,messages;
    for(auto id:report.offenders) offenders.push_back(double(id));
    for(const auto& m:report.messages) messages.push_back(m);
    v["offenders"]=offenders;v["messages"]=messages;
    return {{"dialect",dialect},{"operation","validate"},{"valid",report.ok},{"complete",report.ok},
            {"cancelled",report.cancelled},
            {"output_precision",options.output_precision},{"validation",v}};
}
} // namespace uvpack::cli
