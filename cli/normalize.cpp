#include "cli.hpp"
#include "checked_input.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace uvpack::cli {
namespace {
struct Chart {
    std::uint64_t id;
    Ring points;
};

void unit_option(const json::Value& item,const char* key) {
    if(const auto* value=item.find(key))
        if(input::number(*value,key)!=1)input::fail(key,"normalize requires original coordinates and unit scale");
}
}

json::Object normalize_request(const json::Value& doc,const CancellationToken& cancel) {
    const auto& source=input::required(doc,"islands");
    input::array_limit(source,input::Limits{}.islands,"islands");
    if(source.as_array().empty())input::fail("islands","must not be empty");
    const auto& options=input::required(doc,"options");
    const std::set<std::string> supported{"output_precision","equalize_density","density_mode",
        "tex_size","texel_density","density_space","target_aspect","stream"};
    for(const auto& [key,value]:options.as_object())
        if(!supported.count(key))input::fail(key,"unsupported normalization option");
    input::choice(options,"output_precision",{"float64","float32"});
    input::choice(options,"density_mode",{"per_island","uniform_layout"});
    input::choice(options,"density_space",{"local","global"});
    input::boolean(options,"equalize_density");input::boolean(options,"stream");
    for(const char* key:{"tex_size","texel_density","target_aspect"})
        input::range(options,key,0,std::numeric_limits<double>::max(),true);
    const bool uniform=options.string_or("density_mode","per_island")=="uniform_layout";
    const bool equalize=options.bool_or("equalize_density",false);
    if(uniform==equalize)input::fail("density_mode","choose equalize_density or uniform_layout exclusively");
    if(!uniform && options.find("texel_density"))
        input::fail("texel_density","normalize requires uniform_layout for an absolute target");
    const auto precision=options.string_or("output_precision","float64");
    const bool global=options.string_or("density_space","local")=="global";
    const double tex=options.number_or("tex_size",0),aspect=options.number_or("target_aspect",1);
    const double target=options.number_or("texel_density",0);
    if(uniform && (!(tex>0)||!(target>0)||!std::isfinite(tex*aspect)))
        input::fail("uniform_layout","needs positive finite texture dimensions and target density");

    json::Object out{{"dialect",dialect},{"operation","normalize"},{"accepted",false},
        {"complete",false},{"cancelled",false},{"output_precision",precision},{"islands",json::Array{}}};
    auto failure=[&](const std::string& reason) {
        out["failure"]=reason;out["cancelled"]=cancel.requested();return out;
    };
    std::vector<Chart> charts;
    std::vector<DensityIsland> density;
    std::set<std::uint64_t> ids;
    std::size_t vertices=0;
    json::Array fixed_ids;
    for(const auto& item:source.as_array()) {
        if(cancel.requested())return failure("normalization cancelled");
        Chart chart;
        chart.id=input::integer<std::uint64_t>(input::required(item,"id"),"id");
        if(!ids.insert(chart.id).second)input::fail("id","duplicate island ID");
        input::boolean(item,"fixed");
        unit_option(item,"pre_scale");unit_option(item,"scale_multiplier");
        DensityIsland d;
        d.fixed=item.bool_or("fixed",false);
        d.density_group=input::integer_or<long long>(item,"density_group",-1);
        if(uniform && d.density_group>=0)input::fail("density_group","uniform_layout cannot normalize groups");
        d.uv_area=input::number(input::required(item,"area_uv"),"area_uv");
        const char* area_key=global?"area_3d_global":"area_3d";
        d.area_3d=input::number(input::required(item,area_key),area_key);
        if(d.uv_area<0 || d.area_3d<0)input::fail("areas","must be nonnegative");
        const char* cross_key=global?"density_cross_sum_global":"density_cross_sum";
        if(uniform || item.find(cross_key)) {
            d.density_cross_sum=input::number(input::required(item,cross_key),cross_key);
            const long double limit=std::sqrt((long double)d.uv_area)*std::sqrt((long double)d.area_3d);
            if(*d.density_cross_sum<0 || *d.density_cross_sum>limit*(1+64*std::numeric_limits<double>::epsilon()))
                input::fail(cross_key,"violates original-area consistency");
        }
        const auto& points=input::required(item,"envelope_points").as_array();
        if(points.empty())input::fail("envelope_points","must contain original chart corners");
        input::count(points.size(),vertices,input::Limits{}.vertices,"vertices");
        for(const auto& point:points) {
            if(cancel.requested())return failure("normalization cancelled");
            const auto& xy=point.as_array();
            if(xy.size()!=2)input::fail("envelope_points","each corner must be [x,y]");
            chart.points.push_back({input::number(xy[0],"corner x"),input::number(xy[1],"corner y")});
        }
        if(d.fixed)fixed_ids.push_back(double(chart.id));
        charts.push_back(std::move(chart));density.push_back(d);
    }
    double factor=1;
    try {
        if(uniform)factor=set_uniform_layout_density(density,target,tex*aspect,tex);
        else equalize_density(density);
    } catch(const std::domain_error& error) {
        return failure(error.what());
    }
    json::Array transforms;
    long double cross_sum=0,area_sum=0;
    bool have_cross=true;
    for(std::size_t i=0;i<charts.size();++i) {
        if(cancel.requested())return failure("normalization cancelled");
        const auto& chart=charts[i];const auto& d=density[i];
        const double scale=d.pre_scale;
        if(!std::isfinite(scale)||!(scale>0))return failure("normalization scale exceeds numeric range");
        // A single transform for all original corners preserves exact chart
        // connections. The bounding-box center avoids weighting repeated corners.
        Vec2 low=chart.points.front(),high=low;
        for(const auto& point:chart.points) {
            if(cancel.requested())return failure("normalization cancelled");
            low.x=std::min(low.x,point.x);low.y=std::min(low.y,point.y);
            high.x=std::max(high.x,point.x);high.y=std::max(high.y,point.y);
        }
        const double tx=(low.x*.5+high.x*.5)*(1-scale),ty=(low.y*.5+high.y*.5)*(1-scale);
        if(!std::isfinite(tx)||!std::isfinite(ty))return failure("normalization transform exceeds numeric range");
        const double maximum=precision=="float32"?double(std::numeric_limits<float>::max()):std::numeric_limits<double>::max();
        for(const auto& point:chart.points) {
            if(cancel.requested())return failure("normalization cancelled");
            const double x=scale*point.x+tx,y=scale*point.y+ty;
            if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>maximum||std::abs(y)>maximum)
                return failure("normalized coordinates exceed storage range");
        }
        transforms.push_back(json::Object{{"id",double(chart.id)},
            {"transform",json::Array{scale,0.,tx,0.,scale,ty,0.,0.,1.}}});
        if(d.density_cross_sum)cross_sum+=(long double)scale * *d.density_cross_sum;
        else have_cross=false;
        area_sum+=d.area_3d;
    }
    json::Value achieved;
    if(have_cross && tex>0 && area_sum>0) {
        const double value=static_cast<double>(std::sqrt((long double)tex*aspect)*std::sqrt((long double)tex)*cross_sum/area_sum);
        if(!std::isfinite(value))return failure("normalized density exceeds numeric range");
        achieved=value;
    }
    out["density"]=json::Object{{"mode",uniform?"uniform_layout":"equalize"},
        {"factor",factor},{"achieved",achieved},{"equalize_scope","movable_charts"}};
    out["fixed_ids"]=fixed_ids;out["islands"]=transforms;
    out["accepted"]=true;out["complete"]=true;
    return out;
}
} // namespace uvpack::cli
