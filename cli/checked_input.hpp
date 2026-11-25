#pragma once
// CLI schema checks run before repair/search. Geometry validity remains the
// native repair/validation policy's job; this layer checks shape, types and bounds.
#include "cli.hpp"
#include "uvpack/rows.hpp"
#include <charconv>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>

namespace uvpack::cli::input {
inline constexpr double safe_integer = 9007199254740991.;
struct Limits {
    std::size_t islands = 65536, vertices = 1048576, rings = 262144, parts = 262144;
    std::size_t targets = 4096;
};

[[noreturn]] inline void fail(const std::string& field,const std::string& why) {
    throw std::runtime_error(field+": "+why);
}
inline const json::Value& required(const json::Value& object,const std::string& field) {
    object.as_object();
    const auto* value=object.find(field);
    if(!value)fail(field,"is required");
    return *value;
}
inline double number(const json::Value& value,const std::string& field) {
    if(value.kind!=json::Value::Number || !std::isfinite(value.num))fail(field,"must be a finite number");
    return value.num;
}
template<class T> inline T integer(const json::Value& value,const std::string& field) {
    const double n=number(value,field);
    if(value.fractional_lexeme || std::floor(n)!=n || n < -safe_integer || n > safe_integer ||
       static_cast<long double>(n)<static_cast<long double>(std::numeric_limits<T>::lowest()) ||
       static_cast<long double>(n)>static_cast<long double>(std::numeric_limits<T>::max()))
        fail(field,"must be an exactly representable integer in range");
    return static_cast<T>(n);
}
template<class T> inline T integer_or(const json::Value& object,const std::string& field,T fallback) {
    object.as_object();
    const auto* value=object.find(field);
    return value?integer<T>(*value,field):fallback;
}
inline std::size_t group_key(const std::string& text) {
    if(text.empty() || (text.size()>1 && text[0]=='0') || text[0]<'0' || text[0]>'9')
        fail("groups key","must be canonical unsigned decimal");
    std::uint64_t value=0;
    const auto result=std::from_chars(text.data(),text.data()+text.size(),value);
    if(result.ec!=std::errc{} || result.ptr!=text.data()+text.size() ||
       value>9007199254740991ULL || value>std::numeric_limits<std::size_t>::max())
        fail("groups key","must be a safe nonnegative integer");
    return static_cast<std::size_t>(value);
}
inline void range(const json::Value& object,const char* field,double low,double high,
                  bool positive=false) {
    if(const auto* value=object.find(field)) {
        const auto n=number(*value,field);
        if(n<low || n>high || (positive && n==low))fail(field,"number outside supported range");
    }
}
inline void choice(const json::Value& object,const char* field,std::initializer_list<const char*> choices) {
    if(const auto* value=object.find(field)) {
        const auto& text=value->as_string();
        for(const char* candidate:choices)if(text==candidate)return;
        fail(field,"unknown value");
    }
}
inline void boolean(const json::Value& object,const char* field) {
    if(const auto* value=object.find(field))
        if(value->kind!=json::Value::Bool)fail(field,"must be bool");
}
inline void count(std::size_t value,std::size_t& used,std::size_t maximum,const char* field) {
    if(value>maximum-used)fail(field,"request limit exceeded");
    used+=value;
}
inline void array_limit(const json::Value& value,std::size_t maximum,const char* field) {
    if(value.as_array().size()>maximum)fail(field,"request limit exceeded");
}

struct Geometry {
    Limits limits;
    std::size_t vertices=0,rings=0,parts=0;
    void ring(const json::Value& value) {
        const auto& points=value.as_array();
        count(1,rings,limits.rings,"rings");
        count(points.size(),vertices,limits.vertices,"vertices");
        for(const auto& point:points) {
            const auto& xy=point.as_array();
            if(xy.size()!=2)fail("vertex","must be [x,y]");
            number(xy[0],"vertex x");number(xy[1],"vertex y");
        }
    }
    void polygon(const json::Value& value,bool envelope_allowed=false) {
        value.as_object();
        count(1,parts,limits.parts,"polygon parts");
        if(const auto* outer=value.find("outer"))ring(*outer);
        else if(!envelope_allowed || !value.find("envelope_points"))fail("outer","is required");
        if(const auto* holes=value.find("holes"))for(const auto& hole:holes->as_array())ring(hole);
        if(envelope_allowed)if(const auto* points=value.find("envelope_points"))ring(*points);
    }
};

inline PackTarget target(const json::Value& value) {
    const auto& a=value.as_array();
    if(a.size()!=4)fail("target","must be [minx,miny,maxx,maxy]");
    PackTarget t{number(a[0],"target"),number(a[1],"target"),number(a[2],"target"),number(a[3],"target")};
    if(!(t.max_x>t.min_x) || !(t.max_y>t.min_y) ||
       !std::isfinite(t.max_x-t.min_x) || !std::isfinite(t.max_y-t.min_y))
        fail("target","must have finite positive dimensions");
    return t;
}

inline void validate(const json::Value& doc,bool validation=false,Limits limits={}) {
    doc.as_object();
    const auto& islands=required(doc,"islands");
    array_limit(islands,limits.islands,"islands");
    Geometry geometry{limits};
    for(const auto& item:islands.as_array()) {
        item.as_object();
        if(const auto* id=item.find("id"))integer<std::uint64_t>(*id,"island ID");
        else if(validation)fail("id","is required for validation");
        if(validation) {
            if(const auto* tile=item.find("tile"))integer<std::size_t>(*tile,"tile index");
            integer_or<std::size_t>(item,"group",0);
            boolean(item,"shelved");
        }
        geometry.polygon(item,!validation);
        if(const auto* parts=item.find("parts"))
            for(const auto& part:parts->as_array())geometry.polygon(part);
        if(!validation) {
            for(const char* key:{"scale_locked","rotation","flip"})boolean(item,key);
            integer_or<int>(item,"rotation_step",-1);
            integer_or<std::size_t>(item,"group",0);
            for(const char* key:{"bind_group","density_group","scale_cluster"})integer_or<long long>(item,key,0);
            for(const char* key:{"pre_scale","scale_multiplier"})
                range(item,key,0,std::numeric_limits<double>::max(),true);
            for(const char* key:{"area_uv","area_3d","area_3d_global","density_cross_sum","density_cross_sum_global"})
                range(item,key,0,std::numeric_limits<double>::max());
        }
    }
    if(const auto* obstacles=doc.find("obstacles")) {
        array_limit(*obstacles,limits.islands,"obstacles");
        for(const auto& item:obstacles->as_array()) {
            geometry.polygon(item);
            range(item,"extra_margin",0,std::numeric_limits<double>::max());
        }
    }
    std::vector<PackTarget> targets;
    if(const auto* t=doc.find("target"))targets.push_back(target(*t));
    if(const auto* ts=doc.find("targets")) {
        array_limit(*ts,limits.targets,"targets");
        if(ts->as_array().empty())fail("targets","must not be empty");
        targets.clear();
        for(const auto& t:ts->as_array())targets.push_back(target(t));
    }
    if(targets.empty())targets.push_back(PackTarget{});
    const auto* supplied=doc.find("options");
    const json::Value defaults(json::Object{});
    const auto& options=supplied?*supplied:defaults;
    options.as_object();
    const double maximum=std::numeric_limits<double>::max();
    for(const char* key:{"margin","pixel_margin","pixel_border_margin","tex_size"})range(options,key,0,maximum);
    range(options,"border_margin",validation?0:-maximum,maximum);
    range(options,"target_aspect",0,maximum,true);
    boolean(options,"stream");
    boolean(options,"overlay_groups");
    choice(options,"output_precision",{"float64","float32"});
    const double tex=options.number_or("tex_size",0);
    if((options.find("pixel_margin") || options.find("pixel_border_margin")) && tex<=0)
        fail("tex_size","pixel margins require a positive texture size");
    if(validation)return;
    for(const char* key:{"allow_mirroring","allow_partial","allow_rotation","compaction_polish","compaction_rotation",
                        "dedup_orientations","equalize_density","extent_estimate","fast_search","gpu","gpu_in_round",
                        "group_adjacent","overlay_groups","parallel_bracket","prealign","progress","shelve_unplaced",
                        "shrink_spanning","single_pass","snap_to_texels","validate","require_complete","strict_input"})
        boolean(options,key);
    if(options.bool_or("snap_to_texels",false) && tex<=0)fail("tex_size","snapping requires a positive texture size");
    for(const char* key:{"sub_rows","threads","compaction_threads"}) {
        const auto value=integer_or<std::size_t>(options,key,0);
        if(value>256)fail(key,"request limit is 256");
    }
    for(const char* key:{"trials","gpu_trials","max_evaluations","bracket_rounds","compaction_attempt_sweeps","compaction_max_rounds"}) {
        const auto value=integer_or<std::size_t>(options,key,0);
        if(value>65536)fail(key,"request limit is 65536");
    }
    integer_or<std::uint32_t>(options,"seed",1);
    integer_or<int>(options,"rotation_step",90);
    for(const char* key:{"stripe_height","scale"})range(options,key,0,maximum,true);
    for(const char* key:{"time_budget","compaction_time","stall_timeout"})range(options,key,0,86400);
    range(options,"texel_density",0,maximum);
    for(const char* key:{"group_cohesion","bracket_start","compaction_delta","estimate_tolerance","bracket_tolerance"})range(options,key,0,1);
    for(const char* key:{"single_pass_start","initial_step"})
        range(options,key,0,1,true);
    range(options,"single_pass_margin_reserve",1,maximum);
    choice(options,"scale_mode",{"max","fixed","max_margin","open_rescale"});
    choice(options,"density_mode",{"per_island","uniform_layout"});
    choice(options,"density_space",{"local","global"});
    choice(options,"scale_policy",{"shared","per_group","custom","by_tiles"});
    choice(options,"hull_policy",{"reject","convex_hull"});
    choice(options,"bind_overlapping",{"off","touching","identical"});
    choice(options,"snap_mode",{"bbox_min","bbox","outline","all"});
    choice(options,"snap_target",{"corner","center","centre"});
    choice(options,"tile_fill",{"sequential","any"});
    choice(options,"fill",{"free","width_first","height_first","proportional"});
    choice(options,"origin",{"bottom_left","bottom_right","top_right","top_left"});
    const auto stripe=options.number_or("stripe_height",PackOptions{}.stripe_height);
    const auto aspect=options.number_or("target_aspect",1);
    for(const auto& t:targets) {
        const auto width=(t.max_x-t.min_x)*aspect;
        if(!std::isfinite(width) || width<=0 || width>double(INT32_MAX)/rows::units ||
           (t.max_y-t.min_y)/stripe>65536)
            fail("target/stripe_height","frame exceeds supported raster dimensions");
    }
    if(const auto* groups=options.find("groups")) {
        groups->as_object();
        if(groups->obj->size()>limits.islands)fail("groups","request limit exceeded");
        for(const auto& [key,group]:*groups->obj) {
            group_key(key);group.as_object();
            integer_or<long long>(group,"scale_cluster",0);
            if(const auto* tiles=group.find("tiles")) {
                array_limit(*tiles,limits.targets,"group tiles");
                for(const auto& tile:tiles->as_array())
                    if(integer<std::size_t>(tile,"tile index")>=targets.size())fail("tile index","outside target list");
            }
        }
    }
}
} // namespace uvpack::cli::input
