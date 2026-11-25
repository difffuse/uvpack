#include <uvpack/packer.hpp>
#include <uvpack/validator.hpp>
#include <uvpack/offset.hpp>
int installed_pack_check() {
    using namespace uvpack;
    const Polygon square{{{0,0},{.1,0},{.1,.1},{0,.1}}, {}};
    const auto grown=expand_packing_margin(square,.02); // Clipper dependency.
    bool beyond=false;
    for (auto point:grown.outer) beyond= beyond || point.x>.1;
    if (!beyond) return 1;
    const std::vector<Island> source{{1,0,square},{2,0,square}};
    PackOptions options;options.output_precision="float32";
    options.margin=.01;options.border_margin=.02;options.trials=1;options.threads=1;
    options.allow_rotation=false;options.prealign=false;options.stripe_height=.01;
    const auto layout=pack_fixed_scale(source,{},1,options);
    ValidationOptions validation;validation.output_precision="float32";
    validation.gap=.01;validation.border=.02;
    if (!layout.valid || layout.scale!=1 || layout.placements.size()!=2 ||
        !validate_stored_layout(source,layout,{},validation).ok) return 2;
    validation.gap=0;validation.border=0;
    const Polygon lower{{{.125,.125},{.875,.125},{.125,.875}}, {}};
    const Polygon upper{{{.875,.125},{.875,.875},{.125,.875}}, {}};
    if (!validate_stored_polygons({{1,{lower},0},{2,{upper},0}},{},validation).ok) return 3;
    if (validate_stored_polygons({{1,{lower},0},{2,{lower},0}},{},validation).ok) return 4;
    return 0;
}
