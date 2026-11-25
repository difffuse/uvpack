#include "interior_overlap.hpp"
#define BOOST_MP_STANDALONE
#include <boost/multiprecision/cpp_int.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace uvpack::detail {
namespace {
using Integer=boost::multiprecision::cpp_int;
// Disable expression templates: intermediates must own their operands.
using Rational=boost::multiprecision::number<boost::multiprecision::rational_adaptor<
    boost::multiprecision::cpp_int_backend<>>,boost::multiprecision::et_off>;
struct Point {Rational x,y;};
struct Edge {Point a,b;};
struct Interrupted {};
void checkpoint(const CancellationToken* token) {if(token && token->requested())throw Interrupted{};}

Rational exact(double value) {
    if(!std::isfinite(value))throw std::invalid_argument("nonfinite overlap coordinate");
    if(value==0)return 0;
    int exponent=0;
    const double fraction=std::frexp(value,&exponent);
    const auto mantissa=static_cast<std::int64_t>(std::ldexp(fraction,53));
    Rational result{Integer{mantissa}};
    if(exponent>=53)result*=Integer{1}<<(exponent-53);
    else result/=Integer{1}<<(53-exponent);
    return result;
}
Rational orientation(const Point& a,const Point& b,const Point& p) {
    return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
}
bool opposite(const Rational& a,const Rational& b) {return (a<0 && b>0)||(a>0 && b<0);}

std::vector<Edge> edges(const std::vector<Ring>& rings,std::vector<double>& levels,const CancellationToken* cancel) {
    std::vector<Edge> result;
    for(const auto& ring:rings) {
        std::vector<Point> points;points.reserve(ring.size());
        for(const auto p:ring){checkpoint(cancel);points.push_back({exact(p.x),exact(p.y)});levels.push_back(p.y);}
        for(std::size_t i=0;i<points.size();++i) {
            checkpoint(cancel);result.push_back({points[i],points[(i+1)%points.size()]});
        }
    }
    return result;
}
std::vector<Rational> crossings(const std::vector<Edge>& edges,const Rational& y,const CancellationToken* cancel) {
    std::vector<Rational> x;
    for(const auto& edge:edges) {
        checkpoint(cancel);
        if((edge.a.y<y && y<edge.b.y)||(edge.b.y<y && y<edge.a.y))
            x.push_back(edge.a.x+(y-edge.a.y)*(edge.b.x-edge.a.x)/(edge.b.y-edge.a.y));
    }
    std::sort(x.begin(),x.end(),[&](const Rational& a,const Rational& b){checkpoint(cancel);return a<b;});
    return x;
}
}

InteriorRelation part_interiors_overlap(const std::vector<Ring>& a,const std::vector<Ring>& b,
                                        const CancellationToken* cancel) {
    try {
        checkpoint(cancel);
        std::vector<double> levels;
        const auto aa=edges(a,levels,cancel),bb=edges(b,levels,cancel);
        // A transverse boundary crossing creates a region inside both polygons,
        // regardless of ring winding or whether either edge borders a hole.
        for(const auto& x:aa)for(const auto& y:bb) {
            checkpoint(cancel);
            if(std::max(x.a.x,x.b.x)<std::min(y.a.x,y.b.x) ||
               std::max(y.a.x,y.b.x)<std::min(x.a.x,x.b.x) ||
               std::max(x.a.y,x.b.y)<std::min(y.a.y,y.b.y) ||
               std::max(y.a.y,y.b.y)<std::min(x.a.y,x.b.y))continue;
            if(opposite(orientation(x.a,x.b,y.a),orientation(x.a,x.b,y.b)) &&
               opposite(orientation(y.a,y.b,x.a),orientation(y.a,y.b,x.b)))
                return InteriorRelation::Overlap;
        }
        // Without transverse crossings, edge order is constant in every open
        // slab between vertex Y levels. Exact rational midpoints also work when
        // adjacent double levels have no representable floating midpoint.
        std::sort(levels.begin(),levels.end(),[&](double a,double b){checkpoint(cancel);return a<b;});
        levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
        for(std::size_t k=1;k<levels.size();++k) {
            checkpoint(cancel);
            const Rational y=(exact(levels[k-1])+exact(levels[k]))/2;
            const auto ax=crossings(aa,y,cancel),bx=crossings(bb,y,cancel);
            if(ax.size()%2 || bx.size()%2)throw std::invalid_argument("invalid overlap ring parity");
            // Even-odd pairs subtract holes independently of their orientation.
            for(std::size_t i=0,j=0;i<ax.size() && j<bx.size();) {
                checkpoint(cancel);
                if(std::max(ax[i],bx[j])<std::min(ax[i+1],bx[j+1]))return InteriorRelation::Overlap;
                if(ax[i+1]<bx[j+1])i+=2;else j+=2;
            }
        }
        checkpoint(cancel);
        return InteriorRelation::Disjoint;
    } catch(const Interrupted&) {return InteriorRelation::Cancelled;}
}
}
