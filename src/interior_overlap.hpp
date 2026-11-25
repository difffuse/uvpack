#pragma once
#include "uvpack/island.hpp"
#include "uvpack/cancellation.hpp"

namespace uvpack::detail {
enum class InteriorRelation { Disjoint, Overlap, Cancelled };
// Simple outer ring followed by valid disjoint holes, already in one coordinate
// space. Classifies interiors exactly for the supplied finite binary doubles.
InteriorRelation part_interiors_overlap(const std::vector<Ring>& a,const std::vector<Ring>& b,
                                        const CancellationToken* cancel=nullptr);
}
