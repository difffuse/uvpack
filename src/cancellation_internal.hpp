#pragma once
#include "uvpack/packer.hpp"
namespace uvpack {
namespace detail {
struct PackingStoragePolicy {
    double gap_reserve = 0;
    double border_reserve = 0;
    bool finish_compaction_candidates = true;
};
}
PackOptions storage_search_options(const std::vector<Island>& islands,const PackTarget& target,
                                  const PackOptions& options,const std::vector<Obstacle>& obstacles,
                                  bool finish_compaction_candidates=true);
// Checks a copy, applying origin/shelving unless already finished. A failed
// certificate marks the candidate invalid before it can replace an incumbent.
bool storage_candidate_valid(Layout& layout,const std::vector<Island>& islands,const PackTarget& target,
                             const PackOptions& options,const std::vector<Obstacle>& obstacles,
                             bool finished=false);
// Final certificates independently check retained cancelled incumbents. Candidate
// checks instead share the request token so they cannot delay search cancellation.
Layout finish_result(Layout layout,const std::vector<Island>& islands,const PackTarget& target,
                           const PackOptions& options,const std::vector<Obstacle>& obstacles,
                           bool final_certificate=true);
}
