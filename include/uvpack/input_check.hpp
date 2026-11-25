#pragma once
// Input validation and light repair for island polygons before packing.
#include "uvpack/island.hpp"
#include <string>
#include <vector>

namespace uvpack {
struct InputIssue { std::uint64_t island; std::string message; bool fatal; std::string reason; };
struct InputReport {
    std::vector<InputIssue> issues;
    bool complete = true;
    std::vector<std::uint64_t> rejected_ids;
    std::vector<std::uint64_t> hull_ids;
    std::size_t repaired = 0;  // Islands changed by repair (duplicate vertices, closing vertex, winding).
    std::size_t rejected = 0;  // Islands removed (fatal issues).
};
struct InputPolicy {
    std::string hull_policy = "reject"; // reject | convex_hull, opt-in only
    bool strict = false;
};
// Rejects duplicate IDs before mutation. Retains original envelope points and
// reports every rejected/recovered ID; hulls are packing envelopes, not UV repair.
InputReport check_and_repair(std::vector<Island>& islands, const InputPolicy& policy);
// Repairs in place: drops repeated/closing vertices, normalizes winding (outer
// counter-clockwise, holes clockwise). Rejects (removes) islands with fewer than
// three distinct vertices, non-finite coordinates, zero area, self-intersecting
// rings, or holes that are not inside the outer ring. `strict` makes any issue
// fatal for that island.
InputReport check_and_repair(std::vector<Island>& islands, bool strict = false);
} // namespace uvpack
