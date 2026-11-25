#pragma once
// Continuous-domain layout validator on transformed ORIGINAL polygons: target
// containment, pairwise overlap and minimum gap. Independent of the rows
// representation; used by the CLI to revalidate its own output.
#include "uvpack/packer.hpp"
#include <optional>

namespace uvpack {
// Eight storage-precision epsilons scaled by the target coordinate magnitude.
double validation_tolerance(const PackTarget& target,const std::vector<PackTarget>& targets={},
                            const std::string& output_precision="float64");
struct ValidationReport {
    bool ok = true;
    bool cancelled = false; // Interrupted validation never certifies a partial result.
    std::size_t overlaps = 0;
    std::size_t border_violations = 0;
    std::size_t invalid_geometry = 0;
    double tolerance = 0;
    double min_border = std::numeric_limits<double>::infinity();
    std::size_t outside_target = 0;
    std::size_t gap_violations = 0;
    double min_gap = std::numeric_limits<double>::infinity();
    std::vector<std::string> messages; // First few problems, human-readable.
    // Ids of the islands involved in a containment, overlap or gap problem, each
    // listed once. Callers that can undo a per-island change (texel snapping) use
    // this to revert exactly the islands that broke the layout.
    std::vector<std::uint64_t> offenders;
};
// Legacy contact predicate: true for shared interior OR boundary contact. Used
// by the bind-overlapping "touching" pre-pass; zero-gap validation is separate.
bool polygons_overlap(const Polygon& a, const Polygon& b);
// `validate_overlaid_groups` is for layouts produced with PackOptions::overlay_groups:
// every group was packed over the full target on its own, so only pairs of islands that
// share a group are compared. Islands flagged `shelved` sit above the target on purpose:
// they are skipped by the containment check and only have to not overlap anything.
// Zero effective gap permits boundary-only contact; any positive gap (including
// obstacle extra margin) forbids contact. True interior overlap always fails.
ValidationReport validate_layout(const std::vector<Island>& islands, const Layout& layout,
                                 const PackTarget& target, double gap, double tolerance = -1,
                                 const std::vector<Obstacle>& obstacles = {}, double target_aspect = 1.0,
                                 const std::vector<PackTarget>& targets = {},
                                 bool validate_overlaid_groups = false, double border = 0);

// Explicit-geometry variant: the polygons arrive already placed, in output UV space
// (the space the CLI's `transform` maps into), one entry per island with its polygon
// first and its extra parts after it. Used to validate geometry that no longer
// follows from a placement's transform -- the texel snap modes that move
// vertices individually. Containment, overlap and gap rules are identical to
// validate_layout's; only the "every island got placed" bookkeeping is missing,
// since there is no Layout to compare against.
struct PlacedIsland {
    std::uint64_t id = 0;
    std::vector<Polygon> parts; // Island polygon first, then its extra parts.
    // When supplied, containment/border apply to this target index only.
    // Absent retains the legacy "inside any target" policy.
    std::optional<std::size_t> tile = std::nullopt;
    std::size_t group = 0;
    bool shelved = false;
};
ValidationReport validate_polygons(const std::vector<PlacedIsland>& placed, const PackTarget& target,
                                   double gap, double tolerance = -1,
                                   const std::vector<Obstacle>& obstacles = {}, double target_aspect = 1.0,
                                   const std::vector<PackTarget>& targets = {}, double border = 0,
                                   bool overlaid_groups = false);
struct ValidationOptions {
    double gap = 0;
    double border = 0;
    double target_aspect = 1;
    std::string output_precision = "float64";
    std::vector<PackTarget> targets;
    CancellationToken cancel;
    std::vector<Obstacle> obstacles; // Rounded and checked at the same declared precision.
    bool overlaid_groups = false; // Only same-group movable pairs require separation.
};
// Checks a copy at declared storage precision without repair or placement.
// Invalid geometry is a failed report; malformed policy/targets throw unless
// cancellation preempts their checks.
// Cancellation returns ok=false/cancelled=true without partial measurements.
ValidationReport validate_stored_polygons(const std::vector<PlacedIsland>& placed,
                                         const PackTarget& target, const ValidationOptions& options);
// Transform ORIGINAL source polygons and validate a copy at declared precision.
// Checks placement/source/unplaced identity, assigned tiles, overlay groups and
// shelved geometry. An explicitly partial layout may pass its geometry checks;
// this does not turn unplaced/shelved islands into a complete packing result.
// No envelope reconstruction, UV repair, packing or mutation is performed.
// Source triangle coverage is outside this polygon-only certificate.
ValidationReport validate_stored_layout(const std::vector<Island>& islands,const Layout& layout,
                                       const PackTarget& target,const ValidationOptions& options);
} // namespace uvpack
