#pragma once
// Overlap-tolerant compaction (in the spirit of sparrow, arXiv 2509.13329:
// "shrink then separate"). The greedy search returns the best layout it can build
// one island at a time; compaction starts from that layout, grows every island in
// place to a slightly larger scale so that overlaps appear, and then repairs the
// overlaps with a guided local search that may translate, rotate (continuously)
// and swap islands. A candidate scale is accepted only when the total overlap of
// the margin-expanded polygons reaches exactly zero AND the exact continuous
// validator accepts the resulting layout, so an invalid layout is never returned.
#include "uvpack/packer.hpp"

namespace uvpack {

struct CompactionStats {
    double start_scale = 0;        // Scale of the incoming layout.
    double best_scale = 0;         // Scale of the returned layout.
    std::size_t attempts = 0;      // Candidate scales tried.
    std::size_t accepted = 0;      // Candidate scales separated and validated.
    std::size_t rejected_by_validator = 0; // Zero overlap reached but the exact check said no.
    std::size_t sweeps = 0;        // Local-search sweeps over the overlapping islands.
    std::size_t relocations = 0;   // Islands relocated.
    std::size_t overlap_tests = 0; // Clipper polygon intersections evaluated.
    std::size_t rotations_applied = 0; // Relocations that changed the island's angle.
    double polish_seconds = 0;     // Time spent in the zero-growth polish pass.
    std::size_t threads = 1;       // Attempts run per round (1 = the single-threaded path).
    std::size_t rounds = 0;        // Portfolio rounds; one attempt per thread each.
    double seconds = 0;
    bool ran = false;              // False when compaction declined the problem.
    std::string skipped;           // Why it declined, when it did.
};

// Compact `start` (which must be a valid layout of `islands` in `target`). Returns
// the best valid layout found, which is `start` itself when nothing better is found.
// Declines (returns `start` unchanged, stats.skipped set) for multi-tile layouts,
// partial layouts, and layouts whose geometry cannot be margin-expanded.
//
// With `options.compaction_threads` above one (the default resolves to `options.threads`)
// the stage runs as a portfolio: every round starts K independent attempts from the same
// incumbent, one per thread, each with its own RNG stream and its own target delta from a
// ladder around the current one, and the largest validated scale of the round becomes the
// next incumbent. Each attempt owns all of its state, Clipper instances included.
Layout compact_layout(const std::vector<Island>& islands, const PackTarget& target,
                      const Layout& start, const PackOptions& options,
                      const std::vector<Obstacle>& obstacles = {},
                      CompactionStats* stats = nullptr);

} // namespace uvpack
