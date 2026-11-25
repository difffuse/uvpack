#pragma once
// The device-side problem encoding, and a CPU implementation of
// exactly the algorithm the compute shader runs.
//
// This file is the specification the shader is written against: same outward
// rounding onto the bit grid, same vertical bands, same collision jump rule
// (including the short-gap skip), same 32-lane clearance accumulation, same cost and
// same first-win tie-breaking. `reference_trial` is therefore a drop-in stand-in for
// `gpu::Worker::run` on small cases, and it is what tests compare the device against.
//
// It also runs without any GPU, which is why the packer can use it to explain what a
// device answer should have been when a device is present but disagrees.
//
// Conservatism. Every interval is widened onto the bit grid (begin rounds down, end
// rounds up) and onto the 8 vertical bands (low rounds down, high rounds up), for the
// canvas AND for the islands. Two widened masks that do not intersect therefore come
// from footprints that do not intersect either, so a placement this search calls
// clear is clear in the continuous domain. The converse does not hold: the search
// rejects some placements the CPU greedy would take. That only ever costs density.
#include "uvpack/stripe_placer.hpp"
#include "uvpack/gpu.hpp"
#include <cstdint>
#include <vector>

namespace uvpack::gpu {

inline constexpr std::uint32_t empty_row = 0xFFFFFFFFu; // Sentinel in `row_first_bit`.
inline constexpr std::uint32_t no_bit = 0xFFFFFFFFu;

// One island-row segment on the bit grid, with the inclusive band range it occupies.
struct BitInterval {
    std::uint32_t begin = 0, end = 0;      // [begin, end) in bits, island-local.
    std::uint32_t band_lo = 0, band_hi = 0; // Inclusive, 0..bands-1.
};

struct BitOrientation {
    std::uint32_t row_begin = 0;    // Index of this orientation's first entry in `row_start`.
    std::uint32_t row_count = 0;
    float width = 0, height = 0;    // Output units.
    // 1 / height, computed once on the host. The cost's clearance term is scaled by
    // stripe_height / height; Vulkan only promises division to 2.5 ULP, so the device
    // multiplies by this instead and matches the CPU reference bit for bit.
    float inv_height = 0;
    std::int32_t max_offset_bits = -1; // Largest x in bits; < 0 = does not fit the frame.
    std::int32_t last_row = -1;        // Largest candidate row; < 0 = does not fit the frame.
    std::uint32_t probe_row = 0;       // Row with the longest segment: the relaxation probe.
};

// Everything one scale's rasters become on the device. Plain arrays: the Vulkan host
// uploads these verbatim and the reference walks the same indices.
struct BitProblem {
    std::uint32_t bit_count = 0;      // Bits across the frame width (a power of two).
    std::uint32_t words_per_row = 0;  // bit_count / 32.
    std::uint32_t canvas_rows = 0;    // Stripe rows the canvas covers.
    std::uint32_t island_count = 0;
    float frame_width = 0, frame_height = 0, stripe_height = 0;
    float bit_width = 0;              // frame_width / bit_count, in output units.
    float initial_occupied_width = 0, initial_occupied_height = 0;

    std::vector<std::uint64_t> island_ids;
    std::vector<std::uint32_t> orientation_begin; // Per island: first index in `orientations`.
    std::vector<std::uint32_t> orientation_count;
    std::vector<BitOrientation> orientations;
    std::vector<std::uint32_t> row_start;     // Per orientation row (+1 terminator): index into `intervals`.
    std::vector<std::uint32_t> row_first_bit; // Per orientation row: first interval begin, or `empty_row`.
    std::vector<BitInterval> intervals;

    std::size_t canvas_words() const {
        return std::size_t(canvas_rows) * words_per_row * band_planes;
    }
};

// Encode a scale's rasters. Throws when the problem cannot be represented (too many
// orientations, too many rows, an orientation wider than the frame is simply marked
// as not fitting rather than rejected).
BitProblem build_problem(const std::vector<RasterIsland>& rasters, const FixedTarget& target,
                         int bit_shift = default_bit_shift);

// The shader's canvas: `canvas_rows * words_per_row * band_planes` words, band-minor
// (eight bands plus their union), plus the per-row "last set bit + 1" column that the
// O(1) clear test and the exact left clearance both read.
class ReferenceCanvas {
public:
    explicit ReferenceCanvas(const BitProblem& problem);
    void clear();
    // First offset in [0, max_offset] at which the orientation placed with its first
    // row at `row` collides with nothing. `no_bit` when there is none. `max_offset`
    // comes from the trial's own frame, not the problem's.
    std::uint32_t first_clear(const BitProblem& p, std::uint32_t orientation, std::uint32_t row,
                              std::uint32_t max_offset) const;
    // The 32-lane clearance sum, in output units, exactly as the greedy cost needs it.
    float clearance_sum(const BitProblem& p, std::uint32_t orientation, std::uint32_t row,
                        std::uint32_t offset) const;
    void commit(const BitProblem& p, std::uint32_t orientation, std::uint32_t row, std::uint32_t offset);

    const std::vector<std::uint32_t>& words() const { return words_; }
    std::uint32_t row_max_end(std::uint32_t row) const { return row_max_end_[row]; }

private:
    // The displacement one island row needs at `offset`; 0 = clear, `no_bit` = never.
    std::uint32_t row_displacement(const BitProblem& p, std::uint32_t row_begin, std::uint32_t i,
                                   std::uint32_t canvas_row, std::uint32_t offset,
                                   std::uint32_t max_offset) const;
    // First offset at or above `start` where island row `i` alone is clear.
    std::uint32_t row_clear(const BitProblem& p, std::uint32_t row_begin, std::uint32_t i,
                            std::uint32_t row, std::uint32_t start, std::uint32_t max_offset) const;

    std::uint32_t words_per_row_ = 0, canvas_rows_ = 0, bit_count_ = 0;
    std::vector<std::uint32_t> words_;
    std::vector<std::uint32_t> row_max_end_;
    std::uint32_t or_bands(std::uint32_t row, std::uint32_t word, std::uint32_t lo, std::uint32_t hi) const;
    std::uint32_t or_all(std::uint32_t row, std::uint32_t word) const;
};

// The cost the kernel minimises: the location cost of placement.hpp evaluated in
// float, with a placement outside the frame returning FLT_MAX.
float reference_cost(const BitProblem& p, std::uint32_t orientation, std::uint32_t row,
                     std::uint32_t offset, float clearance, float occupied_width,
                     float occupied_height, float frame_width, float frame_height);

struct BitPlacement {
    std::uint32_t island = 0;      // Index into the raster set.
    std::uint32_t orientation = 0; // Index within the island.
    std::uint32_t row = 0;
    std::uint32_t x_bits = 0;
};

struct BitTrial {
    bool valid = false;
    std::vector<BitPlacement> placements;
    float occupied_width = 0, occupied_height = 0;
};

// One trial's usable frame inside the shared bit grid. `shrink` >= 1 packs the
// same islands into a frame that much smaller; scaling the layout back up to the
// target then multiplies the scale by `shrink` and grows every gap by the same factor,
// which is why a single upload can cover a whole ladder of scales (see trial.comp).
struct TrialFrame {
    double shrink = 1.0;
    float width(const BitProblem& p) const { return float(double(p.frame_width) / shrink); }
    float height(const BitProblem& p) const { return float(double(p.frame_height) / shrink); }
};

// Run one whole greedy trial the way one workgroup does: islands in `order`, all
// unmasked orientations, all candidate rows, first-win ties.
BitTrial reference_trial(const BitProblem& p, const std::vector<std::uint32_t>& order,
                         const std::vector<std::uint32_t>& skip_mask, TrialFrame frame = {});

} // namespace uvpack::gpu
