#include "uvpack/gpu_reference.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uvpack::gpu {
namespace {

std::uint32_t find_lsb(std::uint32_t value) {
    // Matches GLSL findLSB for nonzero input.
    std::uint32_t i = 0;
    while ((value & 1u) == 0u) { value >>= 1; ++i; }
    return i;
}

std::uint32_t find_msb(std::uint32_t value) {
    std::uint32_t i = 31;
    while ((value >> i) == 0u) --i;
    return i;
}

// Bits [low, 32) of a word.
std::uint32_t mask_from(std::uint32_t low) { return low >= 32 ? 0u : (0xFFFFFFFFu << low); }
// Bits [0, high] of a word.
std::uint32_t mask_upto(std::uint32_t high) { return high >= 31 ? 0xFFFFFFFFu : ((1u << (high + 1)) - 1u); }

// Outward rounding onto the bit grid. `frame_units` is the frame width in the
// rows fixed point, `bit_count` the number of bits across it.
std::uint32_t bit_floor(std::int64_t x, std::int64_t frame_units, std::uint32_t bit_count) {
    if (x <= 0) return 0;
    const std::int64_t bit = (x * std::int64_t(bit_count)) / frame_units;
    return std::uint32_t(std::min<std::int64_t>(bit, bit_count));
}
std::uint32_t bit_ceil(std::int64_t x, std::int64_t frame_units, std::uint32_t bit_count) {
    if (x <= 0) return 0;
    const std::int64_t bit = (x * std::int64_t(bit_count) + frame_units - 1) / frame_units;
    return std::uint32_t(std::min<std::int64_t>(bit, bit_count));
}

} // namespace

BitProblem build_problem(const std::vector<RasterIsland>& rasters, const FixedTarget& target,
                         int bit_shift) {
    if (bit_shift < 6 || bit_shift > 16) throw std::invalid_argument("gpu bit_shift out of range");
    if (!(target.width > 0) || !(target.height > 0) || !(target.stripe_height > 0))
        throw std::invalid_argument("invalid gpu target");
    BitProblem p;
    p.bit_count = std::uint32_t(1) << bit_shift;
    p.words_per_row = p.bit_count / 32;
    p.frame_width = float(target.width);
    p.frame_height = float(target.height);
    p.stripe_height = float(target.stripe_height);
    p.bit_width = float(target.width / double(p.bit_count));
    p.initial_occupied_width = float(target.initial_occupied_width);
    p.initial_occupied_height = float(target.initial_occupied_height);
    p.island_count = std::uint32_t(rasters.size());

    // The frame in the rows fixed point; the same floor the CPU placer uses.
    const std::int64_t frame_units = std::int64_t(std::floor(target.width * rows::units));
    if (frame_units <= 0) throw std::invalid_argument("degenerate gpu frame");

    const std::int64_t last_target_row = std::int64_t(std::floor(target.height / target.stripe_height));
    std::uint32_t tallest = 0;
    for (const auto& island : rasters) {
        if (island.orientations.size() > max_orientations)
            throw std::invalid_argument("gpu kernel supports at most 32 orientations per island");
        p.island_ids.push_back(island.id);
        p.orientation_begin.push_back(std::uint32_t(p.orientations.size()));
        p.orientation_count.push_back(std::uint32_t(island.orientations.size()));
        for (const auto& orientation : island.orientations) {
            BitOrientation o;
            o.row_begin = std::uint32_t(p.row_start.size());
            o.row_count = std::uint32_t(orientation.rows.size());
            o.width = float(orientation.geometry.width);
            o.height = float(orientation.geometry.height);
            o.inv_height = 1.0f / o.height;
            tallest = std::max(tallest, o.row_count);
            // Frame fit: the same bounds the CPU placer enforces.
            if (orientation.geometry.width <= target.width && orientation.geometry.height <= target.height) {
                const double room = target.width - orientation.geometry.width;
                o.max_offset_bits = std::int32_t(std::floor(room / double(target.width) * double(p.bit_count)));
                o.max_offset_bits = std::min<std::int32_t>(o.max_offset_bits, std::int32_t(p.bit_count));
                o.last_row = std::int32_t(std::floor((target.height - orientation.geometry.height) / target.stripe_height));
                if (o.last_row < 0) { o.max_offset_bits = -1; o.last_row = -1; }
            }
            std::uint32_t longest = 0;
            for (const auto& row : orientation.rows) {
                const std::uint32_t row_index = std::uint32_t(p.row_start.size() - o.row_begin);
                p.row_start.push_back(std::uint32_t(p.intervals.size()));
                std::uint32_t first = empty_row;
                for (const auto interval : row) {
                    BitInterval bi;
                    bi.begin = bit_floor(interval.begin, frame_units, p.bit_count);
                    bi.end = bit_ceil(interval.end, frame_units, p.bit_count);
                    if (bi.end <= bi.begin) bi.end = bi.begin + 1; // Never lose a segment to rounding.
                    const float low = std::max(0.0f, std::min(1.0f, interval.y_low));
                    const float high = std::max(0.0f, std::min(1.0f, interval.y_high));
                    bi.band_lo = std::uint32_t(std::min(float(bands - 1), std::floor(low * float(bands))));
                    const float ceiling = std::ceil(high * float(bands));
                    bi.band_hi = std::uint32_t(std::max(1.0f, std::min(float(bands), ceiling))) - 1;
                    if (bi.band_hi < bi.band_lo) bi.band_hi = bi.band_lo;
                    if (first == empty_row) first = bi.begin;
                    // Outward rounding often makes neighbouring segments touch or overlap.
                    // Merging the ones that share bands keeps the row short without losing
                    // coverage; segments in different bands are left alone (they may then
                    // overlap in x, which the search and the commit both handle).
                    if (p.intervals.size() > p.row_start.back() &&
                        p.intervals.back().end >= bi.begin && p.intervals.back().band_lo == bi.band_lo &&
                        p.intervals.back().band_hi == bi.band_hi) {
                        p.intervals.back().end = std::max(p.intervals.back().end, bi.end);
                    } else {
                        p.intervals.push_back(bi);
                    }
                }
                p.row_first_bit.push_back(first);
                // The row that needs the most contiguous room is the relaxation probe:
                // its own first clear offset bounds the island's from below.
                for (auto k = p.row_start.back(); k < p.intervals.size(); ++k) {
                    const auto span = p.intervals[k].end - p.intervals[k].begin;
                    if (span > longest) { longest = span; o.probe_row = row_index; }
                }
            }
            p.row_start.push_back(std::uint32_t(p.intervals.size())); // Terminator.
            p.row_first_bit.push_back(empty_row);
            p.orientations.push_back(o);
        }
    }
    const std::int64_t rows = last_target_row + std::int64_t(tallest) + 1;
    if (rows <= 0 || rows > std::int64_t(target.max_candidate_rows) + std::int64_t(tallest) + 1)
        throw std::length_error("gpu canvas row count exceeds the candidate-row limit");
    p.canvas_rows = std::uint32_t(rows);
    return p;
}

ReferenceCanvas::ReferenceCanvas(const BitProblem& problem)
    : words_per_row_(problem.words_per_row), canvas_rows_(problem.canvas_rows),
      bit_count_(problem.bit_count) {
    words_.assign(problem.canvas_words(), 0u);
    row_max_end_.assign(problem.canvas_rows, 0u);
}

void ReferenceCanvas::clear() {
    std::fill(words_.begin(), words_.end(), 0u);
    std::fill(row_max_end_.begin(), row_max_end_.end(), 0u);
}

std::uint32_t ReferenceCanvas::or_bands(std::uint32_t row, std::uint32_t word,
                                        std::uint32_t lo, std::uint32_t hi) const {
    const std::size_t base = (std::size_t(row) * words_per_row_ + word) * band_planes;
    if (lo == 0 && hi == bands - 1) return words_[base + bands]; // The stored union plane.
    std::uint32_t acc = 0;
    for (std::uint32_t b = lo; b <= hi; ++b) acc |= words_[base + b];
    return acc;
}

std::uint32_t ReferenceCanvas::or_all(std::uint32_t row, std::uint32_t word) const {
    return or_bands(row, word, 0, bands - 1);
}

namespace {
std::uint32_t band_word(const std::vector<std::uint32_t>& words, std::uint32_t words_per_row,
                        std::uint32_t row, std::uint32_t word, std::uint32_t bl, std::uint32_t bh) {
    const std::size_t base = (std::size_t(row) * words_per_row + word) * band_planes;
    if (bl == 0 && bh == bands - 1) return words[base + bands]; // The stored union plane.
    std::uint32_t acc = 0;
    for (std::uint32_t b = bl; b <= bh; ++b) acc |= words[base + b];
    return acc;
}

// First occupied bit in [from, limit) (bands bl..bh), or `no_bit`. Callers pass the
// row's last occupied bit as the limit: nothing is set at or beyond it, so a search
// that has run past the occupancy stops instead of walking the whole frame.
std::uint32_t scan_forward(const std::vector<std::uint32_t>& words, std::uint32_t words_per_row,
                           std::uint32_t row, std::uint32_t bl, std::uint32_t bh,
                           std::uint32_t from, std::uint32_t limit) {
    if (from >= limit) return no_bit;
    std::uint32_t w = from >> 5;
    const std::uint32_t last = (limit - 1) >> 5;
    std::uint32_t acc = band_word(words, words_per_row, row, w, bl, bh) & mask_from(from & 31u);
    while (true) {
        if (w == last) acc &= ((limit & 31u) == 0u) ? 0xFFFFFFFFu : mask_upto((limit - 1) & 31u);
        if (acc != 0u) return (w << 5) + find_lsb(acc);
        if (w >= last) return no_bit;
        ++w;
        acc = band_word(words, words_per_row, row, w, bl, bh);
    }
}

// First CLEAR bit at or after `from` (bands bl..bh). Everything at or beyond the row's
// last occupied bit `end` is free, so that is both the bound and the answer past it.
std::uint32_t scan_clear(const std::vector<std::uint32_t>& words, std::uint32_t words_per_row,
                         std::uint32_t row, std::uint32_t bl, std::uint32_t bh,
                         std::uint32_t from, std::uint32_t end) {
    if (from >= end) return from;
    std::uint32_t w = from >> 5;
    const std::uint32_t last = (end - 1) >> 5;
    std::uint32_t inv = ~band_word(words, words_per_row, row, w, bl, bh) & mask_from(from & 31u);
    while (inv == 0u) {
        if (w >= last) return end;
        ++w;
        inv = ~band_word(words, words_per_row, row, w, bl, bh);
    }
    return std::min(end, (w << 5) + find_lsb(inv));
}
} // namespace

// The displacement one island row needs at `offset`, 0 when it is clear there, and
// `no_bit` when it can never be clear at or below `max_offset`. `row_begin` and the
// canvas row are passed in rather than re-read from the orientation: the shader pays
// dearly for that load and the two must run the same loop.
std::uint32_t ReferenceCanvas::row_displacement(const BitProblem& p, std::uint32_t row_begin,
                                                std::uint32_t i, std::uint32_t canvas_row,
                                                std::uint32_t offset, std::uint32_t max_offset) const {
    const std::uint32_t first = p.row_first_bit[row_begin + i];
    if (first == empty_row) return 0;
    // One load settles both the "cannot collide" case and the scan bounds below.
    const std::uint32_t end = row_max_end_[canvas_row];
    if (first + offset >= end) return 0;
    const std::uint32_t begin = p.row_start[row_begin + i];
    const std::uint32_t stop = p.row_start[row_begin + i + 1];
    for (std::uint32_t k = begin; k < stop; ++k) {
        const auto iv = p.intervals[k];
        const std::uint32_t lo = iv.begin + offset;
        const std::uint32_t hi = std::min(iv.end + offset, end);
        const std::uint32_t hit = scan_forward(words_, words_per_row_, canvas_row,
                                               iv.band_lo, iv.band_hi, lo, hi);
        if (hit == no_bit) continue;
        // Clear the run of occupied bits this segment landed on, then skip any
        // following free run shorter than the segment: it cannot hold it either.
        const std::uint32_t length = iv.end - iv.begin;
        std::uint32_t q = scan_clear(words_, words_per_row_, canvas_row, iv.band_lo, iv.band_hi, hit, end);
        while (true) {
            const std::uint32_t next = scan_forward(words_, words_per_row_, canvas_row,
                                                    iv.band_lo, iv.band_hi, q, end);
            if (next == no_bit || next - q >= length) break;
            q = scan_clear(words_, words_per_row_, canvas_row, iv.band_lo, iv.band_hi, next, end);
        }
        const std::uint32_t next_offset = q - iv.begin;
        if (next_offset > max_offset || next_offset <= offset) return no_bit;
        return next_offset - offset;
    }
    return 0;
}

// The relaxation: the first offset at or above `start` where island row `i` alone is
// clear. Its answer bounds the full search from below, and its failure proves the
// whole island cannot sit in this row.
std::uint32_t ReferenceCanvas::row_clear(const BitProblem& p, std::uint32_t row_begin,
                                         std::uint32_t i, std::uint32_t row, std::uint32_t start,
                                         std::uint32_t max_offset) const {
    std::uint32_t offset = start;
    while (true) {
        const std::uint32_t jump = row_displacement(p, row_begin, i, row + i, offset, max_offset);
        if (jump == no_bit) return no_bit;
        if (jump == 0) return offset;
        offset += jump;
        if (offset > max_offset) return no_bit;
    }
}

std::uint32_t ReferenceCanvas::first_clear(const BitProblem& p, std::uint32_t orientation,
                                           std::uint32_t row, std::uint32_t max_offset) const {
    const auto& o = p.orientations[orientation];
    const std::uint32_t row_begin = o.row_begin;
    // Island rows above the occupancy can never collide, so they leave the sweep.
    const std::uint32_t count = row < canvas_rows_ ? std::min(o.row_count, canvas_rows_ - row) : 0;
    if (count == 0) return 0;
    std::uint32_t offset = o.probe_row < count
        ? row_clear(p, row_begin, o.probe_row, row, 0, max_offset) : 0;
    if (offset == no_bit) return no_bit;
    std::uint32_t witness = no_bit;
    // Every collision moves the offset strictly right, so the loop is bounded.
    for (std::uint32_t guard = 0; guard <= max_offset + 1; ++guard) {
        // A stack of island rows sliding along one obstacle keeps colliding in the same
        // place, so re-probing the row that blocked last time usually skips the sweep.
        if (witness != no_bit) {
            const std::uint32_t jump = row_displacement(p, row_begin, witness, row + witness, offset, max_offset);
            if (jump == no_bit) return no_bit;
            if (jump > 0) {
                offset += jump;
                if (offset > max_offset) return no_bit;
                continue;
            }
        }
        bool collided = false;
        for (std::uint32_t i = 0; i < count; ++i) {
            if (i == witness) continue;
            const std::uint32_t jump = row_displacement(p, row_begin, i, row + i, offset, max_offset);
            if (jump == no_bit) return no_bit;
            if (jump == 0) continue;
            witness = i;
            offset += jump;
            if (offset > max_offset) return no_bit;
            collided = true;
            break;
        }
        if (!collided) return offset;
    }
    return no_bit;
}

float ReferenceCanvas::clearance_sum(const BitProblem& p, std::uint32_t orientation,
                                     std::uint32_t row, std::uint32_t offset) const {
    const auto& o = p.orientations[orientation];
    float lanes[32] = {};
    for (std::uint32_t i = 0; i < o.row_count; ++i) {
        const std::uint32_t first = p.row_first_bit[o.row_begin + i];
        if (first == empty_row) continue; // An empty island row has no left neighbour.
        const std::uint32_t left = first + offset;
        std::uint32_t previous_end = 0;
        const std::uint32_t canvas_row = row + i;
        if (canvas_row < canvas_rows_ && left > 0) {
            const std::uint32_t end = row_max_end_[canvas_row];
            if (left >= end) {
                previous_end = end; // The row's last occupied bit IS end - 1, so this is exact.
            } else {
                // Last occupied bit strictly below `left`, over every band, +1.
                std::int64_t w = std::int64_t((std::min(left, bit_count_) - 1) >> 5);
                std::uint32_t acc = or_all(canvas_row, std::uint32_t(w)) &
                                    mask_upto((std::min(left, bit_count_) - 1) & 31u);
                while (acc == 0u && w > 0) { --w; acc = or_all(canvas_row, std::uint32_t(w)); }
                if (acc != 0u) previous_end = (std::uint32_t(w) << 5) + find_msb(acc) + 1;
            }
        }
        lanes[i & 31u] = float(left - previous_end) * p.bit_width + lanes[i & 31u];
    }
    float sum = 0;
    for (float lane : lanes) sum += lane;
    return sum;
}

void ReferenceCanvas::commit(const BitProblem& p, std::uint32_t orientation, std::uint32_t row,
                             std::uint32_t offset) {
    const auto& o = p.orientations[orientation];
    for (std::uint32_t i = 0; i < o.row_count; ++i) {
        const std::uint32_t canvas_row = row + i;
        if (canvas_row >= canvas_rows_) break;
        const std::uint32_t begin = p.row_start[o.row_begin + i];
        const std::uint32_t end = p.row_start[o.row_begin + i + 1];
        for (std::uint32_t k = begin; k < end; ++k) {
            const auto iv = p.intervals[k];
            const std::uint32_t lo = std::min(iv.begin + offset, bit_count_);
            const std::uint32_t hi = std::min(iv.end + offset, bit_count_);
            if (hi <= lo) continue;
            for (std::uint32_t bit = lo; bit < hi;) {
                const std::uint32_t w = bit >> 5;
                const std::uint32_t top = std::min(hi, (w + 1) << 5);
                std::uint32_t mask = mask_from(bit & 31u);
                if ((top & 31u) != 0u) mask &= mask_upto((top - 1) & 31u);
                const std::size_t base = (std::size_t(canvas_row) * words_per_row_ + w) * band_planes;
                for (std::uint32_t b = iv.band_lo; b <= iv.band_hi; ++b) words_[base + b] |= mask;
                words_[base + bands] |= mask;
                bit = top;
            }
            row_max_end_[canvas_row] = std::max(row_max_end_[canvas_row], hi);
        }
    }
}

float reference_cost(const BitProblem& p, std::uint32_t orientation, std::uint32_t row,
                     std::uint32_t offset, float clearance, float occupied_width,
                     float occupied_height, float frame_width, float frame_height) {
    const auto& o = p.orientations[orientation];
    const float right = float(offset) * p.bit_width + o.width;
    const float top = float(row) * p.stripe_height + o.height;
    if (right > frame_width || top > frame_height) return std::numeric_limits<float>::max();
    // The cost rule of placement.hpp: canvas growth as a fraction of the frame first,
    // bottom-left preference second, both multiplied through by the frame's area so
    // that nothing here has to divide. Plain multiplies and adds, in this order: the
    // kernel writes the same expression and the two must agree to the bit.
    const float dx = std::fmax(right - occupied_width, 0.0f);
    const float dy = std::fmax(top - occupied_height, 0.0f);
    const float growth = std::fmax(dx * frame_height, dy * frame_width);
    const float mean_clearance = (clearance * p.stripe_height) * o.inv_height;
    const float settle = (float(row) * p.stripe_height) * frame_width + mean_clearance * frame_height;
    return 100000.0f * growth + settle;
}

BitTrial reference_trial(const BitProblem& p, const std::vector<std::uint32_t>& order,
                         const std::vector<std::uint32_t>& skip_mask, TrialFrame frame) {
    BitTrial trial;
    if (order.size() != p.island_count || skip_mask.size() != p.island_count)
        throw std::invalid_argument("gpu trial plan must cover every island");
    if (!(frame.shrink >= 1.0)) throw std::invalid_argument("gpu trial frame must not grow");
    const float frame_width = frame.width(p), frame_height = frame.height(p);
    ReferenceCanvas canvas(p);
    float occupied_width = p.initial_occupied_width, occupied_height = p.initial_occupied_height;
    for (const auto island : order) {
        if (island >= p.island_count) throw std::invalid_argument("gpu trial order out of range");
        float best_cost = std::numeric_limits<float>::infinity();
        std::uint32_t best_orientation = 0, best_row = 0, best_x = 0;
        bool found = false;
        const std::uint32_t base = p.orientation_begin[island];
        for (std::uint32_t o = 0; o < p.orientation_count[island]; ++o) {
            if (skip_mask[island] & (1u << o)) continue;
            const auto& orientation = p.orientations[base + o];
            // The frame is per trial, so the two fit bounds are too.
            if (orientation.width > frame_width || orientation.height > frame_height) continue;
            const std::int32_t last_row_signed =
                std::int32_t(std::floor((frame_height - orientation.height) / p.stripe_height));
            if (last_row_signed < 0) continue;
            const std::int32_t last_row = std::min(last_row_signed, std::int32_t(p.canvas_rows) - 1);
            const std::int32_t max_offset_signed =
                std::int32_t(std::floor((frame_width - orientation.width) / p.bit_width));
            if (max_offset_signed < 0) continue;
            const std::uint32_t max_offset = std::min(std::uint32_t(max_offset_signed), p.bit_count);
            for (std::int32_t row = 0; row <= last_row; ++row) {
                // Exact lower bound of the cost for any clear offset in this row, and it
                // only grows with the row: once it cannot win, no higher row can either.
                const float y = float(row) * p.stripe_height;
                const float lower = frame_width *
                    (100000.0f * std::fmax(0.0f, y + orientation.height - occupied_height) + y);
                if (!(lower < best_cost)) break;
                const std::uint32_t offset = canvas.first_clear(p, base + o, std::uint32_t(row), max_offset);
                if (offset == no_bit) continue;
                const float clearance = canvas.clearance_sum(p, base + o, std::uint32_t(row), offset);
                const float cost = reference_cost(p, base + o, std::uint32_t(row), offset, clearance,
                                                  occupied_width, occupied_height, frame_width, frame_height);
                if (cost == std::numeric_limits<float>::max()) continue;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_orientation = o;
                    best_row = std::uint32_t(row);
                    best_x = offset;
                    found = true;
                }
            }
        }
        if (!found) { trial.valid = false; return trial; }
        const auto& chosen = p.orientations[base + best_orientation];
        canvas.commit(p, base + best_orientation, best_row, best_x);
        occupied_width = std::fmax(occupied_width, float(best_x) * p.bit_width + chosen.width);
        occupied_height = std::fmax(occupied_height, float(best_row) * p.stripe_height + chosen.height);
        trial.placements.push_back({island, best_orientation, best_row, best_x});
    }
    trial.valid = trial.placements.size() == p.island_count;
    trial.occupied_width = occupied_width;
    trial.occupied_height = occupied_height;
    return trial;
}

} // namespace uvpack::gpu
