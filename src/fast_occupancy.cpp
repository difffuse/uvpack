#include "uvpack/fast_occupancy.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uvpack {
namespace {
rows::Coordinate checked(std::int64_t value) {
    if (value < 0 || value > std::numeric_limits<rows::Coordinate>::max()) throw std::overflow_error("rows coordinate overflow");
    return static_cast<rows::Coordinate>(value);
}
// Bins per frame width for the cursor table. Small on purpose: the whole table has
// to stay in the first-level cache, and one bin holds only a couple of intervals.
constexpr std::size_t target_bins = 16;
} // namespace

void FastIsland::build(const rows::Shape& shape) {
    intervals.clear(); starts.clear(); first_begin.clear(); limits.clear();
    starts.reserve(shape.size() + 1);
    first_begin.reserve(shape.size());
    limits.reserve(shape.size());
    starts.push_back(0);
    for (const auto& row : shape) {
        intervals.insert(intervals.end(), row.begin(), row.end());
        starts.push_back(static_cast<std::uint32_t>(intervals.size()));
        first_begin.push_back(row.empty() ? FastOccupancy::empty_row_begin : row.front().begin);
        limits.push_back(row.empty() ? FastOccupancy::coordinate_limit : 0);
    }
}

FastOccupancy::FastOccupancy(Coordinate frame_units) { reset(frame_units); }

void FastOccupancy::reset(Coordinate frame_units) {
    if (frame_units <= 0) throw std::invalid_argument("frame width must be positive");
    frame_units_ = frame_units;
    bin_shift_ = 0;
    while ((frame_units_ >> bin_shift_) > static_cast<Coordinate>(target_bins)) ++bin_shift_;
    bin_count_ = static_cast<std::size_t>(frame_units_ >> bin_shift_) + 2;
    clear();
}

void FastOccupancy::clear() {
    rows_.clear();
    index_.clear();
    max_end_.clear();
    gap_.clear();
    prefix_end_.assign(1, 0);
}

void FastOccupancy::ensure_rows(std::size_t count) {
    if (count <= rows_.size()) return;
    const std::int64_t last = prefix_end_.empty() ? 0 : prefix_end_.back();
    rows_.resize(count);
    max_end_.resize(count, 0);
    gap_.resize(count, frame_units_);
    index_.resize(count * bin_count_, 0);
    prefix_end_.resize(count + 1, last); // New rows carry nothing yet.
}

void FastOccupancy::rebuild_prefix(std::size_t from) {
    if (prefix_end_.size() != rows_.size() + 1) prefix_end_.assign(rows_.size() + 1, 0);
    for (std::size_t r = from; r < rows_.size(); ++r) prefix_end_[r + 1] = prefix_end_[r] + max_end_[r];
}

void FastOccupancy::refresh(std::size_t row) {
    const auto& intervals = rows_[row];
    max_end_[row] = intervals.empty() ? 0 : intervals.back().end;
    // Binned cursor: index[c] is the first interval ending past the bin's start.
    std::uint16_t* bins = index_.data() + row * bin_count_;
    std::size_t k = 0;
    for (std::size_t c = 0; c < bin_count_; ++c) {
        const std::int64_t start = static_cast<std::int64_t>(c) << bin_shift_;
        while (k < intervals.size() && std::int64_t(intervals[k].end) <= start) ++k;
        bins[c] = static_cast<std::uint16_t>(std::min<std::size_t>(k, coordinate_hint_max));
    }
    // Longest full-height free run; partial pieces may still be passable in y.
    Coordinate best = 0, cursor = 0;
    for (const auto iv : intervals) {
        if (!rows::full_height(iv)) continue;
        best = std::max(best, iv.begin - cursor);
        cursor = std::max(cursor, iv.end);
    }
    gap_[row] = std::max(best, frame_units_ - cursor);
}

void FastOccupancy::commit(const rows::Shape& island, std::size_t first_row, Coordinate offset) {
    if (offset < 0 || first_row > rows_.max_size() || island.size() > rows_.max_size() - first_row)
        throw std::invalid_argument("invalid rows commit bounds");
    // Validate the translation before touching occupancy, exactly as commit_unchecked does.
    for (const auto& source : island)
        for (const auto interval : source) checked(std::int64_t(interval.end) + offset);
    ensure_rows(std::max(rows_.size(), first_row + island.size()));
    for (std::size_t i = 0; i < island.size(); ++i) {
        for (const auto interval : island[i])
            rows::insert_piece(rows_[first_row + i],
                {checked(std::int64_t(interval.begin) + offset), checked(std::int64_t(interval.end) + offset),
                 interval.y_low, interval.y_high});
        refresh(first_row + i);
    }
    rebuild_prefix(first_row);
}

std::size_t FastOccupancy::locate(std::size_t row, std::int64_t x, std::size_t from) const {
    const auto& intervals = rows_[row];
    std::size_t bin = x <= 0 ? 0 : static_cast<std::size_t>(x >> bin_shift_);
    if (bin >= bin_count_) bin = bin_count_ - 1;
    std::size_t k = std::max<std::size_t>(from, index_[row * bin_count_ + bin]);
    while (k < intervals.size() && std::int64_t(intervals[k].end) <= x) ++k;
    return k;
}

std::int64_t FastOccupancy::probe_row(std::size_t canvas_row, const rows::Interval* first,
                                      const rows::Interval* last, Coordinate offset,
                                      Coordinate& limit) const {
    limit = coordinate_limit;
    const auto& occupied = rows_[canvas_row];
    const std::size_t n = occupied.size();
    if (n == 0) return 0;
    std::int64_t next_touch = coordinate_limit;
    std::size_t k = 0;
    for (const rows::Interval* it = first; it != last; ++it) {
        const rows::Interval segment = *it;
        const std::int64_t left = std::int64_t(segment.begin) + offset;
        const std::int64_t right = std::int64_t(segment.end) + offset;
        k = locate(canvas_row, left, k);
        if (k == n) break;
        std::size_t j = k;
        for (; j < n && std::int64_t(occupied[j].begin) < right; ++j) {
            if (!rows::y_overlap(occupied[j], segment)) continue;
            // A gap shorter than this segment cannot hold it either: landing the
            // segment's left edge there puts its right edge inside the next piece. So
            // the jump runs on past every such gap in one step.
            const auto length = segment.end - segment.begin;
            std::size_t m = j;
            while (m + 1 < n && occupied[m + 1].begin - occupied[m].end < length &&
                   rows::y_overlap(occupied[m + 1], segment)) ++m;
            return std::int64_t(occupied[m].end) - left;
        }
        // Clear: this segment cannot meet occupancy before its right edge passes the
        // next interval's begin. Ignoring that interval's band only lowers the bound.
        if (j < n) next_touch = std::min(next_touch, std::int64_t(occupied[j].begin) - segment.end + 1);
    }
    limit = static_cast<Coordinate>(std::min<std::int64_t>(next_touch, coordinate_limit));
    return 0;
}

bool FastOccupancy::advance(Coordinate& offset, std::int64_t jump, Coordinate max_offset) {
    const auto next = std::int64_t(offset) + jump;
    if (next > max_offset) return false;
    try { offset = checked(next); }
    catch (const std::overflow_error&) { return false; }
    return offset <= max_offset;
}

std::optional<rows::Coordinate> FastOccupancy::row_first_clear(const FastIsland& island, std::size_t index,
                                                                   std::size_t row, Coordinate start,
                                                                   Coordinate max_offset) const {
    const std::size_t target = row + index;
    if (target >= rows_.size() || island.starts[index] == island.starts[index + 1]) return start;
    const rows::Interval* first = island.intervals.data() + island.starts[index];
    const rows::Interval* last = island.intervals.data() + island.starts[index + 1];
    Coordinate offset = start;
    Coordinate limit = 0;
    while (true) {
        if (std::int64_t(island.first_begin[index]) + offset >= std::int64_t(max_end_[target])) return offset;
        const auto jump = probe_row(target, first, last, offset, limit);
        if (jump == 0) return offset;
        if (!advance(offset, jump, max_offset)) return std::nullopt;
    }
}

float FastOccupancy::left_margin_sum(const FastIsland& island, std::size_t row, Coordinate offset) const {
    std::int64_t total = 0;
    const std::size_t count = island.rows();
    for (std::size_t i = 0; i < count; ++i) {
        if (island.starts[i] != island.starts[i + 1]) {
            const std::int64_t left = std::int64_t(island.first_begin[i]) + offset;
            std::int64_t previous_end = 0;
            const std::size_t target = row + i;
            if (target < rows_.size() && !rows_[target].empty()) {
                if (left >= std::int64_t(max_end_[target])) previous_end = max_end_[target];
                else {
                    const auto k = locate(target, left, 0);
                    previous_end = k == 0 ? 0 : rows_[target][k - 1].end;
                }
            }
            total += left - previous_end;
        }
    }
    return static_cast<float>(total) * (1.0f / rows::units);
}

double FastOccupancy::left_margin_window_bound(std::int64_t sum_begin, std::size_t nonempty,
                                               const std::vector<std::uint32_t>& empty_rows, std::size_t row,
                                               std::size_t count, Coordinate offset) const {
    const std::size_t rows = rows_.size();
    const std::size_t lo = std::min(row, rows), hi = std::min(row + count, rows);
    std::int64_t window = prefix_end_[hi] - prefix_end_[lo];
    for (const auto i : empty_rows) { // These rows contribute nothing to the clearance sum.
        const std::size_t target = row + i;
        if (target < rows) window -= max_end_[target];
    }
    const std::int64_t sum = std::int64_t(nonempty) * std::int64_t(offset) + sum_begin - window;
    return sum > 0 ? double(sum) / rows::units : 0.0;
}

double FastOccupancy::left_margin_lower_bound(const std::vector<Coordinate>& first_begin, std::size_t row,
                                              Coordinate offset) const {
    const std::size_t count = first_begin.size();
    const std::size_t known = row < max_end_.size() ? std::min(count, max_end_.size() - row) : 0;
    double sum = 0;
    const Coordinate* ends = max_end_.data() + row;
    for (std::size_t i = 0; i < known; ++i) {
        const double slack = double(first_begin[i]) + double(offset) - double(ends[i]);
        sum += slack > 0 ? slack : 0;
    }
    for (std::size_t i = known; i < count; ++i) { // Rows past the occupancy: nothing to the left.
        const double slack = double(first_begin[i]) + double(offset);
        sum += slack > 0 ? slack : 0;
    }
    return sum / rows::units;
}

} // namespace uvpack
