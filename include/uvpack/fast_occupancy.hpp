#pragma once
// The greedy loop's trial occupancy with the indexes its search needs.
//
// The occupancy itself stays exactly the `rows::Shape` the rest of the code
// (commit, refine, the validator, the tests) works with. What this class adds are
// per-row columns that let the inner loop answer most questions without touching a
// single interval, and a search built around them:
//
//   * `max_end`  the row's last interval end. An island row whose left edge is at or
//                past it can neither collide there nor have any nearer left neighbour,
//                so both the collision test and the exact left clearance are O(1) and
//                read only contiguous memory.
//   * `index`    binned cursor table: the first interval of the row that ends past
//                each 2^bin_shift slice of the frame, so positioning a segment is one
//                table read plus, on average, less than one step forward.
//   * `gap`      the longest free run between full-height pieces (the free-gap table
//                the placer already kept), used to reject candidate rows outright.
//   * `prefix`   running sums of `max_end`, which turn the lower bound on an island's
//                clearance sum into two lookups (see left_margin_window_bound).
//
// The search answers exactly what `rows::find_location_unchecked` answers: the
// same first feasible offset and the same clearance sum. It gets there differently.
// Each island row's required displacement rules out a whole interval of offsets, so a
// search may jump by any one of them without skipping a feasible offset; this one
// jumps on the first blocking row it finds and then re-probes that row (and its
// neighbour) first, because a stack of rows sliding along one obstacle keeps colliding
// in the same place. Rows already verified carry the offset at which they could next
// collide, so a jump only re-probes the rows it can have invalidated.
#include "uvpack/rows.hpp"
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

namespace uvpack {

// One orientation's rows flattened into a single array, plus the two per-row
// columns the search reads before it ever looks at an interval. Islands are built
// once per scale, so the greedy loop walks them linearly instead of chasing one
// heap block per row.
struct FastIsland {
    std::vector<rows::Interval> intervals;
    std::vector<std::uint32_t> starts;             // rows + 1 offsets into `intervals`.
    std::vector<rows::Coordinate> first_begin; // Row's first begin, sentinel when empty.
    std::vector<rows::Coordinate> limits;      // Initial per-row collision limits.
    std::size_t rows() const { return starts.empty() ? 0 : starts.size() - 1; }
    void build(const rows::Shape& shape);
};

class FastOccupancy {
public:
    using Coordinate = rows::Coordinate;
    explicit FastOccupancy(Coordinate frame_units = rows::units);

    void reset(Coordinate frame_units);
    void clear();

    const rows::Shape& shape() const { return rows_; }
    // Longest free run between full-height pieces inside the frame (frame width for
    // rows that carry nothing). Partial-height pieces may still be passable in y.
    Coordinate row_gap(std::size_t row) const { return row < gap_.size() ? gap_[row] : frame_units_; }

    // Commit `island` translated by `offset` with its first row at `first_row`,
    // keeping every derived table in sync. Same semantics as rows::commit_unchecked.
    void commit(const rows::Shape& island, std::size_t first_row, Coordinate offset);

    // First offset in [start, max_offset] at which `island` placed at `row` collides
    // with nothing, or nullopt. `start` must be an offset below which no placement is
    // feasible; `scratch` is the caller's per-row reuse buffer. `abandon(offset)` is
    // consulted after every jump: offsets only grow, so a caller whose cost bound is
    // monotone in the offset can stop a search that can no longer pay off.
    template <class Abandon>
    std::optional<Coordinate> first_clear(const FastIsland& island, std::size_t row,
                                          Coordinate max_offset,
                                          std::vector<Coordinate>& scratch, Coordinate start,
                                          Abandon&& abandon) const {
        if (max_offset < 0 || start < 0 || row > (std::size_t(-1) - island.rows()))
            throw std::invalid_argument("invalid rows search bounds");
        const std::size_t count = island.rows();
        // scratch[i] is the smallest offset at which island row i can collide again;
        // rows still below it are known clear and are skipped after a jump. Empty rows
        // and rows past the occupancy start out clear for good.
        scratch.assign(island.limits.begin(), island.limits.end());
        const std::size_t known = row < rows_.size() ? std::min(count, rows_.size() - row) : 0;
        for (std::size_t i = known; i < count; ++i) scratch[i] = coordinate_limit;
        const Coordinate* first_begin = island.first_begin.data();
        const Coordinate* ends = max_end_.data() + std::min(row, max_end_.size());
        const rows::Interval* intervals = island.intervals.data();
        const std::uint32_t* starts = island.starts.data();
        Coordinate offset = start;
        std::size_t witness = count; // Row that forced the last jump; it usually forces the next.
        const auto examine = [&](std::size_t i, Coordinate& limit) -> std::int64_t {
            // Both columns are contiguous: a row whose left edge is at or past the
            // occupancy's last end is settled without touching a single interval.
            if (std::int64_t(first_begin[i]) + offset >= std::int64_t(ends[i])) {
                limit = coordinate_limit;
                return 0;
            }
            return probe_row(row + i, intervals + starts[i], intervals + starts[i + 1], offset, limit);
        };
        while (true) {
            if (witness < count) {
                // The row that blocked last time, then its neighbour: a stack of rows
                // slides along the same obstacle, so these two carry most collisions.
                bool jumped = false;
                for (std::size_t w = witness; w <= witness + 1 && w < known; ++w) {
                    if (scratch[w] > offset) continue;
                    Coordinate limit = 0;
                    const auto jump = examine(w, limit);
                    if (jump > 0) {
                        witness = w;
                        if (!advance(offset, jump, max_offset) || abandon(offset)) return std::nullopt;
                        jumped = true;
                        break;
                    }
                    scratch[w] = limit;
                }
                if (jumped) continue;
            }
            // Sweep for a blocking row and hand it straight to the witness loop: a sweep
            // that finds one costs only the rows before it, and a sweep that finds none
            // is the proof that this offset is clear.
            std::int64_t best = 0;
            std::size_t blocker = count;
            for (std::size_t i = 0; i < known; ++i) {
                if (scratch[i] > offset) continue;
                Coordinate limit = 0;
                const auto jump = examine(i, limit);
                if (jump > 0) { best = jump; blocker = i; break; }
                scratch[i] = limit;
            }
            if (best == 0) return offset;
            witness = blocker;
            if (!advance(offset, best, max_offset) || abandon(offset)) return std::nullopt;
        }
    }
    std::optional<Coordinate> first_clear(const FastIsland& island, std::size_t row,
                                          Coordinate max_offset,
                                          std::vector<Coordinate>& scratch, Coordinate start = 0) const {
        return first_clear(island, row, max_offset, scratch, start,
                           [](Coordinate) { return false; });
    }
    // Same for a single island row against a single canvas row: a relaxation of the
    // full search, so its answer is a lower bound on the full search's offset and its
    // failure proves the whole island cannot be placed in this row.
    std::optional<Coordinate> row_first_clear(const FastIsland& island, std::size_t index,
                                              std::size_t row, Coordinate start,
                                              Coordinate max_offset) const;
    // Exact sum of the per-row left clearances, accumulated in fixed point.
    float left_margin_sum(const FastIsland& island, std::size_t row, Coordinate offset) const;
    // Lower bound of that sum that needs no interval lookups: a row whose left edge
    // is at or past the row's last end contributes its exact clearance, every other
    // row contributes zero (its true clearance is nonnegative). `first_begin` holds
    // each island row's first interval begin, or a large negative sentinel if empty.
    double left_margin_lower_bound(const std::vector<Coordinate>& first_begin, std::size_t row,
                                   Coordinate offset) const;
    // O(1) relaxation of that bound: dropping the per-row clamp leaves the sum of the
    // rows' last ends, which a prefix table answers in constant time. `sum_begin` and
    // `nonempty` describe the island's nonempty rows and `empty_rows` lists the rest,
    // whose last ends have to come back out of the window.
    double left_margin_window_bound(std::int64_t sum_begin, std::size_t nonempty,
                                    const std::vector<std::uint32_t>& empty_rows, std::size_t row,
                                    std::size_t count, Coordinate offset) const;
    // Sentinel for empty island rows in `first_begin` (contributes nothing to the bound).
    static constexpr Coordinate empty_row_begin = -(Coordinate(1) << 30);
    // Marks a row that can no longer collide at any larger offset.
    static constexpr Coordinate coordinate_limit = 2147483647;

private:
    static constexpr std::size_t coordinate_hint_max = 65535; // Cursor table saturates; the scan corrects it.
    std::size_t bin_count_ = 0;
    int bin_shift_ = 0;
    Coordinate frame_units_ = rows::units;
    rows::Shape rows_;
    std::vector<std::uint16_t> index_; // bin_count_ cursor hints per row.
    std::vector<Coordinate> max_end_;
    std::vector<Coordinate> gap_;
    std::vector<std::int64_t> prefix_end_; // prefix_end_[r] = sum of max_end_ below row r.

    void rebuild_prefix(std::size_t from);

    void refresh(std::size_t row);
    void ensure_rows(std::size_t count);
    // First interval of `row` whose end is past `x`, never before `from`.
    std::size_t locate(std::size_t row, std::int64_t x, std::size_t from) const;
    // Displacement one island row needs at `offset`, or 0 when it is clear there; a
    // clear row also reports the smallest offset at which it can collide again. The
    // caller has already ruled out the empty and past-the-last-end cases.
    std::int64_t probe_row(std::size_t canvas_row, const rows::Interval* first,
                           const rows::Interval* last, Coordinate offset, Coordinate& limit) const;
    // Apply a jump; false when it leaves the bound.
    static bool advance(Coordinate& offset, std::int64_t jump, Coordinate max_offset);
};

} // namespace uvpack
