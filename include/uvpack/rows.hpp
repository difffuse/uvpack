#pragma once
// Rows of occupied width: the representation everything above the rasterizer works on.
//
// A shape is a stack of rows, one per horizontal stripe of the frame. A row is a
// sorted, disjoint list of x-intervals, in fixed point, each carrying the vertical
// band of its stripe that it actually fills. Two pieces collide only when they overlap
// in x AND in that band, which is what lets two islands share a stripe when one sits
// above the other inside it.
//
// Intervals are half-open in x: touching endpoints do not collide. The fixed-point
// grid is fine enough that the rasterizer's outward rounding (one unit each way) is
// far below any margin, so a layout that is interval-disjoint here is overlap-free in
// the continuous domain.
#include <climits>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace uvpack::rows {

using Coordinate = std::int32_t;
inline constexpr Coordinate units = 1048576; // 2^20 fixed-point units per output unit.

struct Interval {
    Coordinate begin, end;
    float y_low = 0.0f, y_high = 1.0f; // Fraction of the stripe's height that is filled.
};
inline bool y_overlap(const Interval& a, const Interval& b) { return a.y_low < b.y_high && b.y_low < a.y_high; }
inline bool full_height(const Interval& a) { return a.y_low <= 0.0f && a.y_high >= 1.0f; }

using Row = std::vector<Interval>;
using Shape = std::vector<Row>;

// Union one piece into a sorted, disjoint row. Where the piece overlaps something
// already there, only the shared x-range takes the union of the two height bands, so a
// piece stacked above another inside one stripe does not make the whole overlap solid.
// Touching pieces with the same band coalesce; the row stays sorted and disjoint.
void insert_piece(Row& row, Interval piece);

// Throws unless every row is sorted, disjoint, nonnegative and has positive lengths.
void validate(const Shape& shape);

// Decode a run-length row buffer laid out row-minor: plane 0 holds the row's value
// count (two per interval), plane 1 its first begin, then alternating interval length
// and the gap to the next one. `stride` is the distance between planes.
Shape decode(const std::vector<Coordinate>& buffer, std::size_t row_count, std::size_t stride);

struct Location {
    Coordinate offset_x;
    float left_margin_sum; // Total empty width left of the island's rows, in output units.
};

// The leftmost x at which `island`, placed with its row 0 on canvas row `row`, hits
// nothing. nullopt when that x would exceed `max_offset`. This is the reference
// implementation of the search: plain, obviously correct, and the baseline the indexed
// occupancy in fast_occupancy.hpp is tested against.
std::optional<Location> find_location(const Shape& canvas, const Shape& island,
                                      std::size_t row, Coordinate max_offset = INT32_MAX);
// The same search without re-validating both shapes first. Callers must keep the
// canvas valid (commit() does) and validate islands once when they are built.
std::optional<Location> find_location_unchecked(const Shape& canvas, const Shape& island,
                                                std::size_t row, Coordinate max_offset = INT32_MAX);

// Union the island's rows into the canvas, translated by `offset_x` and starting at
// canvas row `row`. The canvas grows to fit.
void commit(Shape& canvas, const Shape& island, std::size_t row, Coordinate offset_x);
void commit_unchecked(Shape& canvas, const Shape& island, std::size_t row, Coordinate offset_x);

} // namespace uvpack::rows
