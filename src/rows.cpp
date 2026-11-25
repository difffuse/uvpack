#include "uvpack/rows.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace uvpack::rows {
namespace {
Coordinate checked(std::int64_t value) {
    if (value < 0 || value > INT32_MAX) throw std::overflow_error("rows coordinate overflow");
    return static_cast<Coordinate>(value);
}

// First interval of a sorted, disjoint row that ends strictly past `x`. Nothing before
// it can reach `x` or anything to its right.
Row::const_iterator first_ending_after(const Row& row, std::int64_t x) {
    return std::upper_bound(row.begin(), row.end(), x,
        [](std::int64_t value, const Interval& interval) { return value < std::int64_t(interval.end); });
}

// Empty width to the left of this island row's first segment, in fixed-point units.
std::int64_t row_clearance(const Row& occupied, const Row& segments, Coordinate offset) {
    if (segments.empty()) return 0;
    const std::int64_t left = std::int64_t(segments.front().begin) + offset;
    const auto it = first_ending_after(occupied, left);
    const std::int64_t previous_end = it == occupied.begin() ? 0 : std::int64_t((it - 1)->end);
    return left - previous_end;
}
} // namespace

void insert_piece(Row& row, Interval piece) {
    if (piece.end <= piece.begin) return;
    // Landing past everything already in the row is the common case - rasterization
    // walks spans left to right, and a commit usually lands right of the occupancy -
    // and it needs no rebuild at all.
    if (row.empty() || row.back().end <= piece.begin) {
        if (!row.empty() && row.back().end == piece.begin &&
            row.back().y_low == piece.y_low && row.back().y_high == piece.y_high)
            row.back().end = piece.end;
        else row.push_back(piece);
        return;
    }
    // Otherwise the row is rebuilt into a second buffer. Keeping that buffer per thread
    // and copying back (rather than swapping) leaves both sides at the capacity they
    // have grown to, so a run of inserts stops allocating altogether.
    thread_local Row out;
    out.clear();
    out.reserve(row.size() + 3);
    bool live = true; // `piece` still has an unplaced remainder.
    auto push = [&](Interval iv) {
        if (iv.end <= iv.begin) return;
        if (!out.empty() && out.back().end == iv.begin && out.back().y_low == iv.y_low && out.back().y_high == iv.y_high)
            out.back().end = iv.end;
        else out.push_back(iv);
    };
    for (const auto q : row) {
        if (!live || q.end <= piece.begin) { push(q); continue; }
        if (q.begin >= piece.end) { push(piece); live = false; push(q); continue; }
        // They overlap: emit whichever starts first up to the shared part, then the
        // shared part with the union of both bands, then carry on with the remainder.
        if (piece.begin < q.begin) push({piece.begin, q.begin, piece.y_low, piece.y_high});
        if (q.begin < piece.begin) push({q.begin, piece.begin, q.y_low, q.y_high});
        const Coordinate shared_begin = std::max(q.begin, piece.begin), shared_end = std::min(q.end, piece.end);
        push({shared_begin, shared_end, std::min(q.y_low, piece.y_low), std::max(q.y_high, piece.y_high)});
        if (q.end > piece.end) { push({piece.end, q.end, q.y_low, q.y_high}); live = false; }
        else if (piece.end > q.end) { piece = {q.end, piece.end, piece.y_low, piece.y_high}; }
        else live = false;
    }
    if (live) push(piece);
    row.assign(out.begin(), out.end());
}

void validate(const Shape& shape) {
    for (const auto& row : shape) {
        Coordinate previous_end = 0;
        for (const auto interval : row) {
            if (interval.begin < previous_end || interval.begin < 0 || interval.end <= interval.begin)
                throw std::invalid_argument("invalid rows interval order or extent");
            previous_end = interval.end;
        }
    }
}

Shape decode(const std::vector<Coordinate>& buffer, std::size_t row_count, std::size_t stride) {
    if (stride < row_count) throw std::invalid_argument("rows stride smaller than row count");
    auto at = [&](std::size_t plane, std::size_t row) {
        if (stride != 0 && plane > (std::numeric_limits<std::size_t>::max() - row) / stride)
            throw std::out_of_range("rows buffer index overflow");
        return buffer.at(plane * stride + row);
    };
    Shape result(row_count);
    for (std::size_t row = 0; row < row_count; ++row) {
        const auto count = at(0, row);
        if (count < 0 || count % 2 != 0) throw std::invalid_argument("invalid encoded interval count");
        if (count == 0) continue;
        std::int64_t cursor = at(1, row);
        for (std::size_t plane = 2; plane <= static_cast<std::size_t>(count); plane += 2) {
            const auto length = at(plane, row);
            if (length <= 0) throw std::invalid_argument("nonpositive encoded interval length");
            result[row].push_back({checked(cursor), checked(cursor + length)});
            cursor += length;
            if (plane < static_cast<std::size_t>(count)) {
                const auto gap = at(plane + 1, row);
                if (gap < 0) throw std::invalid_argument("negative encoded gap");
                cursor += gap;
            }
        }
    }
    validate(result);
    return result;
}

std::optional<Location> find_location(const Shape& canvas, const Shape& island,
                                      std::size_t row, Coordinate max_offset) {
    validate(canvas); validate(island);
    return find_location_unchecked(canvas, island, row, max_offset);
}

std::optional<Location> find_location_unchecked(const Shape& canvas, const Shape& island,
                                                std::size_t row, Coordinate max_offset) {
    if (max_offset < 0 || row > std::numeric_limits<std::size_t>::max() - island.size())
        throw std::invalid_argument("invalid rows search bounds");
    // Straight from the definition. At the current x, look for any island segment that
    // shares both an x-range and a height band with an occupied interval. Every such
    // pair proves that no offset below `occupied.end - segment.begin` can work, because
    // the pair keeps overlapping all the way up to it; so the largest of those bounds
    // is where the search moves next. x only grows, and each step is at least one unit.
    const std::size_t overlap = row < canvas.size() ? std::min(island.size(), canvas.size() - row) : 0;
    Coordinate offset = 0;
    while (true) {
        std::int64_t required = 0;
        for (std::size_t i = 0; i < overlap; ++i) {
            const Row& occupied = canvas[row + i];
            if (occupied.empty()) continue;
            for (const auto segment : island[i]) {
                const std::int64_t left = std::int64_t(segment.begin) + offset;
                const std::int64_t right = std::int64_t(segment.end) + offset;
                for (auto it = first_ending_after(occupied, left);
                     it != occupied.end() && std::int64_t(it->begin) < right; ++it)
                    if (y_overlap(*it, segment))
                        required = std::max(required, std::int64_t(it->end) - left);
            }
        }
        if (required == 0) break;
        const std::int64_t next = std::int64_t(offset) + required;
        if (next > max_offset) return std::nullopt;
        offset = static_cast<Coordinate>(next);
    }
    // The cost's clearance term. Summed exactly in fixed point and converted once, so
    // the indexed search can reproduce it bit for bit however it visits the rows.
    std::int64_t clearance = 0;
    const Row empty;
    for (std::size_t i = 0; i < island.size(); ++i)
        clearance += row_clearance(row + i < canvas.size() ? canvas[row + i] : empty, island[i], offset);
    return Location{offset, static_cast<float>(clearance) * (1.0f / units)};
}

void commit(Shape& canvas, const Shape& island, std::size_t row, Coordinate offset_x) {
    validate(canvas); validate(island);
    commit_unchecked(canvas, island, row, offset_x);
}

void commit_unchecked(Shape& canvas, const Shape& island, std::size_t row, Coordinate offset_x) {
    if (offset_x < 0 || row > canvas.max_size() || island.size() > canvas.max_size() - row)
        throw std::invalid_argument("invalid rows commit bounds");
    // Check the whole translation before any of it is applied, so a row that overflows
    // cannot leave the canvas half-updated.
    for (const auto& source : island)
        for (const auto interval : source) checked(std::int64_t(interval.end) + offset_x);
    canvas.resize(std::max(canvas.size(), row + island.size()));
    for (std::size_t i = 0; i < island.size(); ++i)
        for (const auto interval : island[i])
            insert_piece(canvas[row + i], {checked(std::int64_t(interval.begin) + offset_x),
                                           checked(std::int64_t(interval.end) + offset_x),
                                           interval.y_low, interval.y_high});
}

} // namespace uvpack::rows
