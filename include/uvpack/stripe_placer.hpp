#pragma once
#include "uvpack/fast_occupancy.hpp"
#include "uvpack/geometry.hpp"
#include "uvpack/placement.hpp"
#include "uvpack/rows.hpp"
#include <array>
#include <memory>

namespace uvpack {
// One orientation of one island, ready to place: its size, its occupancy rows, and
// (when sub-row placement is on) the rotated geometry behind those rows so the placer
// can re-rasterize them at a fractional stripe offset on demand.
struct RasterOrientation {
    Orientation geometry;
    rows::Shape rows;
    std::shared_ptr<const std::vector<Polygon>> source; // Null disables sub-row placement.
    std::size_t sub_row_count = 1;   // Sub-row resolution; sub_row k sits k/sub_row_count up a stripe.
    std::array<double, 4> frame{}; // min_x, min_y, max_x, max_y of those parts.
};
struct RasterIsland {
    std::uint64_t id;
    double hull_area; // Of the ORIGINAL island, before the margin expansion.
    std::vector<RasterOrientation> orientations;
};

// One frame to pack into, at one scale.
struct FixedTarget {
    double width, height, stripe_height;
    std::size_t max_candidate_rows = 65536; // Resource guard, not an algorithmic limit.
    // The occupied canvas can start out already the size of the frame, so the growth
    // term only charges for spilling past it; 0 = start from nothing, which is what a
    // bounded pack wants.
    double initial_occupied_width = 0, initial_occupied_height = 0;
    // Indexed occupancy plus the exact per-row cost bound (see fast_occupancy.hpp).
    // False falls back to the plain interval search, which answers identically, slower.
    bool fast_search = true;
};

// Fixed-scale, single-target placement over caller-prepared orientation rows.
// Takes no Parameters: anything nonempty is rejected rather than ignored.
class StripePlacer final : public RowPlacer {
    friend class TilePlacer;
public:
    std::size_t raster_index(std::size_t island) const { return raster_indices_.at(island); }
    StripePlacer(FixedTarget target, std::vector<RasterIsland> rasters);
    PreparedProblem prepare(const Input& input) override;
    const rows::Shape& occupancy() const { return occupancy_.shape(); }
    double occupied_width() const { return occupied_width_; }
    double occupied_height() const { return occupied_height_; }
    // Occupancy present before every trial (static obstacles), already clipped to the
    // frame and expressed in frame coordinates (row index, x in fixed-point units).
    void add_fixed_occupancy(rows::Shape rows, std::size_t first_row);
    // Weight of the group-cohesion term added to the location cost. 0 (the default)
    // disables every group code path bit for bit.
    void set_group_weight(double weight) { group_weight_ = weight; }
    // Bounding box (min_x, min_y, max_x, max_y) of the group's members already placed
    // in this tile during the current trial; empty when the group has none yet.
    std::optional<std::array<double, 4>> group_box(std::size_t group) const;
    // Summed area of those boxes after a trial: a comparator between attempts at the
    // same scale, since they place the same islands. Lower is more compact.
    double group_box_area() const;
protected:
    void begin_trial(const PreparedProblem&, const Parameters&) override;
    const std::vector<std::int64_t>& candidate_rows(std::size_t island, std::size_t orientation) override;
    std::optional<ClearLocation> find_first_clear(std::size_t island, std::size_t orientation,
                                                std::int64_t row, double cost_ceiling) override;
    ClearLocation refine(std::size_t island, std::size_t orientation, const ClearLocation&) override;
    void commit(std::size_t island, std::size_t orientation, const ClearLocation&) override;
    Evaluation finish_trial(const std::vector<Placement>&, bool all_placed) override;
    double score_location(const ClearLocation&) const override;
    double row_cost_lower_bound(std::size_t island, std::size_t orientation, std::int64_t row) const override;
private:
    FixedTarget target_;
    std::vector<RasterIsland> rasters_;
    std::vector<std::size_t> raster_indices_;
    PreparedProblem prepared_;
    bool ready_ = false;
    FastOccupancy occupancy_;
    double occupied_width_ = 0, occupied_height_ = 0;
    rows::Coordinate frame_units_ = 0;
    std::vector<std::int64_t> row_scratch_;
    std::vector<rows::Coordinate> search_scratch_;
    struct FixedOccupancy { rows::Shape rows; std::size_t first_row; };
    std::vector<FixedOccupancy> fixed_;
    void commit_rows(const rows::Shape& rows, std::size_t first_row, rows::Coordinate offset_units, double x, double y, double width, double height);
    // Per raster orientation: the longest interval of each row, the same rows ordered by
    // decreasing longest interval (so the free-gap rejection hits its worst row first),
    // the flattened rows the search walks, and the row sums the clearance bound needs.
    struct OrientationIndex {
        std::vector<rows::Coordinate> max_segment;
        std::vector<std::uint32_t> gap_order;
        FastIsland flat;                       // The orientation's rows in one array.
        std::vector<std::uint32_t> empty_rows; // Rows with no intervals: no clearance of their own.
        std::int64_t sum_begin = 0;            // Sum of the nonempty rows' first begins.
        std::size_t nonempty = 0;
    };
    std::vector<std::vector<OrientationIndex>> orientation_index_;
    const OrientationIndex& index_for(std::size_t island, std::size_t orientation) const {
        return orientation_index_[raster_indices_[island]][orientation];
    }
    const RasterOrientation& raster(std::size_t island, std::size_t orientation) const;
    const rows::Shape& rows_for(std::size_t island, std::size_t orientation, std::size_t sub_row);
    // Lazily computed sub-row rasters, [raster][orientation][sub_row-1]; empty = not yet built.
    std::vector<std::vector<std::vector<rows::Shape>>> sub_row_cache_;
    // Per-trial bounding box of each group's placed members.
    // Groups are remapped to dense slots in prepare(), keyed by the sorted group ids.
    double group_weight_ = 0;
    std::vector<std::size_t> group_ids_;         // Sorted unique group ids -> slot.
    std::vector<std::size_t> group_slot_;        // Per prepared island index.
    std::vector<std::array<double, 4>> group_box_;
    std::vector<char> group_used_;
    std::size_t slot_of(std::size_t group) const;
    // Pull towards the group's box: centre distance plus the box's half-perimeter growth.
    double group_cost(std::size_t island, const ClearLocation& p) const;
};
} // namespace uvpack
