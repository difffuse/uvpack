#pragma once
// Several rectangular targets (tiles) packed at one uniform scale. Each tile
// owns a StripePlacer; candidate rows and locations carry the tile index
// so the generic greedy loop stays unchanged. Islands may be restricted to a set
// of tiles through `group_tiles`; a per-tile penalty implements one-by-one filling.
#include "uvpack/stripe_placer.hpp"
#include <memory>

namespace uvpack {
class TilePlacer final : public RowPlacer {
public:
    struct Tile { FixedTarget target; };
    static constexpr std::int64_t row_stride = std::int64_t(1) << 40; // Tile index encoded into candidate rows.

    // group_adjacent: `group_weight` > 0 turns on the compactness cost inside
    // every tile and confines each group to the tile its first island landed in.
    TilePlacer(std::vector<Tile> tiles, const std::vector<RasterIsland>& rasters,
                       std::vector<std::vector<bool>> allowed_tiles, double tile_penalty,
                       bool group_adjacent = false, double group_weight = 0);
    PreparedProblem prepare(const Input& input) override;
    void add_fixed_occupancy(std::size_t tile, rows::Shape rows, std::size_t first_row);
    std::size_t tile_count() const { return tiles_.size(); }
    const StripePlacer& tile(std::size_t t) const { return *tiles_.at(t); }
    // Summed group box area over every tile after a trial (group_adjacent).
    double group_box_area() const;

protected:
    void begin_trial(const PreparedProblem&, const Parameters&) override;
    const std::vector<std::int64_t>& candidate_rows(std::size_t island, std::size_t orientation) override;
    std::optional<ClearLocation> find_first_clear(std::size_t island, std::size_t orientation, std::int64_t row, double cost_ceiling) override;
    ClearLocation refine(std::size_t island, std::size_t orientation, const ClearLocation&) override;
    void commit(std::size_t island, std::size_t orientation, const ClearLocation&) override;
    Evaluation finish_trial(const std::vector<Placement>&, bool all_placed) override;
    double score_location(const ClearLocation&) const override;
    double row_cost_lower_bound(std::size_t island, std::size_t orientation, std::int64_t row) const override;

private:
    std::vector<std::unique_ptr<StripePlacer>> tiles_;
    std::vector<std::vector<bool>> allowed_;
    double penalty_;
    std::vector<std::int64_t> merged_;
    // group_adjacent: per prepared island index the group id, and per trial the tile a
    // group has been pinned to (-1 = still free). One box per group.
    bool group_adjacent_ = false;
    std::vector<std::size_t> group_of_;
    std::vector<std::size_t> group_ids_;
    std::vector<int> group_tile_;
    std::size_t group_slot(std::size_t island) const;
};
} // namespace uvpack
