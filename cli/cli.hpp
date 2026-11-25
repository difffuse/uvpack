#pragma once
// The CLI's own pieces: a parsed request (cli/request.cpp) and the JSON that goes back
// out (cli/response.cpp). cli/main.cpp wires them to the engine.
#include "uvpack/minijson.hpp"
#include "uvpack/packer.hpp"
#include "uvpack/texel_density.hpp"
#include "uvpack/version.hpp"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace uvpack::cli {

// The vocabulary this executable speaks, reported in `--version` and in every result
// and streamed event, so a client can tell what it is talking to before it talks.
inline constexpr const char* dialect = metadata::dialect;

// One request, ready to pack: the islands as the engine will see them, the options,
// and the bookkeeping the response needs to describe what was done to them.
struct Request {
    PackTarget target;
    bool require_complete = false;
    std::vector<std::uint64_t> requested_ids;
    std::vector<Island> islands;
    std::vector<Obstacle> obstacles;
    PackOptions options;
    std::string scale_mode = "max";
    double fixed_scale = 1;
    bool stream = false;      // One JSON line per improved layout, then a "done" line.
    bool self_check = true;   // Include diagnostics; final certification is mandatory.
    double requested_margin = -1; // The margin asked for, when snapping reserved more.
    double requested_border = -1; // Explicit border before any snapping reserve; negative follows the gap.
    // bind_overlapping / bind_group: leader island id -> every original id it stands for.
    std::map<std::uint64_t, std::vector<std::uint64_t>> members;
    // Original footprints for final certification and per-vertex output. Explicit
    // envelope-only recovery retains its approved hull as the declared footprint.
    std::map<std::uint64_t, Island> source_islands;
    json::Object input_report;
    // Texel density: what the density controls were handed and what they did.
    std::vector<DensityIsland> density;
    std::string density_mode = "per_island";
    double density_factor = 1;
    double tex_size = 0;
    double texel_density_request = 0;
    std::size_t texel_density_islands = 0;
};

// JSON in, a packable request out. Throws std::runtime_error on anything malformed.
Request parse_request(const json::Value& doc);
json::Object validate_request(const json::Value& doc,const CancellationToken& cancel={});
json::Object normalize_request(const json::Value& doc,const CancellationToken& cancel={});

// One JSON entry per island of a layout: the transform and the pose it came from, with
// one entry per original id for bound members. Shared by the streamed "improved" lines
// and the final result, so a client can apply either with the same code.
json::Array islands_json(const Request& request, const Layout& layout);

// One streamed "improved" line for a layout the search just committed.
json::Object stream_event(const Request& request, const Layout& layout, double elapsed);

// The full result object. Applies the texel snapping (which can move the layout and
// the achieved margin) and the self check on the way.
json::Object result_json(const Request& request, Layout& layout, const SearchStats& stats,
                         double achieved_margin);

} // namespace uvpack::cli
