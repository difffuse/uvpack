// The parts of the GPU worker that exist in every build, with or without
// Vulkan. Chiefly the admission rule: exactly which problems the kernel covers.
#include "uvpack/gpu.hpp"
#include <cmath>
#include <mutex>

namespace uvpack::gpu {
namespace {
std::mutex g_lease_mutex;
std::unique_ptr<Worker> g_worker;
bool g_tried = false;
std::string g_lease_diagnostic;
} // namespace

WorkerLease::WorkerLease() {
    g_lease_mutex.lock();
    held_ = true;
    if (!g_tried) {
        g_tried = true;
        g_worker = Worker::create();
        if (!g_worker) g_lease_diagnostic = diagnostic();
    }
    worker_ = g_worker.get();
    if (!worker_) {
        g_lease_mutex.unlock();
        held_ = false;
    }
}

WorkerLease::WorkerLease(WorkerLease&& other) noexcept : worker_(other.worker_), held_(other.held_) {
    other.worker_ = nullptr;
    other.held_ = false;
}

WorkerLease::~WorkerLease() {
    if (held_) g_lease_mutex.unlock();
}

std::string lease_diagnostic() {
    std::lock_guard<std::mutex> guard(g_lease_mutex);
    return g_lease_diagnostic;
}

void release_shared_worker() {
    std::lock_guard<std::mutex> guard(g_lease_mutex);
    g_worker.reset();
    g_tried = false;
    g_lease_diagnostic.clear();
}

std::string unsupported_reason(const std::vector<RasterIsland>& rasters, const FixedTarget& target,
                               std::size_t tile_count, bool obstacles, bool group_adjacent,
                               bool allow_partial) {
    // Each of these is a real gap in the kernel, not a policy choice; the caller runs
    // the CPU path alone when one of them applies. README "GPU worker" lists them.
    //
    // Sub-row placement is deliberately NOT here. The kernel places on row boundaries
    // (sub-row 0) while the CPU refines into the stripe below, so a GPU trial is simply a
    // slightly coarser trial of the same problem: a worse candidate, never an invalid
    // one, and the portfolio keeps the best of the two.
    if (tile_count != 1) return "several tiles";
    if (obstacles) return "static obstacles";
    if (group_adjacent) return "groups packed together";
    if (allow_partial) return "partial packing";
    if (rasters.empty()) return "no islands";
    if (!(target.width > 0) || !(target.height > 0) || !(target.stripe_height > 0)) return "degenerate target";
    if (target.height / target.stripe_height > 65535.0) return "too many stripe rows";
    for (const auto& island : rasters) {
        if (island.orientations.size() > max_orientations) return "more than 32 orientations per island";
        for (const auto& orientation : island.orientations)
            if (orientation.rows.size() > 65535) return "island taller than 65535 stripe rows";
    }
    return {};
}

} // namespace uvpack::gpu
