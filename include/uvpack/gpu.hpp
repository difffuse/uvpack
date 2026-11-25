#pragma once
// An optional Vulkan compute worker that runs WHOLE greedy packing trials on
// the GPU, concurrently with the CPU worker threads.
//
// The contract with the rest of the packer is deliberately narrow: a worker is handed
// the same `RasterIsland` set and `FixedTarget` a CPU trial would get, plus a batch of
// trial plans, and gives back one `Evaluation`-shaped result per plan. Every result is
// re-validated on the CPU before it is used, and results only ever ENTER the portfolio
// as extra candidates, so a GPU that answers badly (or not at all) can never make a
// run worse than the CPU-only run.
//
// The whole surface degrades to "no device": every entry point is safe to call when
// the build has no Vulkan at all (UVPACK_VULKAN=OFF) or when the loader, the ICD or
// the device are missing at runtime. See `diagnostic()` for why.
//
// What the kernel does NOT do (the caller must fall back to the CPU; `supported()`
// checks these): sub-row sub_rows, static obstacles, several tiles, adjacent groups,
// event rows, partial packing, and texel snapping. See README.md.
#include "uvpack/stripe_placer.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace uvpack::gpu {

// True when this build has the Vulkan code compiled in at all.
bool compiled_in();

struct DeviceInfo {
    std::string name;
    std::string type;                 // discrete | integrated | virtual | cpu | other
    std::uint32_t vendor_id = 0, device_id = 0;
    std::uint64_t device_local_bytes = 0;
    std::uint32_t api_major = 0, api_minor = 0, api_patch = 0;
    std::uint32_t max_workgroup_invocations = 0;
    std::uint32_t max_shared_memory_bytes = 0;
    std::uint64_t max_storage_buffer_bytes = 0;
    bool portability = false;         // Reached through VK_KHR_portability_subset (MoltenVK).
};

// Enumerate every compute-capable device. Empty when there is none (or no loader).
const std::vector<DeviceInfo>& devices();
// True when at least one device is usable.
bool available();
// Human-readable reason the loader/instance/device enumeration ended where it did.
const std::string& diagnostic();

// The kernel's x resolution. The canvas is a bitmask with `1 << bit_shift` bits
// across the frame width and 8 vertical bands per stripe row; every interval is
// rounded OUTWARD onto that grid, so the occupancy over-approximates the true one and
// a clear placement on the GPU is always clear in the continuous domain too.
// 2^-14 of the frame. Measured on mixed100: at 2^-12 the outward rounding costs the
// kernel about 1.3 percent of scale against the CPU greedy on the same order, which
// makes a device trial useless as a portfolio candidate; at 2^-14 the two reach exactly
// the same scales. It costs 4x the canvas memory and about 6x the dispatch time.
inline constexpr int default_bit_shift = 14;
inline constexpr int bands = 8;                // Vertical bands per stripe row.
inline constexpr int band_planes = bands + 1;  // Plus a ninth plane holding their union.
inline constexpr std::size_t max_orientations = 32; // Skip masks are one uint per island.

struct Limits {
    int bit_shift = default_bit_shift;
    // Total canvas scratch the worker may allocate; trials are capped to fit.
    std::size_t max_canvas_bytes = std::size_t(512) << 20;
    std::size_t max_trials = 256;
};

// Why a problem cannot go to the GPU; empty means it can. Sub-row placement is not a
// reason: the kernel places on row boundaries, which is a coarser trial, not a wrong one.
std::string unsupported_reason(const std::vector<RasterIsland>& rasters, const FixedTarget& target,
                               std::size_t tile_count, bool obstacles, bool group_adjacent,
                               bool allow_partial);

// One trial's diversification, the device-side shape of a TrialPlan. Sub-row 0 only.
struct TrialSpec {
    std::vector<std::uint32_t> island_order;  // Permutation of raster indices.
    std::vector<std::uint32_t> skip_mask;     // Per raster index: bit o set = orientation o masked out.
    // Pack into a frame this much smaller than the uploaded one. Scaling the
    // resulting layout back up to the target multiplies the scale by `frame_shrink` and
    // grows every gap by the same factor, so one upload of one scale's rasters covers a
    // whole ladder of effective scales in a single dispatch. 1 = the frame as uploaded.
    double frame_shrink = 1.0;
};

struct TrialResult {
    bool valid = false;
    // In the same shape the CPU greedy produces, so a GPU placement is just a Placement.
    // Offsets are in the SHRUNKEN frame: the caller multiplies them, and the scale, by
    // `frame_shrink` to bring the layout back to the target.
    std::vector<Placement> placements;
    double occupied_width = 0, occupied_height = 0;
    double frame_shrink = 1.0;
};

struct WorkerStats {
    std::size_t dispatches = 0;
    std::size_t trials = 0;        // Trials handed to the device.
    std::size_t valid_trials = 0;  // Trials the device reported complete.
    std::size_t accepted = 0;      // Trials the CPU validator then accepted.
    std::size_t uploads = 0;
    std::size_t bytes_uploaded = 0;
    double upload_seconds = 0, compute_seconds = 0, readback_seconds = 0;
};

// Runs whole greedy trials for ONE tile at ONE scale. Not thread-safe: one worker per
// packing thread (or one worker driven by the thread that owns the round).
class Worker {
public:
    ~Worker();
    Worker(const Worker&) = delete;
    Worker& operator=(const Worker&) = delete;
    // nullptr when there is no usable device or the pipeline could not be built.
    // `device_index` < 0 picks the best device (discrete > integrated > virtual > cpu).
    static std::unique_ptr<Worker> create(int device_index = -1, Limits limits = {});

    const DeviceInfo& device() const;
    const std::string& error() const;

    // Upload one scale's rasters. False leaves the worker usable but idle (the caller
    // simply runs CPU-only). Must be called before run(); safe to call repeatedly.
    bool upload(const std::vector<RasterIsland>& rasters, const FixedTarget& target);
    // Largest trial batch this worker can run for the uploaded problem.
    std::size_t trial_capacity() const;
    // Run `plans` independent greedy trials in one dispatch. `out` is resized to
    // plans.size(); an entry with valid=false simply did not place everything.
    bool run(const std::vector<TrialSpec>& plans, std::vector<TrialResult>& out);

    const WorkerStats& stats() const;
    void note_accepted(std::size_t count);

private:
    Worker();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Liveness probe: dispatches a trivial kernel and checks its output.
// Returns an empty string on success, otherwise the failure.
std::string self_test(int device_index = -1);

// Exclusive access to the process-wide worker, built on first use. There is one
// device, so there is one worker: a lease serialises the packer's calls onto it
// without every call paying for device and pipeline creation. Empty when there is no
// device, when creation failed, or when `disable()` has been called.
//
// The packer arranges for at most one lease to be wanted at a time, so this lock is a
// safety net rather than a bottleneck; a caller packing from several threads of its
// own simply queues.
class WorkerLease {
public:
    WorkerLease();
    ~WorkerLease();
    WorkerLease(WorkerLease&&) noexcept;
    WorkerLease(const WorkerLease&) = delete;
    WorkerLease& operator=(const WorkerLease&) = delete;
    Worker* operator->() const { return worker_; }
    Worker* get() const { return worker_; }
    explicit operator bool() const { return worker_ != nullptr; }

private:
    Worker* worker_ = nullptr;
    bool held_ = false;
};

// Why the shared worker is empty; empty string when it is fine.
std::string lease_diagnostic();
// Tear the shared worker down (tests, and the "gpu": false measurement path).
void release_shared_worker();

} // namespace uvpack::gpu
