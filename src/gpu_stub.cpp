// Built when UVPACK_VULKAN is OFF (or the Vulkan headers are absent): every entry
// point answers "no device". Nothing else in the packer changes behaviour, so a build
// without Vulkan is exactly the CPU-only packer.
#include "uvpack/gpu.hpp"

namespace uvpack::gpu {

bool compiled_in() { return false; }

const std::vector<DeviceInfo>& devices() {
    static const std::vector<DeviceInfo> none;
    return none;
}

bool available() { return false; }

const std::string& diagnostic() {
    static const std::string message = "built without Vulkan (UVPACK_VULKAN=OFF)";
    return message;
}

std::unique_ptr<Worker> Worker::create(int, Limits) { return nullptr; }

std::string self_test(int) { return diagnostic(); }

// Never reached: create() always fails here. They exist so callers holding a
// std::unique_ptr<Worker> still link.
struct Worker::Impl {};
Worker::Worker() = default;
Worker::~Worker() = default;
const DeviceInfo& Worker::device() const { static const DeviceInfo none; return none; }
const std::string& Worker::error() const { return diagnostic(); }
bool Worker::upload(const std::vector<RasterIsland>&, const FixedTarget&) { return false; }
std::size_t Worker::trial_capacity() const { return 0; }
bool Worker::run(const std::vector<TrialSpec>&, std::vector<TrialResult>& out) { out.clear(); return false; }
const WorkerStats& Worker::stats() const { static const WorkerStats none; return none; }
void Worker::note_accepted(std::size_t) {}

} // namespace uvpack::gpu
