// The Vulkan compute runtime behind uvpack/gpu.hpp.
//
// The loader is opened with dlopen/LoadLibrary and every entry point is fetched
// through vkGetInstanceProcAddr, so this translation unit links against nothing: a
// binary built with UVPACK_VULKAN=ON still starts and packs on a machine with no
// Vulkan installed at all. Every failure path ends in "no device", never a crash.
//
// Portability (MoltenVK on macOS) is handled where the spec requires it: the
// VK_KHR_portability_enumeration instance extension plus its create flag, and the
// VK_KHR_portability_subset device extension when a device advertises it.
#include "uvpack/gpu.hpp"
#include "uvpack/gpu_reference.hpp"
#include "uvpack/gpu_shaders.hpp"

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace uvpack::gpu {
namespace {

using Clock = std::chrono::steady_clock;

#ifndef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
#  define VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME "VK_KHR_portability_enumeration"
#endif
#ifndef VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR
#  define VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR 0x00000001
#endif

// ---------------------------------------------------------------------------
// Loader
// ---------------------------------------------------------------------------

struct Library {
    void* handle = nullptr;
    ~Library() {
        // Left open deliberately: the instance outlives individual workers and some
        // loaders do not survive being unloaded while an ICD still has state.
    }
    void* symbol(const char* name) const {
#if defined(_WIN32)
        return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle), name));
#else
        return dlsym(handle, name);
#endif
    }
};

void* open_library(const char* name) {
#if defined(_WIN32)
    return reinterpret_cast<void*>(LoadLibraryA(name));
#else
    return dlopen(name, RTLD_NOW | RTLD_LOCAL);
#endif
}

struct InstanceApi {
    PFN_vkGetInstanceProcAddr GetInstanceProcAddr = nullptr;
    PFN_vkCreateInstance CreateInstance = nullptr;
    PFN_vkDestroyInstance DestroyInstance = nullptr;
    PFN_vkEnumerateInstanceExtensionProperties EnumerateInstanceExtensionProperties = nullptr;
    PFN_vkEnumeratePhysicalDevices EnumeratePhysicalDevices = nullptr;
    PFN_vkGetPhysicalDeviceProperties GetPhysicalDeviceProperties = nullptr;
    PFN_vkGetPhysicalDeviceMemoryProperties GetPhysicalDeviceMemoryProperties = nullptr;
    PFN_vkGetPhysicalDeviceQueueFamilyProperties GetPhysicalDeviceQueueFamilyProperties = nullptr;
    PFN_vkEnumerateDeviceExtensionProperties EnumerateDeviceExtensionProperties = nullptr;
    PFN_vkCreateDevice CreateDevice = nullptr;
    PFN_vkGetDeviceProcAddr GetDeviceProcAddr = nullptr;
};

struct DeviceApi {
    PFN_vkDestroyDevice DestroyDevice = nullptr;
    PFN_vkGetDeviceQueue GetDeviceQueue = nullptr;
    PFN_vkDeviceWaitIdle DeviceWaitIdle = nullptr;
    PFN_vkCreateBuffer CreateBuffer = nullptr;
    PFN_vkDestroyBuffer DestroyBuffer = nullptr;
    PFN_vkGetBufferMemoryRequirements GetBufferMemoryRequirements = nullptr;
    PFN_vkAllocateMemory AllocateMemory = nullptr;
    PFN_vkFreeMemory FreeMemory = nullptr;
    PFN_vkBindBufferMemory BindBufferMemory = nullptr;
    PFN_vkMapMemory MapMemory = nullptr;
    PFN_vkUnmapMemory UnmapMemory = nullptr;
    PFN_vkFlushMappedMemoryRanges FlushMappedMemoryRanges = nullptr;
    PFN_vkInvalidateMappedMemoryRanges InvalidateMappedMemoryRanges = nullptr;
    PFN_vkCreateShaderModule CreateShaderModule = nullptr;
    PFN_vkDestroyShaderModule DestroyShaderModule = nullptr;
    PFN_vkCreateDescriptorSetLayout CreateDescriptorSetLayout = nullptr;
    PFN_vkDestroyDescriptorSetLayout DestroyDescriptorSetLayout = nullptr;
    PFN_vkCreatePipelineLayout CreatePipelineLayout = nullptr;
    PFN_vkDestroyPipelineLayout DestroyPipelineLayout = nullptr;
    PFN_vkCreateComputePipelines CreateComputePipelines = nullptr;
    PFN_vkDestroyPipeline DestroyPipeline = nullptr;
    PFN_vkCreateDescriptorPool CreateDescriptorPool = nullptr;
    PFN_vkDestroyDescriptorPool DestroyDescriptorPool = nullptr;
    PFN_vkAllocateDescriptorSets AllocateDescriptorSets = nullptr;
    PFN_vkUpdateDescriptorSets UpdateDescriptorSets = nullptr;
    PFN_vkCreateCommandPool CreateCommandPool = nullptr;
    PFN_vkDestroyCommandPool DestroyCommandPool = nullptr;
    PFN_vkAllocateCommandBuffers AllocateCommandBuffers = nullptr;
    PFN_vkBeginCommandBuffer BeginCommandBuffer = nullptr;
    PFN_vkEndCommandBuffer EndCommandBuffer = nullptr;
    PFN_vkResetCommandBuffer ResetCommandBuffer = nullptr;
    PFN_vkCmdBindPipeline CmdBindPipeline = nullptr;
    PFN_vkCmdBindDescriptorSets CmdBindDescriptorSets = nullptr;
    PFN_vkCmdPushConstants CmdPushConstants = nullptr;
    PFN_vkCmdDispatch CmdDispatch = nullptr;
    PFN_vkCmdPipelineBarrier CmdPipelineBarrier = nullptr;
    PFN_vkQueueSubmit QueueSubmit = nullptr;
    PFN_vkCreateFence CreateFence = nullptr;
    PFN_vkDestroyFence DestroyFence = nullptr;
    PFN_vkWaitForFences WaitForFences = nullptr;
    PFN_vkResetFences ResetFences = nullptr;
};

// One process-wide instance plus the device list; built once, lazily.
struct Runtime {
    Library library;
    InstanceApi api;
    VkInstance instance = VK_NULL_HANDLE;
    std::vector<VkPhysicalDevice> handles;
    std::vector<DeviceInfo> infos;
    std::string diagnostic;
    bool ready = false;
};

std::string type_name(VkPhysicalDeviceType type) {
    switch (type) {
    case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return "integrated";
    case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return "discrete";
    case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return "virtual";
    case VK_PHYSICAL_DEVICE_TYPE_CPU: return "cpu";
    default: return "other";
    }
}

int type_rank(const std::string& type) {
    if (type == "discrete") return 4;
    if (type == "integrated") return 3;
    if (type == "virtual") return 2;
    if (type == "cpu") return 1;
    return 0;
}

void load_instance_api(Runtime& rt) {
    auto get = [&](const char* name) { return rt.api.GetInstanceProcAddr(rt.instance, name); };
#define UVPACK_LOAD(field) rt.api.field = reinterpret_cast<PFN_vk##field>(get("vk" #field))
    UVPACK_LOAD(DestroyInstance);
    UVPACK_LOAD(EnumeratePhysicalDevices);
    UVPACK_LOAD(GetPhysicalDeviceProperties);
    UVPACK_LOAD(GetPhysicalDeviceMemoryProperties);
    UVPACK_LOAD(GetPhysicalDeviceQueueFamilyProperties);
    UVPACK_LOAD(EnumerateDeviceExtensionProperties);
    UVPACK_LOAD(CreateDevice);
    UVPACK_LOAD(GetDeviceProcAddr);
#undef UVPACK_LOAD
}

bool device_has_extension(Runtime& rt, VkPhysicalDevice device, const char* name) {
    std::uint32_t count = 0;
    if (!rt.api.EnumerateDeviceExtensionProperties) return false;
    rt.api.EnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    if (count) rt.api.EnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());
    for (const auto& extension : extensions)
        if (std::strcmp(extension.extensionName, name) == 0) return true;
    return false;
}

void build_runtime(Runtime& rt) {
    static const char* candidates[] = {
#if defined(_WIN32)
        "vulkan-1.dll",
#elif defined(__APPLE__)
        "libvulkan.1.dylib", "libvulkan.dylib",
        "/opt/homebrew/lib/libvulkan.1.dylib", "/usr/local/lib/libvulkan.1.dylib",
        "libMoltenVK.dylib", "/opt/homebrew/lib/libMoltenVK.dylib",
#else
        "libvulkan.so.1", "libvulkan.so",
#endif
    };
    for (const char* name : candidates) {
        rt.library.handle = open_library(name);
        if (rt.library.handle) break;
    }
    if (!rt.library.handle) {
        rt.diagnostic = "no Vulkan loader found (tried libvulkan / MoltenVK)";
        return;
    }
    rt.api.GetInstanceProcAddr =
        reinterpret_cast<PFN_vkGetInstanceProcAddr>(rt.library.symbol("vkGetInstanceProcAddr"));
    if (!rt.api.GetInstanceProcAddr) {
        rt.diagnostic = "loader has no vkGetInstanceProcAddr";
        return;
    }
    rt.api.CreateInstance = reinterpret_cast<PFN_vkCreateInstance>(
        rt.api.GetInstanceProcAddr(VK_NULL_HANDLE, "vkCreateInstance"));
    rt.api.EnumerateInstanceExtensionProperties =
        reinterpret_cast<PFN_vkEnumerateInstanceExtensionProperties>(
            rt.api.GetInstanceProcAddr(VK_NULL_HANDLE, "vkEnumerateInstanceExtensionProperties"));
    if (!rt.api.CreateInstance) {
        rt.diagnostic = "loader has no vkCreateInstance";
        return;
    }

    std::vector<const char*> extensions;
    VkInstanceCreateFlags flags = 0;
    if (rt.api.EnumerateInstanceExtensionProperties) {
        std::uint32_t count = 0;
        rt.api.EnumerateInstanceExtensionProperties(nullptr, &count, nullptr);
        std::vector<VkExtensionProperties> available(count);
        if (count) rt.api.EnumerateInstanceExtensionProperties(nullptr, &count, available.data());
        for (const auto& extension : available) {
            if (std::strcmp(extension.extensionName, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) == 0) {
                extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
                flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
            }
        }
    }

    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "uvpack";
    app.applicationVersion = 1;
    app.pEngineName = "uvpack";
    app.engineVersion = 1;
    app.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.flags = flags;
    info.pApplicationInfo = &app;
    info.enabledExtensionCount = std::uint32_t(extensions.size());
    info.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

    VkResult result = rt.api.CreateInstance(&info, nullptr, &rt.instance);
    if (result != VK_SUCCESS && flags != 0) {
        // Some loaders report the extension but refuse the flag; retry plain.
        info.flags = 0;
        info.enabledExtensionCount = 0;
        info.ppEnabledExtensionNames = nullptr;
        result = rt.api.CreateInstance(&info, nullptr, &rt.instance);
    }
    if (result != VK_SUCCESS) {
        rt.diagnostic = "vkCreateInstance failed (" + std::to_string(int(result)) + ")";
        rt.instance = VK_NULL_HANDLE;
        return;
    }
    load_instance_api(rt);
    if (!rt.api.EnumeratePhysicalDevices) {
        rt.diagnostic = "instance has no vkEnumeratePhysicalDevices";
        return;
    }
    std::uint32_t count = 0;
    rt.api.EnumeratePhysicalDevices(rt.instance, &count, nullptr);
    if (count == 0) {
        rt.diagnostic = "instance created but no physical device was reported"
                        " (on macOS set VK_ICD_FILENAMES to the MoltenVK_icd.json path)";
        return;
    }
    rt.handles.resize(count);
    rt.api.EnumeratePhysicalDevices(rt.instance, &count, rt.handles.data());
    for (auto handle : rt.handles) {
        VkPhysicalDeviceProperties properties{};
        rt.api.GetPhysicalDeviceProperties(handle, &properties);
        DeviceInfo info_out;
        info_out.name = properties.deviceName;
        info_out.type = type_name(properties.deviceType);
        info_out.vendor_id = properties.vendorID;
        info_out.device_id = properties.deviceID;
        info_out.api_major = VK_VERSION_MAJOR(properties.apiVersion);
        info_out.api_minor = VK_VERSION_MINOR(properties.apiVersion);
        info_out.api_patch = VK_VERSION_PATCH(properties.apiVersion);
        info_out.max_workgroup_invocations = properties.limits.maxComputeWorkGroupInvocations;
        info_out.max_shared_memory_bytes = properties.limits.maxComputeSharedMemorySize;
        info_out.max_storage_buffer_bytes = properties.limits.maxStorageBufferRange;
        info_out.portability = device_has_extension(rt, handle, "VK_KHR_portability_subset");
        VkPhysicalDeviceMemoryProperties memory{};
        rt.api.GetPhysicalDeviceMemoryProperties(handle, &memory);
        for (std::uint32_t h = 0; h < memory.memoryHeapCount; ++h)
            if (memory.memoryHeaps[h].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)
                info_out.device_local_bytes = std::max<std::uint64_t>(info_out.device_local_bytes,
                                                                      memory.memoryHeaps[h].size);
        rt.infos.push_back(std::move(info_out));
    }
    rt.diagnostic = std::to_string(rt.infos.size()) + " device(s)";
    rt.ready = true;
}

Runtime& runtime() {
    static Runtime rt;
    static std::once_flag once;
    std::call_once(once, [] { build_runtime(rt); });
    return rt;
}

int pick_device(int requested) {
    auto& rt = runtime();
    if (rt.infos.empty()) return -1;
    if (requested >= 0) return requested < int(rt.infos.size()) ? requested : -1;
    int best = 0;
    for (int i = 1; i < int(rt.infos.size()); ++i)
        if (type_rank(rt.infos[i].type) > type_rank(rt.infos[best].type)) best = i;
    return best;
}

// ---------------------------------------------------------------------------
// Device context
// ---------------------------------------------------------------------------

struct Buffer {
    VkBuffer handle = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
    void* mapped = nullptr;
    bool coherent = false;
};

struct Context {
    Runtime* rt = nullptr;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    std::uint32_t queue_family = 0;
    DeviceApi api;
    VkPhysicalDeviceMemoryProperties memory{};
    VkCommandPool command_pool = VK_NULL_HANDLE;
    VkCommandBuffer command = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    DeviceInfo info;
    std::string error;

    ~Context() { destroy(); }

    void destroy() {
        if (!device) return;
        if (api.DeviceWaitIdle) api.DeviceWaitIdle(device);
        if (fence && api.DestroyFence) api.DestroyFence(device, fence, nullptr);
        if (command_pool && api.DestroyCommandPool) api.DestroyCommandPool(device, command_pool, nullptr);
        if (api.DestroyDevice) api.DestroyDevice(device, nullptr);
        device = VK_NULL_HANDLE;
    }

    bool create(int index) {
        rt = &runtime();
        if (!rt->ready) { error = rt->diagnostic; return false; }
        const int chosen = pick_device(index);
        if (chosen < 0) { error = "no such Vulkan device"; return false; }
        physical = rt->handles[std::size_t(chosen)];
        info = rt->infos[std::size_t(chosen)];

        std::uint32_t families = 0;
        rt->api.GetPhysicalDeviceQueueFamilyProperties(physical, &families, nullptr);
        std::vector<VkQueueFamilyProperties> properties(families);
        if (families) rt->api.GetPhysicalDeviceQueueFamilyProperties(physical, &families, properties.data());
        bool found = false;
        for (std::uint32_t i = 0; i < families; ++i)
            if (properties[i].queueFlags & VK_QUEUE_COMPUTE_BIT) { queue_family = i; found = true; break; }
        if (!found) { error = "device has no compute queue"; return false; }

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo queue_info{};
        queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue_info.queueFamilyIndex = queue_family;
        queue_info.queueCount = 1;
        queue_info.pQueuePriorities = &priority;

        std::vector<const char*> extensions;
        if (device_has_extension(*rt, physical, "VK_KHR_portability_subset"))
            extensions.push_back("VK_KHR_portability_subset"); // Required by the spec when present.

        VkDeviceCreateInfo device_info{};
        device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        device_info.queueCreateInfoCount = 1;
        device_info.pQueueCreateInfos = &queue_info;
        device_info.enabledExtensionCount = std::uint32_t(extensions.size());
        device_info.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();
        if (rt->api.CreateDevice(physical, &device_info, nullptr, &device) != VK_SUCCESS) {
            error = "vkCreateDevice failed";
            device = VK_NULL_HANDLE;
            return false;
        }
        auto get = [&](const char* name) { return rt->api.GetDeviceProcAddr(device, name); };
#define UVPACK_LOAD(field) api.field = reinterpret_cast<PFN_vk##field>(get("vk" #field))
        UVPACK_LOAD(DestroyDevice); UVPACK_LOAD(GetDeviceQueue); UVPACK_LOAD(DeviceWaitIdle);
        UVPACK_LOAD(CreateBuffer); UVPACK_LOAD(DestroyBuffer); UVPACK_LOAD(GetBufferMemoryRequirements);
        UVPACK_LOAD(AllocateMemory); UVPACK_LOAD(FreeMemory); UVPACK_LOAD(BindBufferMemory);
        UVPACK_LOAD(MapMemory); UVPACK_LOAD(UnmapMemory);
        UVPACK_LOAD(FlushMappedMemoryRanges); UVPACK_LOAD(InvalidateMappedMemoryRanges);
        UVPACK_LOAD(CreateShaderModule); UVPACK_LOAD(DestroyShaderModule);
        UVPACK_LOAD(CreateDescriptorSetLayout); UVPACK_LOAD(DestroyDescriptorSetLayout);
        UVPACK_LOAD(CreatePipelineLayout); UVPACK_LOAD(DestroyPipelineLayout);
        UVPACK_LOAD(CreateComputePipelines); UVPACK_LOAD(DestroyPipeline);
        UVPACK_LOAD(CreateDescriptorPool); UVPACK_LOAD(DestroyDescriptorPool);
        UVPACK_LOAD(AllocateDescriptorSets); UVPACK_LOAD(UpdateDescriptorSets);
        UVPACK_LOAD(CreateCommandPool); UVPACK_LOAD(DestroyCommandPool);
        UVPACK_LOAD(AllocateCommandBuffers); UVPACK_LOAD(BeginCommandBuffer);
        UVPACK_LOAD(EndCommandBuffer); UVPACK_LOAD(ResetCommandBuffer);
        UVPACK_LOAD(CmdBindPipeline); UVPACK_LOAD(CmdBindDescriptorSets);
        UVPACK_LOAD(CmdPushConstants); UVPACK_LOAD(CmdDispatch); UVPACK_LOAD(CmdPipelineBarrier);
        UVPACK_LOAD(QueueSubmit); UVPACK_LOAD(CreateFence); UVPACK_LOAD(DestroyFence);
        UVPACK_LOAD(WaitForFences); UVPACK_LOAD(ResetFences);
#undef UVPACK_LOAD
        if (!api.GetDeviceQueue || !api.CreateBuffer || !api.CreateComputePipelines || !api.QueueSubmit) {
            error = "device is missing core compute entry points";
            destroy();
            return false;
        }
        api.GetDeviceQueue(device, queue_family, 0, &queue);
        rt->api.GetPhysicalDeviceMemoryProperties(physical, &memory);

        VkCommandPoolCreateInfo pool{};
        pool.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pool.queueFamilyIndex = queue_family;
        if (api.CreateCommandPool(device, &pool, nullptr, &command_pool) != VK_SUCCESS) {
            error = "vkCreateCommandPool failed";
            destroy();
            return false;
        }
        VkCommandBufferAllocateInfo allocate{};
        allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocate.commandPool = command_pool;
        allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocate.commandBufferCount = 1;
        if (api.AllocateCommandBuffers(device, &allocate, &command) != VK_SUCCESS) {
            error = "vkAllocateCommandBuffers failed";
            destroy();
            return false;
        }
        VkFenceCreateInfo fence_info{};
        fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        if (api.CreateFence(device, &fence_info, nullptr, &fence) != VK_SUCCESS) {
            error = "vkCreateFence failed";
            destroy();
            return false;
        }
        return true;
    }

    std::uint32_t memory_type(std::uint32_t bits, VkMemoryPropertyFlags want) const {
        for (std::uint32_t i = 0; i < memory.memoryTypeCount; ++i)
            if ((bits & (1u << i)) && (memory.memoryTypes[i].propertyFlags & want) == want) return i;
        return std::uint32_t(-1);
    }

    bool allocate(Buffer& buffer, VkDeviceSize size, bool host_visible) {
        destroy(buffer);
        if (size == 0) size = 4;
        VkBufferCreateInfo info_buffer{};
        info_buffer.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        info_buffer.size = size;
        info_buffer.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        info_buffer.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (api.CreateBuffer(device, &info_buffer, nullptr, &buffer.handle) != VK_SUCCESS) return false;
        VkMemoryRequirements requirements{};
        api.GetBufferMemoryRequirements(device, buffer.handle, &requirements);
        std::uint32_t type = std::uint32_t(-1);
        if (host_visible) {
            type = memory_type(requirements.memoryTypeBits,
                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            buffer.coherent = type != std::uint32_t(-1);
            if (!buffer.coherent)
                type = memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        } else {
            type = memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            if (type == std::uint32_t(-1))
                type = memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
        }
        if (type == std::uint32_t(-1)) { destroy(buffer); return false; }
        VkMemoryAllocateInfo allocate{};
        allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocate.allocationSize = requirements.size;
        allocate.memoryTypeIndex = type;
        if (api.AllocateMemory(device, &allocate, nullptr, &buffer.memory) != VK_SUCCESS) {
            destroy(buffer);
            return false;
        }
        if (api.BindBufferMemory(device, buffer.handle, buffer.memory, 0) != VK_SUCCESS) {
            destroy(buffer);
            return false;
        }
        buffer.size = requirements.size;
        if (host_visible && api.MapMemory(device, buffer.memory, 0, VK_WHOLE_SIZE, 0, &buffer.mapped) != VK_SUCCESS) {
            destroy(buffer);
            return false;
        }
        return true;
    }

    void destroy(Buffer& buffer) {
        if (buffer.mapped && api.UnmapMemory) api.UnmapMemory(device, buffer.memory);
        if (buffer.handle && api.DestroyBuffer) api.DestroyBuffer(device, buffer.handle, nullptr);
        if (buffer.memory && api.FreeMemory) api.FreeMemory(device, buffer.memory, nullptr);
        buffer = Buffer{};
    }

    void flush(Buffer& buffer) {
        if (!buffer.mapped || buffer.coherent || !api.FlushMappedMemoryRanges) return;
        VkMappedMemoryRange range{};
        range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
        range.memory = buffer.memory;
        range.size = VK_WHOLE_SIZE;
        api.FlushMappedMemoryRanges(device, 1, &range);
    }

    void invalidate(Buffer& buffer) {
        if (!buffer.mapped || buffer.coherent || !api.InvalidateMappedMemoryRanges) return;
        VkMappedMemoryRange range{};
        range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
        range.memory = buffer.memory;
        range.size = VK_WHOLE_SIZE;
        api.InvalidateMappedMemoryRanges(device, 1, &range);
    }
};

// Matches the shader's push-constant block exactly.
struct PushConstants {
    std::uint32_t bit_count, words_per_row, canvas_rows, island_count, trial_count, result_stride;
    std::uint32_t plan_stride;
    float frame_width, frame_height, stripe_height, bit_width;
    float initial_occupied_width, initial_occupied_height;
};

// Matches the shader's OrientationT.
struct GpuOrientation {
    std::uint32_t row_begin, row_count;
    float width, height;
    std::int32_t max_offset_bits, last_row; // Full-frame fit; per-trial frames recompute both.
    std::uint32_t probe_row = 0;
    float inv_height = 0; // See BitOrientation::inv_height.
};

constexpr std::uint32_t binding_count = 9;

} // namespace

// ---------------------------------------------------------------------------
// Public surface
// ---------------------------------------------------------------------------

bool compiled_in() { return true; }

const std::vector<DeviceInfo>& devices() { return runtime().infos; }

bool available() { return runtime().ready && !runtime().infos.empty(); }

const std::string& diagnostic() { return runtime().diagnostic; }

struct Worker::Impl {
    Context context;
    Limits limits;
    VkShaderModule module = VK_NULL_HANDLE;
    VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;

    std::array<Buffer, binding_count> buffers{};
    BitProblem problem;
    bool uploaded = false;
    std::size_t capacity = 0;       // Trials the canvas allocation covers.
    std::size_t result_stride = 0;  // Uints per trial in the result buffer.
    std::size_t plan_stride = 0;    // Uints per trial in the plan buffer.
    WorkerStats stats;
    std::string error;

    ~Impl() { teardown(); }

    void teardown() {
        if (!context.device) return;
        if (context.api.DeviceWaitIdle) context.api.DeviceWaitIdle(context.device);
        for (auto& buffer : buffers) context.destroy(buffer);
        auto& api = context.api;
        if (pipeline && api.DestroyPipeline) api.DestroyPipeline(context.device, pipeline, nullptr);
        if (pipeline_layout && api.DestroyPipelineLayout) api.DestroyPipelineLayout(context.device, pipeline_layout, nullptr);
        if (descriptor_pool && api.DestroyDescriptorPool) api.DestroyDescriptorPool(context.device, descriptor_pool, nullptr);
        if (set_layout && api.DestroyDescriptorSetLayout) api.DestroyDescriptorSetLayout(context.device, set_layout, nullptr);
        if (module && api.DestroyShaderModule) api.DestroyShaderModule(context.device, module, nullptr);
        pipeline = VK_NULL_HANDLE;
        pipeline_layout = VK_NULL_HANDLE;
        descriptor_pool = VK_NULL_HANDLE;
        set_layout = VK_NULL_HANDLE;
        module = VK_NULL_HANDLE;
    }

    bool build_pipeline() {
        auto& api = context.api;
        VkShaderModuleCreateInfo shader{};
        shader.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        shader.codeSize = trial_spv_size;
        shader.pCode = reinterpret_cast<const std::uint32_t*>(trial_spv);
        if (api.CreateShaderModule(context.device, &shader, nullptr, &module) != VK_SUCCESS) {
            error = "vkCreateShaderModule failed for the trial kernel";
            return false;
        }
        std::array<VkDescriptorSetLayoutBinding, binding_count> bindings{};
        for (std::uint32_t i = 0; i < binding_count; ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo layout{};
        layout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layout.bindingCount = binding_count;
        layout.pBindings = bindings.data();
        if (api.CreateDescriptorSetLayout(context.device, &layout, nullptr, &set_layout) != VK_SUCCESS) {
            error = "vkCreateDescriptorSetLayout failed";
            return false;
        }
        VkPushConstantRange range{};
        range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        range.offset = 0;
        range.size = sizeof(PushConstants);
        VkPipelineLayoutCreateInfo pipeline_info{};
        pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipeline_info.setLayoutCount = 1;
        pipeline_info.pSetLayouts = &set_layout;
        pipeline_info.pushConstantRangeCount = 1;
        pipeline_info.pPushConstantRanges = &range;
        if (api.CreatePipelineLayout(context.device, &pipeline_info, nullptr, &pipeline_layout) != VK_SUCCESS) {
            error = "vkCreatePipelineLayout failed";
            return false;
        }
        VkComputePipelineCreateInfo compute{};
        compute.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
        compute.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        compute.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        compute.stage.module = module;
        compute.stage.pName = "main";
        compute.layout = pipeline_layout;
        if (api.CreateComputePipelines(context.device, VK_NULL_HANDLE, 1, &compute, nullptr, &pipeline) != VK_SUCCESS) {
            error = "vkCreateComputePipelines failed";
            return false;
        }
        VkDescriptorPoolSize size{};
        size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        size.descriptorCount = binding_count;
        VkDescriptorPoolCreateInfo pool{};
        pool.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool.maxSets = 1;
        pool.poolSizeCount = 1;
        pool.pPoolSizes = &size;
        if (api.CreateDescriptorPool(context.device, &pool, nullptr, &descriptor_pool) != VK_SUCCESS) {
            error = "vkCreateDescriptorPool failed";
            return false;
        }
        VkDescriptorSetAllocateInfo allocate{};
        allocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocate.descriptorPool = descriptor_pool;
        allocate.descriptorSetCount = 1;
        allocate.pSetLayouts = &set_layout;
        if (api.AllocateDescriptorSets(context.device, &allocate, &descriptor_set) != VK_SUCCESS) {
            error = "vkAllocateDescriptorSets failed";
            return false;
        }
        return true;
    }

    void bind_descriptors() {
        std::array<VkDescriptorBufferInfo, binding_count> info{};
        std::array<VkWriteDescriptorSet, binding_count> writes{};
        for (std::uint32_t i = 0; i < binding_count; ++i) {
            info[i].buffer = buffers[i].handle;
            info[i].offset = 0;
            info[i].range = VK_WHOLE_SIZE;
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = descriptor_set;
            writes[i].dstBinding = i;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &info[i];
        }
        context.api.UpdateDescriptorSets(context.device, binding_count, writes.data(), 0, nullptr);
    }

    template <class T>
    bool upload_buffer(std::size_t binding, const std::vector<T>& data) {
        const VkDeviceSize bytes = std::max<VkDeviceSize>(4, VkDeviceSize(data.size() * sizeof(T)));
        if (!context.allocate(buffers[binding], bytes, true)) {
            error = "storage buffer allocation failed";
            return false;
        }
        if (!data.empty()) std::memcpy(buffers[binding].mapped, data.data(), data.size() * sizeof(T));
        context.flush(buffers[binding]);
        stats.bytes_uploaded += std::size_t(bytes);
        return true;
    }
};

Worker::Worker() : impl_(new Impl) {}
Worker::~Worker() = default;

std::unique_ptr<Worker> Worker::create(int device_index, Limits limits) {
    if (!available()) return nullptr;
    std::unique_ptr<Worker> worker(new Worker);
    worker->impl_->limits = limits;
    if (!worker->impl_->context.create(device_index)) return nullptr;
    // The kernel is written for a 256-thread workgroup.
    if (worker->impl_->context.info.max_workgroup_invocations < 256) return nullptr;
    if (!worker->impl_->build_pipeline()) return nullptr;
    return worker;
}

const DeviceInfo& Worker::device() const { return impl_->context.info; }

const std::string& Worker::error() const {
    return impl_->error.empty() ? impl_->context.error : impl_->error;
}

const WorkerStats& Worker::stats() const { return impl_->stats; }

void Worker::note_accepted(std::size_t count) { impl_->stats.accepted += count; }

std::size_t Worker::trial_capacity() const { return impl_->uploaded ? impl_->capacity : 0; }

bool Worker::upload(const std::vector<RasterIsland>& rasters, const FixedTarget& target) {
    auto& impl = *impl_;
    impl.uploaded = false;
    const auto start = Clock::now();
    try {
        impl.problem = build_problem(rasters, target, impl.limits.bit_shift);
    } catch (const std::exception& ex) {
        impl.error = ex.what();
        return false;
    }
    const auto& p = impl.problem;

    std::vector<std::uint32_t> island_table;
    island_table.reserve(p.island_count * 2);
    for (std::uint32_t i = 0; i < p.island_count; ++i) {
        island_table.push_back(p.orientation_begin[i]);
        island_table.push_back(p.orientation_count[i]);
    }
    std::vector<GpuOrientation> orientations;
    orientations.reserve(p.orientations.size());
    for (const auto& o : p.orientations)
        orientations.push_back({o.row_begin, o.row_count, o.width, o.height, o.max_offset_bits,
                                o.last_row, o.probe_row, o.inv_height});
    std::vector<std::uint32_t> intervals;
    intervals.reserve(p.intervals.size() * 4);
    for (const auto& iv : p.intervals) {
        intervals.push_back(iv.begin);
        intervals.push_back(iv.end);
        intervals.push_back(iv.band_lo);
        intervals.push_back(iv.band_hi);
    }

    // How many trials the canvas budget covers.
    const std::size_t canvas_words_per_trial = p.canvas_words();
    if (canvas_words_per_trial == 0) { impl.error = "empty gpu canvas"; return false; }
    const std::size_t per_trial_bytes = canvas_words_per_trial * 4 + std::size_t(p.canvas_rows) * 4;
    std::size_t capacity = impl.limits.max_canvas_bytes / std::max<std::size_t>(1, per_trial_bytes);
    capacity = std::min(capacity, impl.limits.max_trials);
    const std::uint64_t range = impl.context.info.max_storage_buffer_bytes;
    if (range > 0)
        capacity = std::min<std::size_t>(capacity, std::size_t(range / std::max<std::uint64_t>(1, canvas_words_per_trial * 4)));
    if (capacity == 0) { impl.error = "one gpu trial does not fit the canvas budget"; return false; }

    impl.capacity = capacity;
    impl.result_stride = 4 + std::size_t(p.island_count) * 4;
    impl.plan_stride = 2 + std::size_t(p.island_count) * 2; // Per-trial frame, then order, then masks.

    if (!impl.upload_buffer(0, island_table)) return false;
    if (!impl.upload_buffer(1, orientations)) return false;
    if (!impl.upload_buffer(2, p.row_start)) return false;
    if (!impl.upload_buffer(3, p.row_first_bit)) return false;
    if (!impl.upload_buffer(4, intervals)) return false;
    // Plans and results are rewritten per dispatch; the canvas is device-local scratch.
    if (!impl.context.allocate(impl.buffers[5], VkDeviceSize(capacity) * impl.plan_stride * 4, true)) {
        impl.error = "plan buffer allocation failed";
        return false;
    }
    if (!impl.context.allocate(impl.buffers[6], VkDeviceSize(capacity) * canvas_words_per_trial * 4, false)) {
        impl.error = "canvas allocation failed (" +
                     std::to_string((capacity * canvas_words_per_trial * 4) >> 20) + " MiB)";
        return false;
    }
    if (!impl.context.allocate(impl.buffers[7], VkDeviceSize(capacity) * p.canvas_rows * 4, false)) {
        impl.error = "row index allocation failed";
        return false;
    }
    if (!impl.context.allocate(impl.buffers[8], VkDeviceSize(capacity) * impl.result_stride * 4, true)) {
        impl.error = "result buffer allocation failed";
        return false;
    }
    impl.bind_descriptors();
    impl.uploaded = true;
    ++impl.stats.uploads;
    impl.stats.upload_seconds += std::chrono::duration<double>(Clock::now() - start).count();
    return true;
}

bool Worker::run(const std::vector<TrialSpec>& plans, std::vector<TrialResult>& out) {
    auto& impl = *impl_;
    out.clear();
    if (!impl.uploaded || plans.empty()) return false;
    if (plans.size() > impl.capacity) return false;
    const auto& p = impl.problem;
    auto& api = impl.context.api;

    const auto t_upload = Clock::now();
    auto* plan_data = static_cast<std::uint32_t*>(impl.buffers[5].mapped);
    for (std::size_t t = 0; t < plans.size(); ++t) {
        const auto& plan = plans[t];
        if (plan.island_order.size() != p.island_count || plan.skip_mask.size() != p.island_count) return false;
        if (!(plan.frame_shrink >= 1.0)) return false; // A trial may shrink its frame, never grow it.
        std::uint32_t* base = plan_data + t * impl.plan_stride;
        const float width = float(double(p.frame_width) / plan.frame_shrink);
        const float height = float(double(p.frame_height) / plan.frame_shrink);
        std::memcpy(base + 0, &width, 4);
        std::memcpy(base + 1, &height, 4);
        std::memcpy(base + 2, plan.island_order.data(), p.island_count * sizeof(std::uint32_t));
        std::memcpy(base + 2 + p.island_count, plan.skip_mask.data(), p.island_count * sizeof(std::uint32_t));
    }
    impl.context.flush(impl.buffers[5]);
    impl.stats.upload_seconds += std::chrono::duration<double>(Clock::now() - t_upload).count();

    PushConstants push{};
    push.bit_count = p.bit_count;
    push.words_per_row = p.words_per_row;
    push.canvas_rows = p.canvas_rows;
    push.island_count = p.island_count;
    push.trial_count = std::uint32_t(plans.size());
    push.result_stride = std::uint32_t(impl.result_stride);
    push.plan_stride = std::uint32_t(impl.plan_stride);
    push.frame_width = p.frame_width;
    push.frame_height = p.frame_height;
    push.stripe_height = p.stripe_height;
    push.bit_width = p.bit_width;
    push.initial_occupied_width = p.initial_occupied_width;
    push.initial_occupied_height = p.initial_occupied_height;

    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    api.ResetCommandBuffer(impl.context.command, 0);
    if (api.BeginCommandBuffer(impl.context.command, &begin) != VK_SUCCESS) return false;
    api.CmdBindPipeline(impl.context.command, VK_PIPELINE_BIND_POINT_COMPUTE, impl.pipeline);
    api.CmdBindDescriptorSets(impl.context.command, VK_PIPELINE_BIND_POINT_COMPUTE, impl.pipeline_layout,
                              0, 1, &impl.descriptor_set, 0, nullptr);
    api.CmdPushConstants(impl.context.command, impl.pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                         sizeof(push), &push);
    api.CmdDispatch(impl.context.command, std::uint32_t(plans.size()), 1, 1);
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    api.CmdPipelineBarrier(impl.context.command, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0, nullptr, 0, nullptr);
    if (api.EndCommandBuffer(impl.context.command) != VK_SUCCESS) return false;

    const auto t_compute = Clock::now();
    api.ResetFences(impl.context.device, 1, &impl.context.fence);
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &impl.context.command;
    if (api.QueueSubmit(impl.context.queue, 1, &submit, impl.context.fence) != VK_SUCCESS) {
        impl.error = "vkQueueSubmit failed";
        return false;
    }
    // Generous but finite: a hung device must not hang the packer.
    const std::uint64_t timeout = 120ull * 1000ull * 1000ull * 1000ull;
    if (api.WaitForFences(impl.context.device, 1, &impl.context.fence, VK_TRUE, timeout) != VK_SUCCESS) {
        impl.error = "compute dispatch timed out or was lost";
        return false;
    }
    impl.stats.compute_seconds += std::chrono::duration<double>(Clock::now() - t_compute).count();
    ++impl.stats.dispatches;
    impl.stats.trials += plans.size();

    const auto t_read = Clock::now();
    impl.context.invalidate(impl.buffers[8]);
    const auto* results = static_cast<const std::uint32_t*>(impl.buffers[8].mapped);
    out.resize(plans.size());
    for (std::size_t t = 0; t < plans.size(); ++t) {
        const std::uint32_t* base = results + t * impl.result_stride;
        TrialResult& result = out[t];
        result.valid = base[0] == 1u;
        const std::uint32_t placed = std::min<std::uint32_t>(base[1], p.island_count);
        float occupied_width = 0, occupied_height = 0;
        std::memcpy(&occupied_width, &base[2], 4);
        std::memcpy(&occupied_height, &base[3], 4);
        result.occupied_width = occupied_width;
        result.occupied_height = occupied_height;
        result.frame_shrink = plans[t].frame_shrink;
        if (!result.valid) continue;
        ++impl.stats.valid_trials;
        result.placements.reserve(placed);
        for (std::uint32_t i = 0; i < placed; ++i) {
            const std::uint32_t* slot = base + 4 + i * 4;
            const std::uint32_t island = slot[0];
            if (island >= p.island_count) { result.valid = false; break; }
            Placement placement;
            placement.island_id = p.island_ids[island];
            placement.orientation = slot[1];
            placement.offset = {double(slot[3]) * double(p.bit_width),
                                double(slot[2]) * double(p.stripe_height)};
            placement.tile = 0;
            const auto& orientation = p.orientations[p.orientation_begin[island] + slot[1]];
            placement.near_spanning = nearly_spans_target(
                orientation.width, orientation.height,
                double(p.frame_width) / plans[t].frame_shrink,
                double(p.frame_height) / plans[t].frame_shrink);
            result.placements.push_back(placement);
        }
        if (!result.valid) { --impl.stats.valid_trials; result.placements.clear(); }
    }
    impl.stats.readback_seconds += std::chrono::duration<double>(Clock::now() - t_read).count();
    return true;
}

std::string self_test(int device_index) {
    if (!available()) return diagnostic();
    Context context;
    if (!context.create(device_index)) return context.error.empty() ? "device creation failed" : context.error;
    auto& api = context.api;

    VkShaderModule module = VK_NULL_HANDLE;
    VkShaderModuleCreateInfo shader{};
    shader.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shader.codeSize = probe_spv_size;
    shader.pCode = reinterpret_cast<const std::uint32_t*>(probe_spv);
    if (api.CreateShaderModule(context.device, &shader, nullptr, &module) != VK_SUCCESS)
        return "vkCreateShaderModule failed for the probe";

    std::string failure;
    VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool pool = VK_NULL_HANDLE;
    Buffer buffer;
    const std::uint32_t count = 1024;
    const std::uint32_t seed = 12345u;
    do {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        VkDescriptorSetLayoutCreateInfo layout{};
        layout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layout.bindingCount = 1;
        layout.pBindings = &binding;
        if (api.CreateDescriptorSetLayout(context.device, &layout, nullptr, &set_layout) != VK_SUCCESS) {
            failure = "descriptor set layout failed";
            break;
        }
        VkPushConstantRange range{};
        range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        range.size = 8;
        VkPipelineLayoutCreateInfo pipeline_info{};
        pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipeline_info.setLayoutCount = 1;
        pipeline_info.pSetLayouts = &set_layout;
        pipeline_info.pushConstantRangeCount = 1;
        pipeline_info.pPushConstantRanges = &range;
        if (api.CreatePipelineLayout(context.device, &pipeline_info, nullptr, &pipeline_layout) != VK_SUCCESS) {
            failure = "pipeline layout failed";
            break;
        }
        VkComputePipelineCreateInfo compute{};
        compute.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
        compute.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        compute.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        compute.stage.module = module;
        compute.stage.pName = "main";
        compute.layout = pipeline_layout;
        if (api.CreateComputePipelines(context.device, VK_NULL_HANDLE, 1, &compute, nullptr, &pipeline) != VK_SUCCESS) {
            failure = "compute pipeline failed";
            break;
        }
        if (!context.allocate(buffer, count * 4, true)) { failure = "buffer allocation failed"; break; }
        std::memset(buffer.mapped, 0, count * 4);
        context.flush(buffer);

        VkDescriptorPoolSize size{};
        size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        size.descriptorCount = 1;
        VkDescriptorPoolCreateInfo pool_info{};
        pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_info.maxSets = 1;
        pool_info.poolSizeCount = 1;
        pool_info.pPoolSizes = &size;
        if (api.CreateDescriptorPool(context.device, &pool_info, nullptr, &pool) != VK_SUCCESS) {
            failure = "descriptor pool failed";
            break;
        }
        VkDescriptorSet set = VK_NULL_HANDLE;
        VkDescriptorSetAllocateInfo allocate{};
        allocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocate.descriptorPool = pool;
        allocate.descriptorSetCount = 1;
        allocate.pSetLayouts = &set_layout;
        if (api.AllocateDescriptorSets(context.device, &allocate, &set) != VK_SUCCESS) {
            failure = "descriptor set allocation failed";
            break;
        }
        VkDescriptorBufferInfo info{};
        info.buffer = buffer.handle;
        info.range = VK_WHOLE_SIZE;
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = set;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.pBufferInfo = &info;
        api.UpdateDescriptorSets(context.device, 1, &write, 0, nullptr);

        VkCommandBufferBeginInfo begin{};
        begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        api.BeginCommandBuffer(context.command, &begin);
        api.CmdBindPipeline(context.command, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
        api.CmdBindDescriptorSets(context.command, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout,
                                  0, 1, &set, 0, nullptr);
        const std::uint32_t push[2] = {count, seed};
        api.CmdPushConstants(context.command, pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, 8, push);
        api.CmdDispatch(context.command, (count + 63) / 64, 1, 1);
        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        api.CmdPipelineBarrier(context.command, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0, nullptr, 0, nullptr);
        api.EndCommandBuffer(context.command);

        api.ResetFences(context.device, 1, &context.fence);
        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &context.command;
        if (api.QueueSubmit(context.queue, 1, &submit, context.fence) != VK_SUCCESS) {
            failure = "queue submit failed";
            break;
        }
        if (api.WaitForFences(context.device, 1, &context.fence, VK_TRUE, 10ull * 1000000000ull) != VK_SUCCESS) {
            failure = "probe dispatch did not complete";
            break;
        }
        context.invalidate(buffer);
        const auto* values = static_cast<const std::uint32_t*>(buffer.mapped);
        for (std::uint32_t i = 0; i < count; ++i) {
            const std::uint32_t expected = i * 2654435761u + seed;
            if (values[i] != expected) {
                failure = "probe mismatch at " + std::to_string(i);
                break;
            }
        }
    } while (false);

    context.destroy(buffer);
    if (pool) api.DestroyDescriptorPool(context.device, pool, nullptr);
    if (pipeline) api.DestroyPipeline(context.device, pipeline, nullptr);
    if (pipeline_layout) api.DestroyPipelineLayout(context.device, pipeline_layout, nullptr);
    if (set_layout) api.DestroyDescriptorSetLayout(context.device, set_layout, nullptr);
    if (module) api.DestroyShaderModule(context.device, module, nullptr);
    return failure;
}

} // namespace uvpack::gpu
