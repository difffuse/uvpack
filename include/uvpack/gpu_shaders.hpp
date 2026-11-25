#pragma once
// SPIR-V modules embedded at build time by cmake/embed_spv.cmake. The .spv files are
// committed under shaders/, so a build without glslangValidator embeds
// those instead of recompiling them.
#include <cstddef>

namespace uvpack::gpu {
extern const unsigned char probe_spv[];
extern const std::size_t probe_spv_size;
extern const unsigned char trial_spv[];
extern const std::size_t trial_spv_size;
} // namespace uvpack::gpu
