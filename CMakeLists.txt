cmake_minimum_required(VERSION 3.19)
project(uvpack LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

# One source for the CLI handshake, native constants and Python distribution.
set(UVPACK_METADATA_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/uvpack_geometry/release.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${UVPACK_METADATA_FILE}")
file(READ "${UVPACK_METADATA_FILE}" UVPACK_RELEASE_JSON)
string(JSON UVPACK_VERSION GET "${UVPACK_RELEASE_JSON}" version)
string(JSON UVPACK_DIALECT GET "${UVPACK_RELEASE_JSON}" dialect)
string(JSON UVPACK_PROTOCOL_API GET "${UVPACK_RELEASE_JSON}" protocol_api)
string(JSON UVPACK_COMPANION_API GET "${UVPACK_RELEASE_JSON}" companion_api)
if(NOT UVPACK_VERSION MATCHES "^[0-9]+[.][0-9]+[.][0-9]+([.]dev[0-9]+)?$" OR
   NOT UVPACK_DIALECT STREQUAL "uvpack" OR NOT UVPACK_PROTOCOL_API MATCHES "^[1-9][0-9]*$" OR
   NOT UVPACK_COMPANION_API MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "Invalid UVPack release metadata identity/API")
endif()
configure_file(cmake/version.hpp.in generated/uvpack/version.hpp @ONLY)

add_library(clipper_vendor STATIC third_party/clipper/clipper.cpp)
set_target_properties(clipper_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON
  EXPORT_NAME clipper_internal OUTPUT_NAME uvpack_clipper)
target_include_directories(clipper_vendor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/clipper>")
target_compile_features(clipper_vendor PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# Optional GPU worker (Vulkan compute). Default ON only when the headers are
# found; the library, the CLI and every test build and run unchanged without it.
# ---------------------------------------------------------------------------
find_package(Vulkan QUIET)
option(UVPACK_VULKAN "Build the Vulkan compute GPU worker" ${Vulkan_FOUND})

# SPIR-V: recompiled with glslangValidator (or glslc) when one is on PATH, otherwise
# the committed shaders/*.spv are embedded, so a build needs no shader compiler.
find_program(UVPACK_GLSL_COMPILER NAMES glslangValidator glslc)
set(UVPACK_SHADER_NAMES probe trial)
set(UVPACK_SHADER_SOURCES "")
set(UVPACK_SPV_UPDATES "")
foreach(name IN LISTS UVPACK_SHADER_NAMES)
  set(committed "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.spv")
  set(spv "${committed}")
  if(UVPACK_GLSL_COMPILER)
    set(spv "${CMAKE_CURRENT_BINARY_DIR}/shaders/${name}.spv")
    if(UVPACK_GLSL_COMPILER MATCHES "glslc")
      set(compile_args -O -fshader-stage=compute -o "${spv}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.comp")
    else()
      set(compile_args -V --target-env vulkan1.0 -S comp -o "${spv}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.comp")
    endif()
    add_custom_command(OUTPUT "${spv}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/shaders"
      COMMAND "${UVPACK_GLSL_COMPILER}" ${compile_args}
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.comp"
      COMMENT "Compiling ${name}.comp to SPIR-V"
      VERBATIM)
    list(APPEND UVPACK_SPV_UPDATES COMMAND ${CMAKE_COMMAND} -E copy_if_different "${spv}" "${committed}")
  endif()
  set(generated "${CMAKE_CURRENT_BINARY_DIR}/generated/${name}_spv.cpp")
  add_custom_command(OUTPUT "${generated}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND ${CMAKE_COMMAND} "-DSPV=${spv}" "-DOUT=${generated}" "-DSYMBOL=${name}_spv"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake"
    DEPENDS "${spv}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spv.cmake"
    COMMENT "Embedding ${name}.spv"
    VERBATIM)
  list(APPEND UVPACK_SHADER_SOURCES "${generated}")
endforeach()
if(UVPACK_SPV_UPDATES)
  # Refresh the committed .spv files after editing a shader, so a compiler-less
  # build stays in sync with the GLSL. Run it, then commit shaders/*.spv.
  add_custom_target(uvpack_update_shaders ${UVPACK_SPV_UPDATES}
    DEPENDS ${UVPACK_SHADER_SOURCES}
    COMMENT "Refreshing the committed SPIR-V")
endif()

set(UVPACK_GPU_SOURCES src/gpu_common.cpp src/gpu_reference.cpp)
# The reference kernel must evaluate every cost expression exactly as the shader does
# (correctly rounded multiplies and adds, never fused): with contraction on, a*b+c
# becomes an fma with one rounding and equal-cost ties break differently.
if(MSVC)
  set_source_files_properties(src/gpu_reference.cpp PROPERTIES COMPILE_OPTIONS "/fp:strict")
else()
  set_source_files_properties(src/gpu_reference.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
endif()
if(UVPACK_VULKAN)
  list(APPEND UVPACK_GPU_SOURCES src/gpu_vulkan.cpp ${UVPACK_SHADER_SOURCES})
else()
  list(APPEND UVPACK_GPU_SOURCES src/gpu_stub.cpp)
endif()

# The SDK deliberately ships static, PIC archives, including its private Clipper
# link dependency. No new shared-library ABI or public Boost dependency is added.
add_library(uvpack_core STATIC src/geometry.cpp src/rows.cpp src/offset.cpp
  src/stripe_placer.cpp src/fast_occupancy.cpp src/orientations.cpp src/bounding.cpp
  src/texel_density.cpp src/scale_search.cpp src/single_pass.cpp src/finish.cpp src/clusters.cpp src/overlay_groups.cpp src/obstacles.cpp src/compaction.cpp src/stripes.cpp src/validator.cpp src/interior_overlap.cpp src/snap.cpp src/tile_placer.cpp src/input_check.cpp
  ${UVPACK_GPU_SOURCES})
set_target_properties(uvpack_core PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME core)
add_library(uvpack::core ALIAS uvpack_core)
target_include_directories(uvpack_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(uvpack_core SYSTEM PRIVATE third_party/boost_multiprecision/include)
target_include_directories(uvpack_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>")
target_link_libraries(uvpack_core PRIVATE clipper_vendor Threads::Threads)
target_compile_features(uvpack_core PUBLIC cxx_std_17)
if(UVPACK_VULKAN)
  # Headers only: the loader is opened with dlopen/LoadLibrary at runtime, so the
  # binary has no link-time or load-time dependency on libvulkan.
  target_include_directories(uvpack_core PRIVATE ${Vulkan_INCLUDE_DIRS})
  target_compile_definitions(uvpack_core PUBLIC UVPACK_VULKAN=1)
  if(NOT WIN32)
    target_link_libraries(uvpack_core PRIVATE ${CMAKE_DL_LIBS})
  endif()
  message(STATUS "uvpack: Vulkan GPU worker ON (headers ${Vulkan_INCLUDE_DIRS})")
else()
  message(STATUS "uvpack: Vulkan GPU worker OFF")
endif()
if(UVPACK_GLSL_COMPILER)
  message(STATUS "uvpack: shader compiler ${UVPACK_GLSL_COMPILER}")
else()
  message(STATUS "uvpack: no shader compiler, embedding the committed SPIR-V")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(uvpack_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(uvpack_cli cli/main.cpp cli/request.cpp cli/response.cpp cli/validate.cpp cli/normalize.cpp)
set_target_properties(uvpack_cli PROPERTIES EXPORT_NAME cli)
target_link_libraries(uvpack_cli PRIVATE uvpack_core)
if(WIN32)
  target_link_libraries(uvpack_cli PRIVATE shell32)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(uvpack_smoke examples/installed/main.cpp examples/installed/embedded.cpp)
  target_link_libraries(uvpack_smoke PRIVATE uvpack_core)
  add_test(NAME packing_smoke COMMAND uvpack_smoke)
  add_test(NAME cli_version COMMAND uvpack_cli --version)
endif()

# The geometry companion and SDK ship from the same source candidate as the CLI.
install(TARGETS uvpack_core clipper_vendor uvpack_cli EXPORT uvpackTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/uvpack DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY python/uvpack_geometry
        DESTINATION ${CMAKE_INSTALL_DATADIR}/uvpack/python
        FILES_MATCHING PATTERN "*.py" PATTERN "release.json" PATTERN "__pycache__" EXCLUDE)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/uvpack/version.hpp"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/uvpack)
set(UVPACK_RUNTIME_FILES "" CACHE STRING "Explicit app-local runtime files to install beside the CLI")
set(UVPACK_RUNTIME_LICENSE_FILES "" CACHE STRING "License/notice files for the selected app-local runtimes")
if(UVPACK_RUNTIME_FILES AND NOT UVPACK_RUNTIME_LICENSE_FILES)
  message(FATAL_ERROR "Provide UVPACK_RUNTIME_LICENSE_FILES for the selected app-local runtimes")
endif()
foreach(runtime IN LISTS UVPACK_RUNTIME_FILES UVPACK_RUNTIME_LICENSE_FILES)
  if(NOT IS_ABSOLUTE "${runtime}" OR NOT EXISTS "${runtime}" OR IS_DIRECTORY "${runtime}")
    message(FATAL_ERROR "Runtime and license entries must be existing absolute file paths: ${runtime}")
  endif()
endforeach()
if(UVPACK_RUNTIME_FILES)
  install(FILES ${UVPACK_RUNTIME_FILES} DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(UVPACK_RUNTIME_LICENSE_FILES)
  install(FILES ${UVPACK_RUNTIME_LICENSE_FILES} DESTINATION ${CMAKE_INSTALL_DATADIR}/uvpack/licenses/runtime)
endif()
install(FILES LICENSE THIRD_PARTY_NOTICES.md DESTINATION ${CMAKE_INSTALL_DATADIR}/uvpack/licenses)
install(FILES third_party/boost_multiprecision/LICENSE_1_0.txt
        DESTINATION ${CMAKE_INSTALL_DATADIR}/uvpack/licenses RENAME BOOST_LICENSE_1_0.txt)
install(FILES third_party/boost_multiprecision/provenance.json
        DESTINATION ${CMAKE_INSTALL_DATADIR}/uvpack/licenses RENAME boost-provenance.json)
install(EXPORT uvpackTargets NAMESPACE uvpack:: FILE uvpackTargets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uvpack)
configure_package_config_file(cmake/uvpackConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/uvpackConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uvpack
  PATH_VARS CMAKE_INSTALL_DATADIR)
# CMake's version request syntax is numeric. Full candidate identity (including
# .devN) remains in uvpack_VERSION, the public header, companion and CLI metadata.
string(REGEX MATCH "^[0-9]+[.][0-9]+[.][0-9]+" UVPACK_NUMERIC_VERSION "${UVPACK_VERSION}")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/uvpackConfigVersion.cmake"
  VERSION "${UVPACK_NUMERIC_VERSION}" COMPATIBILITY SameMinorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/uvpackConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/uvpackConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/uvpack)
