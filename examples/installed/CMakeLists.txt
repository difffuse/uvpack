cmake_minimum_required(VERSION 3.19)
project(uvpack_installed_example LANGUAGES CXX)
find_package(uvpack CONFIG REQUIRED)
# Exercise the static SDK inside a shared consumer too (PIC and dependency closure).
add_library(embedded_pack SHARED embedded.cpp)
set_target_properties(embedded_pack PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
target_link_libraries(embedded_pack PRIVATE uvpack::core)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE embedded_pack uvpack::core)
