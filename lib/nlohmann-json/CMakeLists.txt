add_library(nlohmann-json INTERFACE)

target_include_directories(nlohmann-json SYSTEM INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
