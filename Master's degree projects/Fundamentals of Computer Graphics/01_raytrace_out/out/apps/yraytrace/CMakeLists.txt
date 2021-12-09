add_executable(yraytrace yraytrace.cpp)

set_target_properties(yraytrace PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yraytrace PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yraytrace yocto yocto_raytrace)
