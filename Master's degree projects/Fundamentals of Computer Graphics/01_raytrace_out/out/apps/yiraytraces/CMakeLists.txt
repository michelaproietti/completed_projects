add_executable(yiraytraces yiraytraces.cpp)

set_target_properties(yiraytraces PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yiraytraces PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yiraytraces yocto yocto_gui yocto_raytrace)
