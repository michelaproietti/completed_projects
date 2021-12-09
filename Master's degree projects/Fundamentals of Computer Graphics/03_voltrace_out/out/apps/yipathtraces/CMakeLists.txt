add_executable(yipathtraces yipathtraces.cpp)

set_target_properties(yipathtraces PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yipathtraces PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yipathtraces yocto yocto_gui yocto_pathtrace)
