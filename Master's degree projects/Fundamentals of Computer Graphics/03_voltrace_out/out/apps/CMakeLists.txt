add_subdirectory(ypathtrace)
add_subdirectory(ypathtrace_adp)

if(YOCTO_OPENGL)
add_subdirectory(yipathtraces)
add_subdirectory(yipathtraces_adp)
endif(YOCTO_OPENGL)
