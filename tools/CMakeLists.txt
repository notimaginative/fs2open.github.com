# Add embedfile as an external project so we can build it for the native platform
# instead of needing crazy cross-compile magic
include(ExternalProject)

ExternalProject_Add(embedfile
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/embedfile
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}
    INSTALL_COMMAND ""
)

# Reference ${_embedfile} as the command to trigger build/use of embedfile
set(_embedfile "${CMAKE_CURRENT_BINARY_DIR}/embedfile${CMAKE_EXECUTABLE_SUFFIX}" CACHE INTERNAL "")


# Now add the optional tools
if (FSO_BUILD_TOOLS)
    ADD_SUBDIRECTORY(strings_tool)
endif ()
