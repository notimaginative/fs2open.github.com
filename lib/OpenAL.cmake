
# We have precompiled libs for windows
IF (WIN32)
    get_prebuilt_path(PREBUILT_PATH)

    add_library(openal SHARED IMPORTED GLOBAL)

    set(dll_name "${PREBUILT_PATH}/openal/bin/OpenAL32.dll")

    set_target_properties(openal
            PROPERTIES
            IMPORTED_LOCATION "${dll_name}"
            IMPORTED_IMPLIB "${PREBUILT_PATH}/openal/libs/${CMAKE_IMPORT_LIBRARY_PREFIX}OpenAL32${CMAKE_IMPORT_LIBRARY_SUFFIX}"
            INTERFACE_INCLUDE_DIRECTORIES "${PREBUILT_PATH}/openal/include"
            )

    add_target_copy_files("${dll_name}")
ELSE(WIN32)
    if(PLATFORM_MAC)
        option(OPENAL_USE_PRECOMPILED "Use precompiled version of OpenAL. If disabled the system libraries will be used." ON)
    endif()

    if(NOT OPENAL_USE_PRECOMPILED)
        FIND_PACKAGE(OpenAL REQUIRED)

        INCLUDE(util)

        ADD_IMPORTED_LIB(openal ${OPENAL_INCLUDE_DIR} ${OPENAL_LIBRARY})
    else()
        message(STATUS "Using pre-built OpenAL libraries.")

        get_prebuilt_path(PREBUILT_PATH)
        set(OPENAL_PATH "${PREBUILT_PATH}/openal")

        find_library(openal_LOCATION openal
            PATHS "${OPENAL_PATH}/lib"
            NO_DEFAULT_PATH)

        file(GLOB openal_LIBS "${OPENAL_PATH}/lib/libopenal*")

        get_filename_component(FULL_LIB_PATH "${openal_LOCATION}" REALPATH)

        add_imported_lib("openal" "${OPENAL_PATH}/include/AL" "${FULL_LIB_PATH}")

        add_target_copy_files("${openal_LIBS}")
    endif()
ENDIF(WIN32)
