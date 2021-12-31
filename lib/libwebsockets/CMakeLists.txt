FIND_PACKAGE(PkgConfig)

if (PKG_CONFIG_FOUND)
    pkg_check_modules(LWS libwebsockets>=2.0)

    if (LWS_FOUND)
        PKG_CONFIG_LIB_RESOLVE(LWS LWS_LIB)
        ADD_IMPORTED_LIB(websockets "${LWS_INCLUDEDIR}" "${LWS_LIBS}")

        SET(LWS_LIBS websockets CACHE INTERNAL "libwebsocket library" FORCE)
    endif()
endif()