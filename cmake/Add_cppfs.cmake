find_package(cppfs QUIET)
if (NOT cppfs_FOUND)
    include(FetchContent)

    FetchContent_Declare(cppfs
            GIT_REPOSITORY ${cppfs_URL}
            GIT_TAG ${cppfs_TAG}
            )

    FetchContent_GetProperties(cppfs)
    message(STATUS CPPFS)
    message(STATUS ${cppfs_POPULATED})
    message(STATUS ${cppfs_SOURCE_DIR})
    message(STATUS ${cppfs_BINARY_DIR})
    if(NOT cppfs_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'cppfs' dependency")
        FetchContent_Populate(cppfs)

        # CPPFS BUILD OPTIONS
#        set(BUILD_SHARED_LIBS TRUE CACHE BOOL "" FORCE)
        set(OPTION_BUILD_TESTS OFF CACHE BOOL "" FORCE)

        add_subdirectory(${cppfs_SOURCE_DIR} ${cppfs_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()

if (TARGET cppfs)

    get_target_property(DIR cppfs INCLUDE_DIRECTORIES)
    message("Found cppfs : ${DIR}")

endif()
