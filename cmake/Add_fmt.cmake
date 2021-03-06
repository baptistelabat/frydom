find_package(fmt QUIET)
if (NOT fmt_FOUND)
    include(FetchContent)

    FetchContent_Declare(fmt
            GIT_REPOSITORY ${fmt_URL}
            GIT_TAG ${fmt_TAG}
            )

    FetchContent_GetProperties(fmt)
    message(STATUS FMT)
    message(STATUS ${fmt_POPULATED})
    message(STATUS ${fmt_SOURCE_DIR})
    message(STATUS ${fmt_BINARY_DIR})
    if(NOT fmt_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'fmt' dependency")
        FetchContent_Populate(fmt)

        # FMT BUILD OPTIONS
        set(BUILD_SHARED_LIBS TRUE CACHE BOOL "" FORCE)
        set(FMT_TEST OFF CACHE BOOL "" FORCE)

        add_subdirectory(${fmt_SOURCE_DIR} ${fmt_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()

if (TARGET fmt)

    get_target_property(DIR fmt INCLUDE_DIRECTORIES)
    message("Found fmt : ${DIR}")

endif()
