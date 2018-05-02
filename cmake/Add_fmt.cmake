find_package(fmt QUIET)
if (NOT fmt_FOUND)
    include(FetchContent)

    set(fmt_URL https://github.com/fmtlib/fmt.git)
    FetchContent_Declare(fmt
            GIT_REPOSITORY ${fmt_URL}
            GIT_TAG 4.1.0
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
        set(BUILD_SHARED_LIBS TRUE)
        set(FMT_TEST OFF)

        add_subdirectory(${fmt_SOURCE_DIR} ${fmt_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()

if (TARGET fmt)

    get_target_property(DIR fmt INCLUDE_DIRECTORIES)
    message("Found fmt : ${DIR}")

endif()