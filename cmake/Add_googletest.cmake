if (NOT googletest_FOUND)
    include(FetchContent)


    message(STATUS ${googletest_URL})
    message(STATUS ${googletest_TAG})


    FetchContent_Declare(googletest
        GIT_REPOSITORY ${googletest_URL}
        GIT_TAG ${googletest_TAG}
        )

    FetchContent_GetProperties(googletest)
    message(STATUS googletest)
    message(STATUS ${googletest_POPULATED})
    message(STATUS ${googletest_SOURCE_DIR})
    message(STATUS ${googletest_BINARY_DIR})

    if(NOT googletest_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'googletest' dependency")
        FetchContent_Populate(googletest)

        # Prevent overriding the parent project's compiler/linker
        # settings on Windows
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

        # Add googletest directly to our build. This defines
        # the gtest and gtest_main targets.
        add_subdirectory(
            ${googletest_SOURCE_DIR}
            ${googletest_BINARY_DIR}
            EXCLUDE_FROM_ALL)

        message(STATUS ${googletest_SOURCE_DIR})
        message(STATUS ${googletest_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()

