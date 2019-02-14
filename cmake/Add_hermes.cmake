
find_package(hermes QUIET)

if (NOT hermes_FOUND)
    include(FetchContent)

    #    set(FETCHCONTENT_QUIET OFF)

    set(hermes_URL "git@frydom-ce.org:ce/hermes.git")
    FetchContent_Declare(hermes
            GIT_REPOSITORY ${hermes_URL}
            GIT_TAG master
            )

    FetchContent_GetProperties(hermes)
    #        message(STATUS hermes)
    #        message(STATUS ${hermes_POPULATED})
    #        message(STATUS ${hermes_SOURCE_DIR})
    #        message(STATUS ${hermes_BINARY_DIR})
    if(NOT hermes_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'hermes' dependency")
        FetchContent_Populate(hermes)

        # hermes BUILD OPTIONS
        set(HERMES_BUILD_TESTS OFF CACHE BOOL "" FORCE)

        add_subdirectory(${hermes_SOURCE_DIR} ${hermes_BINARY_DIR})
    else()
        message(STATUS "hermes already populated")
    endif()
endif()

if (TARGET hermes)
    get_target_property(INC hermes INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found hermes : ${INC}")
else()
    message(STATUS "hermes target NOT FOUND")
endif()
