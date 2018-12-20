
# catway BUILD OPTIONS
set(CATWAY_BUILD_TESTS OFF)

find_package(catway QUIET)

if (NOT catway_FOUND)
    include(FetchContent)

    #    set(FETCHCONTENT_QUIET OFF)

    set(catway_URL "git@d-ice.githost.io:frongere/catenary.git")
    FetchContent_Declare(catway
            GIT_REPOSITORY ${catway_URL}
            GIT_TAG develop
            )

    FetchContent_GetProperties(catway)
    #        message(STATUS hermes)
    #        message(STATUS ${hermes_POPULATED})
    #        message(STATUS ${hermes_SOURCE_DIR})
    #        message(STATUS ${hermes_BINARY_DIR})
    if(NOT catway_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'catway' dependency")
        FetchContent_Populate(catway)


        add_subdirectory(${catway_SOURCE_DIR} ${catway_BINARY_DIR})
    else()
        message(STATUS "catway already populated")
    endif()
endif()

if (TARGET CatenaryEngine)
    get_target_property(INC CatenaryEngine INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found catway : ${INC}")
else()
    message(STATUS "catway target NOT FOUND")
endif()