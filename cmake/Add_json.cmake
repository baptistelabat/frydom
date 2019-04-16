

find_package(json QUIET)
if (NOT JSON_FOUND)
    include(FetchContent)

    FetchContent_Declare(json
            GIT_REPOSITORY ${json_URL}
            GIT_TAG ${json_TAG}
            )

    FetchContent_GetProperties(json)
    message(STATUS json)
    message(STATUS ${json_POPULATED})
    message(STATUS ${json_SOURCE_DIR})
    message(STATUS ${json_BINARY_DIR})
    if(NOT json_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'json' dependency")
        FetchContent_Populate(json)

        # json BUILD OPTIONS
        set(BUILD_SHARED_LIBS TRUE CACHE BOOL "" FORCE)
        set(JSON_BUILD_TESTS OFF CACHE BOOL "" FORCE)# CACHE BOOL "Build json tests")
        set(JSON_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
        set(JSON_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)


        # TODO: ici, on peut reconstuire la target json a base de find_path et find_library

        add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()


if (TARGET json)
#    message(STATUS "FOUND json TARGET")
    get_target_property(INC json INCLUDE_DIRECTORIES)
    message(STATUS "JSON TARGET : ${INC}")

endif()
