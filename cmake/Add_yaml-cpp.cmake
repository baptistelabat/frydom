

#find_package(yaml-cpp QUIET)
if (NOT yaml-cpp_FOUND)
    include(FetchContent)


#    message(STATUS "YAML-CPP PATCH ${PROJECT_SOURCE_DIR}")



    set(yaml-cpp_URL "https://github.com/jbeder/yaml-cpp.git")
    FetchContent_Declare(yaml-cpp
            GIT_REPOSITORY ${yaml-cpp_URL}
            GIT_TAG "yaml-cpp-0.6.2"
            PATCH_COMMAND git apply "${PROJECT_SOURCE_DIR}/cmake/patches/yaml-cpp.patch"
            )

    FetchContent_GetProperties(yaml-cpp)
    message(STATUS yaml-cpp)
    message(STATUS ${yaml-cpp_POPULATED})
    message(STATUS ${yaml-cpp_SOURCE_DIR})
    message(STATUS ${yaml-cpp_BINARY_DIR})
    if(NOT yaml-cpp_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'yaml-cpp' dependency")
        FetchContent_Populate(yaml-cpp)

        # yaml-cpp BUILD OPTIONS
        set(BUILD_SHARED_LIBS TRUE CACHE BOOL "" FORCE)
        set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)# CACHE BOOL "Build yamp-cpp tests")
        set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
        set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)


        # TODO: ici, on peut reconstuire la target yaml-cpp a base de find_path et find_library

        add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})

        message(STATUS "DONE")
    endif()
endif()


if (TARGET yaml-cpp)
#    message(STATUS "FOUND yaml-cpp TARGET")
    get_target_property(INC yaml-cpp INCLUDE_DIRECTORIES)
    message(STATUS "YAML CPP TARGET : ${INC}")

endif()