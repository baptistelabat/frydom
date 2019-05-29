
find_package(mathutils QUIET)

if (NOT mathutils_FOUND)
    include(FetchContent)

#    set(FETCHCONTENT_QUIET OFF)

    FetchContent_Declare(mathutils
            GIT_REPOSITORY ${mathutils_URL}
            GIT_TAG ${mathutils_TAG}
            )

    FetchContent_GetProperties(mathutils)
    if(NOT mathutils_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'MathUtils' dependency")
        FetchContent_Populate(mathutils)

        # MathUtils BUILD OPTIONS
        set(MATHUTILS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(ADD_MATPLOTLIB_CPP ON CACHE BOOL "" FORCE)

        add_subdirectory(${mathutils_SOURCE_DIR} ${mathutils_BINARY_DIR})
    else()
        message(STATUS "MathUtils already populated")
    endif()
endif()

if (TARGET MathUtils::MathUtils)
    get_target_property(INC MathUtils::MathUtils INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "Found MathUtils : ${INC}")
else()
    message(STATUS "MathUtils target NOT FOUND")
endif()
