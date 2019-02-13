find_package(mathutils QUIET)

if (NOT mathutils_FOUND)
    include(FetchContent)

#    set(FETCHCONTENT_QUIET OFF)

    set(MathUtils_URL "git@frydom-ce.org:frydom-ce/mathutils.git") # TODO: pointer vers un depot git distant...
    FetchContent_Declare(mathutils
            GIT_REPOSITORY ${MathUtils_URL}
            GIT_TAG master
            )

    FetchContent_GetProperties(mathutils)
#        message(STATUS MATHUTILS)
#        message(STATUS ${mathutils_POPULATED})
#        message(STATUS ${mathutils_SOURCE_DIR})
#        message(STATUS ${mathutils_BINARY_DIR})
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
