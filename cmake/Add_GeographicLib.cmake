# Exports the GeographicLib target

set(MPFR_LIBRARIES "")

find_package(GeographicLib QUIET)
if (NOT GeographicLib_FOUND)
    include(FetchContent)

    set(GeographicLib_URL https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.49.tar.gz/download)
    FetchContent_Declare(GeographicLib
            URL ${GeographicLib_URL}
            PATCH_COMMAND patch < "${PROJECT_SOURCE_DIR}/cmake/patches/GeographicLib.patch"
            )

    FetchContent_GetProperties(GeographicLib)
    message(STATUS GeographicLib)
    message(STATUS ${geographiclib_POPULATED})
    message(STATUS ${geographiclib_SOURCE_DIR})
    message(STATUS ${geographiclib_BINARY_DIR})


    if(NOT geographiclib_POPULATED)
        message(STATUS "Downloading, Configuring and Generating 'GeographicLib' dependency")
        FetchContent_Populate(GeographicLib)

        # GeographicLib BUILD OPTIONS
        set(GEOGRAPHICLIB_LIB_TYPE SHARED)
        set(GEOGRAPHICLIB_DOCUMENTATION OFF)

        add_subdirectory(${geographiclib_SOURCE_DIR} ${geographiclib_BINARY_DIR})

        # Removing targets
#        set_target_properties(example-Accumulator PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)



    endif()

    set(MagneticModel_URL https://sourceforge.net/projects/geographiclib/files/magnetic-distrib/emm2017.tar.bz2)
    FetchContent_Declare(MagneticModel
            URL ${MagneticModel_URL}
            #PATCH_COMMAND patch < "${PROJECT_SOURCE_DIR}/cmake/patches/MagneticModel.patch"
            )
    FetchContent_GetProperties(MagneticModel)
    message(STATUS MagneticModel)
    message(STATUS ${magneticmodel_POPULATED})
    message(STATUS ${magneticmodel_SOURCE_DIR})
    message(STATUS ${magneticmodel_BINARY_DIR})

    if(NOT magneticmodel_POPULATED)
        message(STATUS "Downloading, Configuring and Generating magnetic models for 'GeographicLib' dependency")
        FetchContent_Populate(MagneticModel)

        # GeographicLib BUILD OPTIONS
        #set(GEOGRAPHICLIB_LIB_TYPE SHARED)
        #set(GEOGRAPHICLIB_DOCUMENTATION OFF)

        #add_subdirectory(${magneticmodel_SOURCE_DIR} ${magneticmodel_BINARY_DIR})

    endif()

endif()


if (TARGET GeographicLib)
    get_target_property(DIR GeographicLib INCLUDE_DIRECTORIES)
    message("FOUND_geographiclib : ${DIR}")
endif()