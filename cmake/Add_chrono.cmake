

#cmake_policy(SET CMP0046 NEW) # To remove some cmake warnings (hack)


find_package(chrono QUIET)
if (NOT chrono_FOUND)
    include(FetchContent)

    set(chrono_URL "https://github.com/projectchrono/chrono.git")
    FetchContent_Declare(chrono
            GIT_REPOSITORY ${chrono_URL}
            GIT_TAG "3.0.0"
            GIT_PROGRESS 1
            PATCH_COMMAND git apply "${CMAKE_SOURCE_DIR}/cmake/patches/chrono_3.0.0.patch"
            )





    FetchContent_GetProperties(chrono)
    message(STATUS ${chrono_POPULATED})
    message(STATUS ${chrono_SOURCE_DIR})
    message(STATUS ${chrono_BINARY_DIR})
    if(NOT chrono_POPULATED)

        message(STATUS "Downloading, Configuring and Generating 'chrono' dependency")
        FetchContent_Populate(chrono)


        list(APPEND CMAKE_MODULE_PATH "${chrono_SOURCE_DIR}")

        # chrono BUILD OPTIONS
        set(CMAKE_SOURCE_DIR ${chrono_SOURCE_DIR})
        set(BUILD_TESTS FALSE)
        set(BUILD_DEMOS FALSE)
        set(BUILD_DEMOS_BASE FALSE)
        set(BUILD_DEMOS_FEA FALSE)
        set(BUILD_DEMOS_IRRLICHT FALSE)
        set(BUILD_DEMOS_POSTPROCESS FALSE)
        set(BUILD_TESTING FALSE)
        set(BUILD_TESTS_BASE FALSE)
        set(BUILD_TESTS_FEA FALSE)

        set(ENABLE_MODULE_CASCADE OFF)
        set(ENABLE_MODULE_COSIMULATION OFF)
        set(ENABLE_MODULE_FEA ON)
        set(ENABLE_MODULE_FSI OFF)
        set(ENABLE_MODULE_IRRLICHT ON)
        set(ENABLE_MODULE_MATLAB OFF)
        set(ENABLE_MODULE_MKL OFF)
        set(ENABLE_MODULE_OPENGL OFF)
        set(ENABLE_MODULE_PARALLEL OFF)
        set(ENABLE_MODULE_POSTPROCESS ON)
        set(ENABLE_MODULE_PYTHON OFF)
        set(ENABLE_MODULE_VEHICLE OFF)
        set(ENABLE_OPENMP ON)


        add_subdirectory(${chrono_SOURCE_DIR} ${chrono_BINARY_DIR})

        message(STATUS "DONE")
        set(chrono_DIR ${chrono_BINARY_DIR}/cmake)


    else()
        message(STATUS "Chrono already populated")
    endif()
endif()

if (TARGET ChronoEngine)
    message(STATUS "ChronoEngine TARGET FOUND")
    get_target_property(INC ChronoEngine INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine PUBLIC ${INC})
endif()

if (TARGET ChronoEngine_fea)
    message(STATUS "ChronoEngine_fea TARGET FOUND")
    get_target_property(INC ChronoEngine_fea INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_fea PUBLIC ${INC})
endif()

if (TARGET ChronoEngine_irrlicht)
    message(STATUS "ChronoEngine_irrlicht TARGET FOUND")
    get_target_property(INC ChronoEngine_irrlicht INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_irrlicht PUBLIC ${INC})
endif()

if (TARGET ChronoEngine_postprocess)
    message(STATUS "ChronoEngine_postprocess TARGET FOUND")
    get_target_property(INC ChronoEngine_postprocess INCLUDE_DIRECTORIES)
    # Chrono cmake does not embed include directories into its targets... Adding them
    target_include_directories(ChronoEngine_postprocess PUBLIC ${INC})
endif()

