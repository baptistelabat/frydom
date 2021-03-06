

#cmake_policy(SET CMP0046 NEW) # To remove some cmake warnings (hack)


find_package(chrono QUIET)
if (NOT chrono_FOUND)
    include(FetchContent)

    FetchContent_Declare(chrono
            GIT_REPOSITORY ${chrono_URL}
            GIT_TAG ${chrono_TAG}
            GIT_PROGRESS 1
            PATCH_COMMAND git apply ${PROJECT_SOURCE_DIR}/cmake/patches/${chrono_PATCH}
            )


    FetchContent_GetProperties(chrono)
    if(NOT chrono_POPULATED)

        message(STATUS "Downloading, Configuring and Generating 'chrono' dependency")
        FetchContent_Populate(chrono)


        list(APPEND CMAKE_MODULE_PATH "${chrono_SOURCE_DIR}")

        # chrono BUILD OPTIONS
        set(CMAKE_SOURCE_DIR ${chrono_SOURCE_DIR})
        set(BUILD_TESTS FALSE CACHE BOOL "" FORCE)
        set(BUILD_DEMOS FALSE CACHE BOOL "" FORCE)
        set(BUILD_DEMOS_BASE FALSE CACHE BOOL "" FORCE)
        set(BUILD_DEMOS_FEA FALSE CACHE BOOL "" FORCE)
        set(BUILD_DEMOS_IRRLICHT FALSE CACHE BOOL "" FORCE)
        set(BUILD_DEMOS_POSTPROCESS FALSE CACHE BOOL "" FORCE)
        set(BUILD_TESTING FALSE CACHE BOOL "" FORCE)
        set(BUILD_TESTS_BASE FALSE CACHE BOOL "" FORCE)
        set(BUILD_TESTS_FEA FALSE CACHE BOOL "" FORCE)
        set(BUILD_BENCHMARKING FALSE CACHE BOOL "" FORCE)


        set(ENABLE_MODULE_CASCADE OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_COSIMULATION OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_FEA ON CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_FSI OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_IRRLICHT ON CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_MATLAB OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_MKL OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_OPENGL OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_PARALLEL OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_POSTPROCESS ON CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_PYTHON OFF CACHE BOOL "" FORCE)
        set(ENABLE_MODULE_VEHICLE OFF CACHE BOOL "" FORCE)
        set(ENABLE_OPENMP ON CACHE BOOL "" FORCE)


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

# TODO: ajouter une custom_target rassemblat toutes les libs delectionnees pour Chrono avec des add_dependencies()
