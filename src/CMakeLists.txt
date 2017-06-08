#=============================================================================
#
# CMake configuration file for the main FRyDoM library.
#
#=============================================================================

message(STATUS "==== ${CMAKE_PROJECT_NAME} core module ====")


#-----------------------------------------------------------------------------
# Collect files
#-----------------------------------------------------------------------------

# Core group

set(FRyDoM_core_SOURCES
        core/FrForce.cpp
        core/FrOffshoreSystem.cpp)

set(FRyDoM_core_HEADERS
        core/FrForce.h
        core/FrOffshoreSystem.h)

# Environment group

set(FRyDoM_environment_SOURCES
        environment/waves/FrFreeSurface.cpp
        environment/waves/FrFlatFreeSurface.cpp)

set(FRyDoM_environment_HEADERS
        environment/waves/FrFreeSurface.h
        environment/waves/FrFlatFreeSurface.h)

# Hydrodynamics group

set(FRyDoM_hydrodynamics_SOURCES
        hydrodynamics/FrTryalForce.cpp)

set(FRyDoM_hydrodynamics_HEADERS
        hydrodynamics/FrTryalForce.h)

# misc group

set(FRyDoM_misc_SOURCES
        misc/FrTriangleMeshConnected.cpp)

set(FRyDoM_misc_HEADERS
        misc/FrTriangleMeshConnected.h)

# utils group

set(FRyDoM_utils_SOURCES
        utils/FrIrrApp.cpp)

set(FRyDoM_utils_HEADERS
        utils/FrIrrApp.h)


# Collect all source and headers files
set(FRyDoM_FILES
        ${FRyDoM_core_SOURCES}
        ${FRyDoM_core_HEADERS}
        ${FRyDoM_environment_SOURCES}
        ${FRyDoM_environment_HEADERS}
        ${FRyDoM_hydrodynamics_SOURCES}
        ${FRyDoM_hydrodynamics_HEADERS}
        ${FRyDoM_misc_SOURCES}
        ${FRyDoM_misc_HEADERS}
        ${FRyDoM_utils_SOURCES}
        ${FRyDoM_utils_HEADERS}
        )

# Make FRyDoM_FILES visible in parent scope
set(FRyDoM_FILES "${FRyDoM_FILES}" PARENT_SCOPE)


