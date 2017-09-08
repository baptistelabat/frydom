# CMakeLists.txt file for the FRyDoM project

cmake_minimum_required(VERSION 2.8)

# Project's name
project(frydom)

# Setting the configuration of the build (Debug, Release...)
set(CMAKE_BUILD_TYPE Debug)  #TODO: See if it is really used...

# Project's C++ dialect
set(CMAKE_CXX_STANDARD 14)

# PATH TO THE CHRONO BUILD DIR INTO THE FRyDoM PROJECT (MAY CHANGE)
# Used by find_package command and has to point to the chrono directory that has the ChronoConfig.cmake file
set(Chrono_DIR "${CMAKE_CURRENT_SOURCE_DIR}/frydom/core/chrono/build/cmake")
#message(${Chrono_DIR})

# FIXME: the following does not make really sense wrt. the variable it creates... (taken from project_template...)
LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib64")
#message(${CMAKE_PREFIX_PATH})

# Find CHRONO package
find_package(Chrono
             COMPONENTS Irrlicht fea postprocess
             CONFIG)

#message(${CMAKE_PREFIX_PATH})

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
    message("Could not find Chrono or one of its required modules")
    return()
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------
include_directories(${CHRONO_INCLUDE_DIRS})


#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    set(EXTRA_COMPILE_FLAGS "/wd4275")             # disable warnings triggered by Irrlicht
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

#--------------------------------------------------------------
# Add the YAML-CPP project
#--------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/yaml-cpp/include)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/yaml-cpp)  // Cette ligne charge le CMakeLists de yaml-cpp...
set(YAML_CPP_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/yaml-cpp/build/libyaml-cpp.so")

#--------------------------------------------------------------
# Specifies frydom project
#--------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#--------------------------------------------------------------
# Add the executable from the project and specify all C++
# files in the targets.
#--------------------------------------------------------------
# TODO : faire en sorte de declarer les target dans un CMakeLists.txt separe par target !!
set(SOURCE_FILES

#        src/frydom/core/FrCore.h

        src/frydom/core/FrConstants.h
        src/frydom/core/FrConstants.cpp

        src/frydom/core/FrOffshoreSystem.h
        src/frydom/core/FrOffshoreSystem.cpp

        src/frydom/core/FrEulerAngles.h
        src/frydom/core/FrEulerAngles.cpp



        src/frydom/IO/FrLoader.h
        src/frydom/IO/FrLoader.cpp

        src/frydom/misc/FrUtils.h
        src/frydom/misc/FrUtils.cpp
        src/frydom/misc/FrMathUtils.h
        src/frydom/misc/FrInterp1d.h
        src/frydom/misc/FrInterp1d.cpp
        src/frydom/misc/FrLookupTable1D.h

        src/frydom/environment/FrEnvironment.h
        src/frydom/environment/FrConventions.h

        src/frydom/environment/waves/FrFreeSurface.h
        src/frydom/environment/waves/FrFreeSurface.cpp
        src/frydom/environment/waves/FrFlatFreeSurface.h
        src/frydom/environment/waves/FrFlatFreeSurface.cpp


        src/frydom/environment/current/FrCurrent.h
        src/frydom/environment/current/FrCurrent.cpp
        src/frydom/environment/current/FrCurrentForce.h
        src/frydom/environment/current/FrCurrentForce.cpp
        src/frydom/environment/current/FrCurrentPolarCoeffs.h
        src/frydom/environment/current/FrCurrentPolarCoeffs.cpp

        src/frydom/misc/FrLinspace.h

        src/frydom/misc/FrTriangleMeshConnected.h
        src/frydom/misc/FrTriangleMeshConnected.cpp

        src/frydom/core/FrBody.h
        src/frydom/core/FrBody.cpp

        src/frydom/core/FrHydroBody.h
        src/frydom/core/FrHydroBody.cpp

        src/frydom/core/FrNode.h
        src/frydom/core/FrNode.cpp

        src/frydom/core/FrShip.h
        src/frydom/core/FrShip.cpp

        src/frydom/core/FrForce.h
        src/frydom/core/FrForce.cpp


        src/frydom/propeller/FrPropeller.h
        src/frydom/propeller/FrPropeller.cpp
        src/frydom/propeller/FrPropellerAzimuthal.h
        src/frydom/propeller/FrPropellerAzimuthal.cpp

        src/frydom/core/FrForceAsset.h
        src/frydom/core/FrForceAsset.cpp

        src/frydom/utils/FrIrrApp.h
        src/frydom/utils/FrIrrApp.cpp

        src/frydom/hydrodynamics/FrTryalForce.h
        src/frydom/hydrodynamics/FrTryalForce.cpp

        src/frydom/hydrodynamics/FrITTC57.h
        src/frydom/hydrodynamics/FrITTC57.cpp

#        src/frydom/catenary/FrCatenaryNode.h
#        src/frydom/catenary/FrCatenaryNode.cpp

        src/frydom/catenary/FrCatenaryLine.h
        src/frydom/catenary/FrCatenaryLine.cpp

        src/frydom/catenary/FrCatenaryForce.h
        src/frydom/catenary/FrCatenaryForce.cpp

        )


# TODO: mettre dans un foreach !!
# To add a target
add_executable(demo_free_surface ${SOURCE_FILES} src/frydom/tests/demo_FreeSurface.cpp)
target_link_libraries(demo_free_surface ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(demo_O3 ${SOURCE_FILES} src/frydom/tests/demo_O3_juin_2017.cpp)
target_link_libraries(demo_O3 ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(demo_rotation ${SOURCE_FILES} src/frydom/tests/demo_rotation.cpp)
target_link_libraries(demo_rotation ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(demo_ship_orientation ${SOURCE_FILES} src/frydom/tests/demo_ship_orientation.cpp)
target_link_libraries(demo_ship_orientation ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_interp1d ${SOURCE_FILES} src/frydom/tests/test_interp1d.cpp)
target_link_libraries(test_interp1d ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_linspace ${SOURCE_FILES} src/frydom/tests/test_linspace.cpp)
target_link_libraries(test_linspace ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_current_force ${SOURCE_FILES} src/frydom/tests/test_current_force.cpp)
target_link_libraries(test_current_force ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_lookup_table ${SOURCE_FILES} src/frydom/tests/test_lookup_table.cpp)
target_link_libraries(test_lookup_table ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_current_polar_coeffs ${SOURCE_FILES} src/frydom/tests/test_CurrentPolarCoeffs.cpp)
target_link_libraries(test_current_polar_coeffs ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_yaml src/frydom/tests/test_cpp_yaml.cpp)
target_link_libraries(test_yaml ${YAML_CPP_LIB})

add_executable(test_catenary ${SOURCE_FILES} src/frydom/tests/test_catenary.cpp)
target_link_libraries(test_catenary ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})

add_executable(test_tug ${SOURCE_FILES} src/frydom/tests/test_tug.cpp)
target_link_libraries(test_tug ${CHRONO_LIBRARIES} ${YAML_CPP_LIB})



#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#add_DLL_copy_command("${CHRONO_DLLS}")
