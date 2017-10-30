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
# Add the Boost library
#--------------------------------------------------------------
find_package(Boost REQUIRED filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

#--------------------------------------------------------------
# Add the YAML-CPP project
#--------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/yaml-cpp/include)
set(YAML_CPP_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/yaml-cpp/build/libyaml-cpp.so")

#--------------------------------------------------------------
# Add the Eigen project
#--------------------------------------------------------------
# TODO: passer eigen en lib dynamique pour accelerer la compilation
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")

#--------------------------------------------------------------
# Using HDF5 library
#--------------------------------------------------------------
#TODO: embarquer hdf5 dans les sous-modules...
find_package(HDF5 REQUIRED COMPONENTS CXX)
include_directories(${HDF5_INCLUDE_DIRS})

#--------------------------------------------------------------
# Specifies frydom project
#--------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#--------------------------------------------------------------
# Add the executable from the project and specify all C++
# files in the targets.
#--------------------------------------------------------------
# TODO : faire en sorte de declarer les target dans un CMakeLists.txt separe par target !!

set(LIBS
        ${CHRONO_LIBRARIES}
        ${YAML_CPP_LIB}
        ${HDF5_CXX_LIBRARIES}
        ${Boost_LIBRARIES}
        )

# TODO: partitionner les sources en fichiers CMakeLists.txt par folder de frydom pour alleger le principal

set(SOURCE_CABLE
        src/frydom/cable/FrCable.h
        src/frydom/cable/FrCable.cpp

        src/frydom/cable/FrCatenaryLine.h
        src/frydom/cable/FrCatenaryLine.cpp

        src/frydom/cable/FrCatenaryForce.h
        src/frydom/cable/FrCatenaryForce.cpp

        src/frydom/cable/FrDynamicCable.h
        src/frydom/cable/FrDynamicCable.cpp
        )

set(SOURCE_CORE

        src/frydom/core/FrOffshoreSystem.h
        src/frydom/core/FrOffshoreSystem.cpp

        src/frydom/core/FrConstants.h
        src/frydom/core/FrConstants.cpp

        src/frydom/core/FrEulerAngles.h
        src/frydom/core/FrEulerAngles.cpp

        src/frydom/core/FrBody.h
        src/frydom/core/FrBody.cpp

        src/frydom/core/FrHydroBody.h
        src/frydom/core/FrHydroBody.cpp

        src/frydom/core/FrBodyEasy.h
        src/frydom/core/FrBodyEasy.cpp

        src/frydom/core/FrNode.h
        src/frydom/core/FrNode.cpp

        src/frydom/core/FrShip.h
        src/frydom/core/FrShip.cpp

        src/frydom/core/FrForce.h
        src/frydom/core/FrForce.cpp

        src/frydom/core/FrForceAsset.h
        src/frydom/core/FrForceAsset.cpp
        )

set (SOURCE_ENVIRONMENT # TODO: par repertoire...
        src/frydom/environment/FrEnvironment.h
        src/frydom/environment/FrConventions.h

        src/frydom/environment/waves/FrFreeSurface.h
        src/frydom/environment/waves/FrFreeSurface.cpp
        src/frydom/environment/waves/FrFlatFreeSurface.h
        src/frydom/environment/waves/FrFlatFreeSurface.cpp

        src/frydom/environment/waves/FrWaveDispersionRelation.h
        src/frydom/environment/waves/FrWaveDispersionRelation.cpp

        src/frydom/environment/waves/FrWaveSpectrum.h
        src/frydom/environment/waves/FrWaveSpectrum.cpp

        src/frydom/environment/waves/FrWaveField.h
        src/frydom/environment/waves/FrWaveField.cpp

        src/frydom/environment/waves/FrWaveProbe.h
        src/frydom/environment/waves/FrWaveProbe.cpp

        src/frydom/environment/current/FrCurrent.h
        src/frydom/environment/current/FrCurrent.cpp
        src/frydom/environment/current/FrCurrentForce.h
        src/frydom/environment/current/FrCurrentForce.cpp
        src/frydom/environment/current/FrCurrentPolarCoeffs.h
        src/frydom/environment/current/FrCurrentPolarCoeffs.cpp
        )

set(SOURCE_HYDRODYNAMICS
        src/frydom/hydrodynamics/FrTryalForce.h
        src/frydom/hydrodynamics/FrTryalForce.cpp

        src/frydom/hydrodynamics/FrITTC57.h
        src/frydom/hydrodynamics/FrITTC57.cpp

        src/frydom/hydrodynamics/FrLinearDamping.h
        src/frydom/hydrodynamics/FrLinearDamping.cpp

        src/frydom/hydrodynamics/FrRadiationForce.h
        src/frydom/hydrodynamics/FrRadiationForce.cpp

        src/frydom/hydrodynamics/FrHydroDB.h
        src/frydom/hydrodynamics/FrHydroDB.cpp
        )

set(SOURCE_IO
        src/frydom/IO/FrLoader.h
        src/frydom/IO/FrLoader.cpp

        src/frydom/IO/FrHDF5.h
        src/frydom/IO/FrHDF5.cpp
        )

set(SOURCE_MISC
        src/frydom/misc/FrUtils.h
        src/frydom/misc/FrUtils.cpp

        src/frydom/misc/FrMathUtils.h

        src/frydom/misc/FrInterp1d.h
        src/frydom/misc/FrInterp1d.cpp

        src/frydom/misc/FrLookupTable1D.h

        src/frydom/misc/FrLinspace.h

        src/frydom/misc/FrTriangleMeshConnected.h
        src/frydom/misc/FrTriangleMeshConnected.cpp

        src/frydom/misc/FrRecorder.h
        src/frydom/misc/FrRecorder.cpp
        )

set(SOURCE_PROPELLER
        src/frydom/propeller/FrPropeller.h
        src/frydom/propeller/FrPropeller.cpp

        src/frydom/propeller/FrPropellerAzimuthal.h
        src/frydom/propeller/FrPropellerAzimuthal.cpp
        )

set(SOURCE_UTILS
        src/frydom/utils/FrIrrApp.h
        src/frydom/utils/FrIrrApp.cpp
        )

set(SOURCE_FILES
        ${SOURCE_CORE}
        ${SOURCE_ENVIRONMENT}
        ${SOURCE_HYDRODYNAMICS}
        ${SOURCE_IO}
        ${SOURCE_MISC}
        ${SOURCE_PROPELLER}
        ${SOURCE_UTILS}
        )



# TODO: mettre dans un foreach !!
# To add a target
add_executable(demo_free_surface ${SOURCE_FILES} src/frydom/tests/demo_FreeSurface.cpp)
target_link_libraries(demo_free_surface ${LIBS})

add_executable(demo_O3 ${SOURCE_FILES} src/frydom/tests/demo_O3_juin_2017.cpp)
target_link_libraries(demo_O3 ${LIBS})

add_executable(demo_rotation ${SOURCE_FILES} src/frydom/tests/demo_rotation.cpp)
target_link_libraries(demo_rotation ${LIBS})

add_executable(demo_ship_orientation ${SOURCE_FILES} src/frydom/tests/demo_ship_orientation.cpp)
target_link_libraries(demo_ship_orientation ${LIBS})

add_executable(test_interp1d ${SOURCE_FILES} src/frydom/tests/test_interp1d.cpp)
target_link_libraries(test_interp1d ${LIBS})

add_executable(test_linspace ${SOURCE_FILES} src/frydom/tests/test_linspace.cpp)
target_link_libraries(test_linspace ${LIBS})

add_executable(test_current_force ${SOURCE_FILES} src/frydom/tests/test_current_force.cpp)
target_link_libraries(test_current_force ${LIBS})

add_executable(test_lookup_table ${SOURCE_FILES} src/frydom/tests/test_lookup_table.cpp)
target_link_libraries(test_lookup_table ${LIBS})

add_executable(test_current_polar_coeffs ${SOURCE_FILES} src/frydom/tests/test_CurrentPolarCoeffs.cpp)
target_link_libraries(test_current_polar_coeffs ${LIBS})

add_executable(test_yaml src/frydom/tests/test_cpp_yaml.cpp)
target_link_libraries(test_yaml ${LIBS})

add_executable(test_catenary ${SOURCE_FILES} src/frydom/tests/test_catenary.cpp)
target_link_libraries(test_catenary ${LIBS})

add_executable(test_tug ${SOURCE_FILES} src/frydom/tests/test_tug.cpp)
target_link_libraries(test_tug ${LIBS})

add_executable(test_eigen ${EIGEN_RELATED} src/frydom/tests/test_eigen.cpp)
target_link_libraries(test_eigen ${LIBS})

add_executable(test_anchored_boat_current ${SOURCE_FILES} src/frydom/tests/test_anchored_boat_current.cpp)
target_link_libraries(test_anchored_boat_current ${LIBS})

add_executable(test_dispersion_relation ${SOURCE_FILES} src/frydom/tests/test_dispersion_relation.cpp)
target_link_libraries(test_dispersion_relation ${LIBS})

add_executable(test_wave_spectrum ${SOURCE_FILES} src/frydom/tests/test_wave_spectrum.cpp)
target_link_libraries(test_wave_spectrum ${LIBS})

add_executable(test_wave_field ${SOURCE_FILES} src/frydom/tests/test_wave_field.cpp)
target_link_libraries(test_wave_field ${LIBS})

add_executable(test_dynamic_cable ${SOURCE_FILES} src/frydom/tests/test_dynamic_cable.cpp)
target_link_libraries(test_dynamic_cable ${LIBS})

add_executable(test_anchored_boat_current_fea_cable ${SOURCE_FILES} src/frydom/tests/test_anchored_boat_current_fea_cable.cpp)
target_link_libraries(test_anchored_boat_current_fea_cable ${LIBS})

add_executable(test_fea_cable_with_body ${SOURCE_FILES} src/frydom/tests/test_fea_cable_with_body.cpp)
target_link_libraries(test_fea_cable_with_body ${LIBS})

add_executable(test_recorder ${SOURCE_FILES} src/frydom/tests/test_recorder.cpp)
target_link_libraries(test_recorder ${LIBS})

add_executable(test_radiation_force ${SOURCE_FILES} src/frydom/tests/test_radiation_force.cpp)
target_link_libraries(test_radiation_force ${LIBS})

add_executable(test_HDF5 ${SOURCE_FILES} src/frydom/tests/test_HDF5.cpp)
target_link_libraries(test_HDF5 ${LIBS})


add_executable(test_load_HDB ${SOURCE_FILES} src/frydom/tests/test_load_HDB.cpp)
target_link_libraries(test_load_HDB ${LIBS})

add_executable(test_wave_excitation_force ${SOURCE_FILES} src/frydom/tests/test_wave_excitation_force.cpp)
target_link_libraries(test_wave_excitation_force ${LIBS})

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
