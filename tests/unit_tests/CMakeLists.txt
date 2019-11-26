
message(STATUS "    ...UNIT TESTS :")

set(UNIT_TESTS
        # include test source files here (without .cpp)
        test_FrLoggable
        test_FrPathManager
        test_FrForce
        test_internal
        test_FrBody
        test_FrUniformField
        test_FrFlowBase
        test_FrFlowForce
        test_FrFrame
        test_FrNode
        test_FrRotation
        test_FrGeographicServices
        test_FrEquilibriumFrame
        test_FrWaveField
        test_FrMorisonElement
        test_FrRecorder
        test_FrStandardWindForce
        test_FrStandardCurrentForce
        test_FrInertia
        test_FrManoeuvringDamping
        test_FrLinearDamping
        test_FrITTCResistance
        test_FrKinematicStretching
        test_FrWaveSpectrum
        test_FrQuadraticDamping
        test_FrShape
        test_edge_splitting
        test_FrAssembly
        test_FrClippingSurface
        test_FrPlane
        test_FrPolygon
        test_FrMesh
        test_FrLink
        test_Torsor
        )

# Set the path definition of the data resources
set(unit_test_resources_path ${CMAKE_SOURCE_DIR}/tests/data/unit_test/)

foreach (test ${UNIT_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine gtest_main)
    target_compile_definitions(${test} PUBLIC RESOURCES_PATH="${unit_test_resources_path}")
    add_test(NAME ${test}
            COMMAND ${test})
endforeach ()


set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()
