
message(STATUS "    ...UNIT TESTS :")

set(UNIT_TESTS
        # include test source files here (without .cpp)
#        test_FrCurrentForce_
#        test_FrWindForce_
        test_FrForce_
        test_internal
        test_FrBody_
        test_FrUniformField
        test_FrFlowBase
        test_FrFlowForce
        test_FrFrame_
        test_FrNode_
        test_FrRotation_
        test_FrGeographicServices
        test_FrEquilibriumFrame_
        test_FrWaveField_
        test_FrMorisonElement_
#        test_FrLinearHydrostaticForce_
        test_FrRecorder_
        test_FrStandardWindForce
        test_FrStandardCurrentForce
        test_FrInertia
        test_FrManoeuvringDamping
        test_FrLinearDamping_
        test_FrITTCResistance
        test_FrKinematicStretching
        test_FrWaveSpectrum
        test_FrQuadraticDamping
        test_FrLinks
        test_FrFunction
        )

#enable_testing()
#include(Add_googletest)il


foreach(test ${UNIT_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp gtest_main)
    add_test(NAME ${test}
            COMMAND ${test})
endforeach()


set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()

configure_file(../data/TNR_database.h5 . COPYONLY)

