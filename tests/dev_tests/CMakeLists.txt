
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_body_mapper
        test_hydrostatic_new_hdb
        test_radiation_model
        test_FrLinks
        test_cylinder_interaction
        bench_Ellipsoid_OMAE2014
        test_WNL_and_NL_hydrostatics
        test_FrFunction
        dev_Carriage
        )

#include(Add_matplotlib-cpp)

set(test_body_mapper_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)
set(test_hydrostatic_new_hdb_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)
set(test_radiation_mode_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)
set(test_WNL_and_NL_hydrostatics_resources_path ${CMAKE_SOURCE_DIR}/tests/data/platform/)
set(test_CylinderInteraction_resources_path ${CMAKE_SOURCE_DIR}/tests/data/CylinderInteraction/)
set(bench_Ellipsoid_OMAE2014_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/Ellipsoid_OMAE2014/)
set(dev_Carriage_resources_path ${CMAKE_SOURCE_DIR}/tests/data/bench/DTMB5512/)

foreach(test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)
    target_compile_definitions(${test} PUBLIC RESOURCES_PATH="${${test}_resources_path}")

endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()

