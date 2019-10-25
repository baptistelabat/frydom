
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_FrLinks
        test_cylinder_interaction
        test_FrFunction
        test_logging
        test_FrFileSystem
        )

#include(Add_matplotlib-cpp)

set(test_cylinder_interaction_resources_path ${CMAKE_SOURCE_DIR}/tests/data/CylinderInteraction/)

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

