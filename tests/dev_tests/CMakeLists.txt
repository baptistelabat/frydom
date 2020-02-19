
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_FrLinks
        test_cylinder_interaction
        test_FrFunction
        test_FrFileSystem
        test_spdlog
        test_lumped_mass_cable
        test_chrono_spring_bodies
        test_LM_element
        )

foreach (test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)

endforeach ()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()

