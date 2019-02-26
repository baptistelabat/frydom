
message(STATUS "    ...DEV TESTS :")

set(REFACTORING_TESTS
        test_body_mapper
        test_hydrostatic_new_hdb
        test_radiation_model
        how_to_cppfs
        test_logging
        )

#include(Add_matplotlib-cpp)

foreach(test ${REFACTORING_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)

endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()


# Adding resource files
configure_file(../../data/frydom_logo.png . COPYONLY)
configure_file(../../data/skybox/sky_dn.jpg . COPYONLY)
configure_file(../../data/skybox/sky_lf.jpg . COPYONLY)
configure_file(../../data/skybox/sky_up.jpg . COPYONLY)


configure_file(../data/platform/Platform_HDB.hdb5 . COPYONLY)
