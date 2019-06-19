
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

configure_file(../data/bench/DTMB5512/DTMB5512.obj . COPYONLY)
configure_file(../data/bench/DTMB5512/DTMB5512.hdb5 . COPYONLY)


configure_file(../data/platform/Platform_HDB.hdb5 . COPYONLY)
configure_file(../data/CylinderInteraction/CylinderInteraction.hdb5 . COPYONLY)

configure_file(../data/bench/Ellipsoid_OMAE2014/elipsoid_scaled_2880_faces.obj . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/elipsoid_scaled_mean_position.obj . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_4200_faces.obj . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_2880_faces.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_4200_faces.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_634_faces_with_sym.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_1416_faces_with_sym.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_2774_faces_with_sym.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_2880_faces_with_sym_mean_position.hdb5 . COPYONLY)
configure_file(../data/bench/Ellipsoid_OMAE2014/Ellipsoid_4500_faces_WAMIT_mesh_sym_mean_position.hdb5 . COPYONLY)
