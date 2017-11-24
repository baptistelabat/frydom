message(STATUS "    ...tests")

set(FRYDOM_TESTS
        demo_FreeSurface
        demo_O3_juin_2017
        demo_rotation
        demo_ship_orientation
        test_anchored_boat_current
        test_anchored_boat_current_fea_cable
        test_catenary
        test_ch_system
        test_cpp_yaml
        test_current_force
        test_CurrentPolarCoeffs
        test_dispersion_relation
        test_dynamic_cable
        test_eigen
        test_fea_cable_with_body
        test_HDF5
        test_interp1d
        test_linspace
        test_load_HDB
        test_lookup_table
        test_quadprogcpp
        test_radiation_force
        test_recorder
        test_tug
        test_wave_excitation_force
        test_wave_field
        test_wave_spectrum
        )


message(STATUS "Test programs for FRyDoM framework...")

foreach(test ${FRYDOM_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine)

endforeach()

# Adding resource filesq
configure_file(data/frydom_logo.png . COPYONLY)
configure_file(../../../data/skybox/sky_dn.jpg . COPYONLY)
configure_file(../../../data/skybox/sky_lf.jpg . COPYONLY)
configure_file(../../../data/skybox/sky_up.jpg . COPYONLY)