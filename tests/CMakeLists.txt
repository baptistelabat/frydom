message(STATUS "    ...tests")

set(FRYDOM_TESTS
        bench_SphereDecay
        bench_SphereRegular
        demo_FreeSurface
        demo_O3_juin_2017
        demo_rotation
        demo_ship_orientation
        test_anchored_boat_current
        test_anchored_boat_current_fea_cable
        test_anchored_platform_waves
        test_catenary
        test_cpp_yaml
        test_current_force
        test_CurrentPolarCoeffs
        test_dispersion_relation
        test_dynamic_cable
        test_fea_cable_with_body
        test_IRF_computation
        test_load_HDB
        test_radiation_force
        test_recorder
        test_tug
        test_wave_drift_HDF5_loader
        test_wave_excitation_force
        test_wave_field
        test_wave_spectrum
        test_geographicLib
        test_timezonelib
        test_sphere_irregular
        test_MorisonModel
        test_CompositeMorisonModel
        test_dynamic_wave_probe
        test_waveProbe
        test_IRF_ForwardSpeed
        test_unrolling_cable
        test_unrolling_catenary_line
        )

set(CMAKE_CXX_FLAGS " -lIrrlicht")

include(Add_matplotlib-cpp)


message(STATUS "Test programs for FRyDoM framework...")

foreach(test ${FRYDOM_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()

# Adding resource files
configure_file(data/frydom_logo.png . COPYONLY)
configure_file(../data/skybox/sky_dn.jpg . COPYONLY)
configure_file(../data/skybox/sky_lf.jpg . COPYONLY)
configure_file(../data/skybox/sky_up.jpg . COPYONLY)

configure_file(data/frydom_hdb.h5 . COPYONLY)
configure_file(data/Cylinder.obj . COPYONLY)
configure_file(../data/ship/MagneViking.obj . COPYONLY)
configure_file(data/PolarCurrentCoeffs.yml . COPYONLY)

file(COPY data/Full.yml  DESTINATION .)

configure_file(data/DeepSea_Stavanger/DeepSea_WaveDriftCoeff.h5 . COPYONLY)
configure_file(data/DeepSea_Stavanger/GVA7500.obj . COPYONLY)
configure_file(data/DeepSea_Stavanger/DeepSeaStavanger.hdb5 . COPYONLY)
configure_file(data/DeepSea_Stavanger/PolarCurrentCoeffs_NC.yml . COPYONLY)
configure_file(data/DeepSea_Stavanger/PolarWindCoeffs_NC.yml . COPYONLY)
configure_file(data/DeepSea_Stavanger/DeepSea_WaveDriftCoeff.h5 . COPYONLY)

configure_file(data/bench_sphere_decay_interp.h5 . COPYONLY)
configure_file(data/bench_sphere_regular.h5 . COPYONLY)
configure_file(data/sphere.obj . COPYONLY)
configure_file(data/sphere_hdb.h5 . COPYONLY)

configure_file(data/Ship/Ship.h5 . COPYONLY)
configure_file(data/Ship/Ship.obj . COPYONLY)
configure_file(data/Ship/Ship_PolarCurrentCoeffs.yml . COPYONLY)
configure_file(data/Ship/Ship_PolarWindCoeffs.yml . COPYONLY)
configure_file(data/Ship/Ship_WaveDriftCoeff.h5 . COPYONLY)
