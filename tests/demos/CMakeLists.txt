
message(STATUS "    ...DEMOS :")



set(DEMOS
        demo_Ballistic
        demo_Environment
        demo_FreeSurface
        demo_Body
        demo_Rotation_and_Frame
        demo_Hydrodynamic
        demo_Morison
        demo_Cable
        demo_Hub_installation
        bench_SphereDecay
        bench_SphereRegular
        bench_DTMB5512_captive
        )


foreach(test ${DEMOS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} FRyDoM_Engine matplotlib-cpp)

endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()


# Adding resource files
configure_file(../data/frydom_logo.png . COPYONLY)
configure_file(../../data/skybox/sky_dn.jpg . COPYONLY)
configure_file(../../data/skybox/sky_lf.jpg . COPYONLY)
configure_file(../../data/skybox/sky_up.jpg . COPYONLY)

configure_file(../data/DTMB5512/DTMB5512.obj . COPYONLY)
configure_file(../data/Ship/Ship.obj . COPYONLY)
configure_file(../data/DeepSea_Stavanger/GVA7500.obj . COPYONLY)
configure_file(../data/DeepSea_Stavanger/DeepSeaStavanger.hdb5 . COPYONLY)
configure_file(../data/DeepSea_Stavanger/PolarCurrentCoeffs_NC.yml . COPYONLY)
configure_file(../data/DeepSea_Stavanger/PolarWindCoeffs_NC.yml . COPYONLY)
configure_file(../data/DeepSea_Stavanger/DeepSea_WaveDriftCoeff.h5 . COPYONLY)

configure_file(../data/sphere_hdb.h5 . COPYONLY)
configure_file(../data/bench_sphere_regular.h5 . COPYONLY)
configure_file(../data/DTMB5512/DTMB5512.hdb5 . COPYONLY)
configure_file(../data/Barge/Barge_frydom.h5 . COPYONLY)
