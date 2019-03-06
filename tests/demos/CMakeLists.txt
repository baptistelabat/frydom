
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
	demo_WNL_and_NL_hydrostatics
        bench_SphereDecay
        bench_SphereRegular
        bench_DTMB5512_captive
        )


foreach(test ${DEMOS})
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

configure_file(../data/ship/Ship.obj . COPYONLY)

configure_file(../data/platform/Platform_GVA7500.obj . COPYONLY)
configure_file(../data/platform/Platform_HDB.hdb5 . COPYONLY)
configure_file(../data/platform/Platform_PolarCurrentCoeffs_NC.yml . COPYONLY)
configure_file(../data/platform/Platform_PolarWindCoeffs_NC.yml . COPYONLY)
configure_file(../data/platform/Platform_WaveDriftCoeff.h5 . COPYONLY)
configure_file(../data/platform/mesh_Platform_GVA7500.nemoh . COPYONLY)
configure_file(../data/platform/mesh_Platform_GVA7500.obj . COPYONLY)

configure_file(../data/bench/sphere/sphere_hdb.h5 . COPYONLY)
configure_file(../data/bench/sphere/bench_sphere_regular.h5 . COPYONLY)

configure_file(../data/bench/DTMB5512/DTMB5512.obj . COPYONLY)
configure_file(../data/bench/DTMB5512/DTMB5512.hdb5 . COPYONLY)

configure_file(../data/barge/Barge_HDB.h5 . COPYONLY)

configure_file(../data/platform/Platform_HDB_Without_drift.hdb5 . COPYONLY)

configure_file(../data/Mesh_clipper/MagneViking.obj . COPYONLY)
configure_file(../data/Mesh_clipper/MagneViking_clip.obj . COPYONLY)
configure_file(../data/Mesh_clipper/searev.obj . COPYONLY)
configure_file(../data/Mesh_clipper/searev_clip.obj . COPYONLY)
configure_file(../data/Mesh_clipper/BARGE.obj . COPYONLY)