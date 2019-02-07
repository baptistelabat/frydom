message(STATUS "    ...tests")

message(STATUS "Test programs for FRyDoM framework...")

add_subdirectory(dev_tests)
add_subdirectory(unit_tests)
add_subdirectory(demos)

## Adding resource files
#configure_file(data/frydom_logo.png . COPYONLY)
#configure_file(../data/skybox/sky_dn.jpg . COPYONLY)
#configure_file(../data/skybox/sky_lf.jpg . COPYONLY)
#configure_file(../data/skybox/sky_up.jpg . COPYONLY)
#
#configure_file(data/frydom_hdb.h5 . COPYONLY)
#configure_file(data/Cylinder.obj . COPYONLY)
#configure_file(../data/ship/MagneViking.obj . COPYONLY)
#configure_file(data/Ship/MagneViking_scaled.obj . COPYONLY)
#configure_file(data/PolarCurrentCoeffs.yml . COPYONLY)
#
#file(COPY data/Full.yml  DESTINATION .)
#
#configure_file(data/DeepSea_Stavanger/DeepSea_WaveDriftCoeff.h5 . COPYONLY)
#configure_file(data/DeepSea_Stavanger/GVA7500.obj . COPYONLY)
#configure_file(data/DeepSea_Stavanger/DeepSeaStavanger.hdb5 . COPYONLY)
#configure_file(data/DeepSea_Stavanger/PolarCurrentCoeffs_NC.yml . COPYONLY)
#configure_file(data/DeepSea_Stavanger/PolarWindCoeffs_NC.yml . COPYONLY)
#configure_file(data/DeepSea_Stavanger/DeepSea_WaveDriftCoeff.h5 . COPYONLY)
#
#configure_file(data/bench_sphere_decay_interp.h5 . COPYONLY)
#configure_file(data/bench_sphere_regular.h5 . COPYONLY)
#configure_file(data/sphere.obj . COPYONLY)
#configure_file(data/sphere_hdb.h5 . COPYONLY)
#
#configure_file(data/Ship/Ship.h5 . COPYONLY)
#configure_file(data/Ship/Ship.obj . COPYONLY)
#configure_file(data/Ship/Ship_PolarCurrentCoeffs.yml . COPYONLY)
#configure_file(data/Ship/Ship_PolarWindCoeffs.yml . COPYONLY)
#configure_file(data/Ship/Ship_WaveDriftCoeff.h5 . COPYONLY)
#
#configure_file(data/DTMB5512/DTMB5512.obj . COPYONLY)
#configure_file(data/DTMB5512/DTMB5512_hdb.h5 . COPYONLY)
#configure_file(data/DTMB5512/DTMB5512_WaveDriftCoeff.h5 . COPYONLY)


#add_subdirectory(dev_tests)