
message(STATUS "    ...DEMOS :")

set(DEMOS
        Ballistic
        Environment
        FreeSurface
        Body
        Rotation_and_Frame
        Hydrodynamic
        Morison
        Cable
        Hub_Installation
        Langlee
        FOSWEC
        MooringLines
        NonLinearHydrostatics
        Constraints
        Links
        HexagonalArticulatedBuoy
        )

set(BENCH
        SphereDecay
        SphereRegular
        SphereIrregular
        DTMB5512_captive
        Cables
        )

include(URL_DEMO_DATA.conf)

foreach(test ${DEMOS})
    message(STATUS "        ...add demo_${test}")
    add_executable(demo_${test} demo_${test}.cpp)
    target_link_libraries(demo_${test} FRyDoM_Engine)
    target_compile_definitions(demo_${test} PUBLIC RESOURCES_PATH="${${test}_resources_path}")
endforeach()

foreach(test ${BENCH})
    message(STATUS "        ...add bench_${test}")
    add_executable(bench_${test} bench_${test}.cpp)
    target_link_libraries(bench_${test} FRyDoM_Engine)
    target_compile_definitions(bench_${test} PUBLIC RESOURCES_PATH="${test}_${resources_path}")
endforeach()

set(subdirs
        # add here the test source subdirectories
        )

foreach (subdir ${subdirs})
    add_subdirectory(${subdir})
endforeach ()


configure_file(../data/ship/Ship.obj . COPYONLY)

configure_file(../data/platform/Platform_GVA7500.obj . COPYONLY)
configure_file(../data/platform/Platform_HDB.hdb5 . COPYONLY)
configure_file(../data/platform/Platform_PolarCurrentCoeffs_NC.json . COPYONLY)
configure_file(../data/platform/Platform_PolarWindCoeffs_NC.json . COPYONLY)
configure_file(../data/platform/Platform_WaveDriftCoeff.h5 . COPYONLY)
configure_file(../data/platform/mesh_Platform_GVA7500.nemoh . COPYONLY)
configure_file(../data/platform/mesh_Platform_GVA7500.obj . COPYONLY)
configure_file(../data/platform/mesh_Platform_GVA7500_Sym.obj . COPYONLY)

configure_file(../data/bench/sphere/sphere_hdb.h5 . COPYONLY)
configure_file(../data/bench/sphere/bench_sphere_regular.h5 . COPYONLY)
configure_file(../data/bench/sphere/sphere_7814_faces.obj . COPYONLY)
configure_file(../data/bench/sphere/Sphere_6200_faces.obj . COPYONLY)
configure_file(../data/bench/sphere/Sphere_10000_faces.obj . COPYONLY)
configure_file(../data/bench/sphere/Sphere_14350_faces.obj . COPYONLY)

configure_file(../data/bench/DTMB5512/DTMB5512.obj . COPYONLY)
configure_file(../data/bench/DTMB5512/DTMB5512.hdb5 . COPYONLY)

configure_file(../data/barge/Barge_HDB.h5 . COPYONLY)
configure_file(../data/barge/barge.obj . COPYONLY)

configure_file(../data/platform/Platform_HDB_Without_drift.hdb5 . COPYONLY)

configure_file(../data/Mesh_clipper/MagneViking.obj . COPYONLY)
configure_file(../data/Mesh_clipper/MagneViking_clip.obj . COPYONLY)
configure_file(../data/Mesh_clipper/searev.obj . COPYONLY)
configure_file(../data/Mesh_clipper/searev_clip.obj . COPYONLY)
configure_file(../data/Mesh_clipper/BARGE.obj . COPYONLY)

configure_file(../data/Cylinder/Free_cylinder_2900_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_3800_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_5100_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_6000_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_8000_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_9700_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_10500_panels.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_11600_panels.obj . COPYONLY)

configure_file(../data/FPSO/fpso_YC_modif_Coarse.obj . COPYONLY)
configure_file(../data/FPSO/fpso_YC_modif_Medium.obj . COPYONLY)
configure_file(../data/FPSO/fpso_YC_modif_Fine.obj . COPYONLY)

configure_file(../data/FOSWEC/FOSWEC.hdb5 . COPYONLY)
configure_file(../data/FOSWEC/FullFlap_mesh.obj . COPYONLY)
configure_file(../data/FOSWEC/FullPlatform.obj . COPYONLY)
configure_file(../data/Cylinder/Free_cylinder_2900_panels.obj . COPYONLY)

configure_file(../data/FPSO/fpso_YC_modif_Coarse.obj . COPYONLY)
configure_file(../data/FPSO/fpso_YC_modif_Medium.obj . COPYONLY)
configure_file(../data/FPSO/fpso_YC_modif_Fine.obj . COPYONLY)

