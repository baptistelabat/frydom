message(STATUS "            ...FreeSurface")

add_subdirectory(tidal)
add_subdirectory(waves)

set(SRC_FILES
        FrFreeSurface.cpp
        FrFreeSurfacePhysicItem.cpp
        )

set(FREESURFACE_FILES "")
foreach(file ${SRC_FILES})
    list(APPEND FREESURFACE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND FREESURFACE_FILES ${TIDAL_FILES})
list(APPEND FREESURFACE_FILES ${WAVES_FILES})


set(FREESURFACE_FILES ${FREESURFACE_FILES} PARENT_SCOPE)
