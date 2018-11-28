message(STATUS "        ...Ocean")

add_subdirectory(current)
add_subdirectory(seabed)
add_subdirectory(freeSurface)

set(SRC_FILES
        FrOcean_.cpp
        )

set(OCEAN_FILES "")
foreach(file ${SRC_FILES})
    list(APPEND OCEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND OCEAN_FILES ${CURRENT_FILES})
list(APPEND OCEAN_FILES ${SEABED_FILES})
list(APPEND OCEAN_FILES ${FREESURFACE_FILES})


set(OCEAN_FILES ${OCEAN_FILES} PARENT_SCOPE)
