message(STATUS "        ...Atmosphere")

add_subdirectory(wind)

set(SRC_FILES
        FrAtmosphere_.cpp
        )

set(ATMOSPHERE_FILES "")
foreach(file ${SRC_FILES})
    list(APPEND ATMOSPHERE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND ATMOSPHERE_FILES ${WIND_FILES})


set(ATMOSPHERE_FILES ${ATMOSPHERE_FILES} PARENT_SCOPE)
