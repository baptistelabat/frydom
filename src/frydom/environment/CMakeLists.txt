message(STATUS "    ...Environment")

add_subdirectory(time)
add_subdirectory(ocean)
add_subdirectory(atmosphere)
add_subdirectory(flow)
add_subdirectory(geographicServices)


set(SRC_FILES
        FrEnvironment.cpp
        )

set(FRYDOM_ENVIRONMENT_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${TIME_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${OCEAN_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${ATMOSPHERE_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${FLOW_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${GEOGRAPHIC_SERVICES_FILES})


set(FRYDOM_ENVIRONMENT_SOURCES ${FRYDOM_ENVIRONMENT_SOURCES} PARENT_SCOPE)
