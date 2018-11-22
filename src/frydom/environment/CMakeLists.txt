message(STATUS "    ...Environment")

add_subdirectory(time)
add_subdirectory(field)
add_subdirectory(ocean)
add_subdirectory(wind)
add_subdirectory(flow)


set(SRC_FILES
        FrEnvironment.cpp
        )

set(FRYDOM_ENVIRONMENT_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${TIME_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${FIELD_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${OCEAN_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${WIND_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${FLOW_FILES})


set(FRYDOM_ENVIRONMENT_SOURCES ${FRYDOM_ENVIRONMENT_SOURCES} PARENT_SCOPE)
