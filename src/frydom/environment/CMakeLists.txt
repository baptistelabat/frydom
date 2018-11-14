message(STATUS "    ...Environment")

add_subdirectory(time)
add_subdirectory(field)
add_subdirectory(current)
add_subdirectory(seabed)
add_subdirectory(tidal)
add_subdirectory(waves)
add_subdirectory(wind)


set(SRC_FILES
        FrEnvironment.cpp
        FrFlowBase.cpp
        )

set(FRYDOM_ENVIRONMENT_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${TIME_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${FIELD_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${CURRENT_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${SEABED_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${TIDAL_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${WAVES_FILES})
list(APPEND FRYDOM_ENVIRONMENT_SOURCES ${WIND_FILES})


set(FRYDOM_ENVIRONMENT_SOURCES ${FRYDOM_ENVIRONMENT_SOURCES} PARENT_SCOPE)
