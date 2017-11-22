message(STATUS "        ...Environment/wind")

set(WIND_FILES_LOCAL
        FrWind.h
        FrWind.cpp
        FrWindForce.h
        FrWindForce.cpp
        )

set(WIND_FILES "")
foreach(file ${WIND_FILES_LOCAL})
    list(APPEND WIND_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(WIND_FILES ${WIND_FILES} PARENT_SCOPE)
