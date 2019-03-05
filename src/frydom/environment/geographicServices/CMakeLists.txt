message(STATUS "        ...GeographicServices")

set(SRC_FILES
        FrGeographicServices.cpp
        )

set(GEOGRAPHIC_SERVICES_FILES "")
foreach(file ${SRC_FILES})
    list(APPEND GEOGRAPHIC_SERVICES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(GEOGRAPHIC_SERVICES_FILES ${GEOGRAPHIC_SERVICES_FILES} PARENT_SCOPE)
