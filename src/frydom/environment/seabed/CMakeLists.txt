message(STATUS "        ...Environment/seabed")

set(SEABED_FILES_LOCAL
#        FrSeabed.h
        FrSeabed.cpp
)

set(SEABED_FILES "")
foreach(file ${SEABED_FILES_LOCAL})
    list(APPEND SEABED_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(SEABED_FILES ${SEABED_FILES} PARENT_SCOPE)
