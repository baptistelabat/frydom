message(STATUS "        ...Environment/current")

set(CURRENT_FILES_LOCAL
        FrCurrent.h
        FrCurrent.cpp
        FrCurrentForce.h
        FrCurrentForce.cpp
        FrCurrentPolarCoeffs.h
        FrCurrentPolarCoeffs.cpp
)

set(CURRENT_FILES "")
foreach(file ${CURRENT_FILES_LOCAL})
    list(APPEND CURRENT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(CURRENT_FILES ${CURRENT_FILES} PARENT_SCOPE)