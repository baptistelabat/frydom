message(STATUS "        ...Environment/current")

set(CURRENT_FILES_LOCAL
        FrCurrent.cpp
        FrCurrentForce.cpp
        FrCurrentPolarCoeffs.cpp
        FrCurrentStandardForce.cpp
)

set(CURRENT_FILES "")
foreach(file ${CURRENT_FILES_LOCAL})
    list(APPEND CURRENT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(CURRENT_FILES ${CURRENT_FILES} PARENT_SCOPE)