message(STATUS "                    ...airy")

set(AIRY_FILES_LOCAL
        FrAiryRegularWaveField.cpp
        )

set(AIRY_FILES "")
foreach(file ${AIRY_FILES_LOCAL})
    list(APPEND AIRY_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(AIRY_FILES ${AIRY_FILES} PARENT_SCOPE)
