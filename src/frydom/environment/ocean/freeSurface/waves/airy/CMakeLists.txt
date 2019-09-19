message(STATUS "                    ...airy")

set(AIRY_FILES_LOCAL
        FrAiryRegularWaveField.cpp
        FrAiryRegularOptimWaveField.cpp
        FrAiryIrregularWaveField.cpp
<<<<<<< Updated upstream
        FrAiryIrregularOptimWaveField.cpp
=======
#        FrAiryIrregularOptimWaveField.cpp
        FrKinematicStretching.cpp
>>>>>>> Stashed changes
        )

set(AIRY_FILES "")
foreach(file ${AIRY_FILES_LOCAL})
    list(APPEND AIRY_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(AIRY_FILES ${AIRY_FILES} PARENT_SCOPE)
