message(STATUS "    ...utils")

set(SRC_FILES
        FrIrrApp.cpp
        FrIrrCamera.cpp
        FrFileSystem.cpp
)

set(FRYDOM_UTILS_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_UTILS_SOURCES ${FRYDOM_UTILS_SOURCES} PARENT_SCOPE)
