message(STATUS "    ...shape")

set(SRC_FILES
    FrBoxShape.cpp
    FrCylinderShape.cpp
    FrSphereShape.cpp
)

set(FRYDOM_SHAPE_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_SHAPE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_SHAPE_SOURCES ${FRYDOM_SHAPE_SOURCES} PARENT_SCOPE)
