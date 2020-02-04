message(STATUS "    ...collision")

set(SRC_FILES
        FrCollisionModel.cpp
        )

set(FRYDOM_COLLISION_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_COLLISION_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

set(FRYDOM_COLLISION_SOURCES ${FRYDOM_COLLISION_SOURCES} PARENT_SCOPE)
