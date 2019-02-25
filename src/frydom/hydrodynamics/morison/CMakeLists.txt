message(STATUS "    ...morison")

set(SRC_FILES
        FrMorisonForce.cpp
        FrMorisonModel.cpp
#        FrMorrisonElement.cpp
        FrMorrisonForce.cpp
        )

set(FRYDOM_MORISON_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_MORISON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_MORISON_SOURCES ${FRYDOM_MORISON_SOURCES} PARENT_SCOPE)
