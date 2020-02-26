message(STATUS "    ...Cable")

set(SRC_FILES
        FrCable.cpp
        FrCatenaryLine.cpp
        FrMooringBuoy.cpp
        FrDynamicCable.cpp
        FrLumpedMassCable.cpp
        FrCableShapeInitializer.cpp
        FrAnchor.cpp
        FrClumpedMass.cpp
)

set(FRYDOM_CABLE_SOURCES "")
foreach (file ${SRC_FILES})
    list(APPEND FRYDOM_CABLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach ()

#message(${FRYDOM_CABLE_SOURCES})

set(FRYDOM_CABLE_SOURCES ${FRYDOM_CABLE_SOURCES} PARENT_SCOPE)
