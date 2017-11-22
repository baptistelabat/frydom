message(STATUS "    ...propeller")

set(PROPELLER_FILES_LOCAL
        FrDieselEngine.h
        FrDieselEngine.cpp
        FrElectricEngine.h
        FrElectricEngine.cpp
        FrKTKQPropellerTable.h
        FrKTKQPropellerTable.cpp
        FrMarineEngine.h
        FrMarineEngine.cpp
        FrPropeller.h
        FrPropeller.cpp
        FrPropellerAzimuthal.h
        FrPropellerAzimuthal.cpp
        FrPropulsionSystem.h
        FrPropulsionSystem.cpp
)

set(PROPELLER_FILES "")
foreach(file ${PROPELLER_FILES_LOCAL})
    list(APPEND PROPELLER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(PROPELLER_FILES ${PROPELLER_FILES} PARENT_SCOPE)
