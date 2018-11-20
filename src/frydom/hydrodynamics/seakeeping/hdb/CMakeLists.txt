message(STATUS "            ...hydrodynamics database")

set(SRC_FILES
        FrBEMBody.cpp
        FrHydroDB.cpp
        FrHydroDBLoader.cpp
        FrHydroMapper.cpp
        )

set(FRYDOM_HYDRODYNAMICS_SEAKEEPING_HDB_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_HYDRODYNAMICS_SEAKEEPING_HDB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_HYDRODYNAMICS_SEAKEEPING_HDB_SOURCES ${FRYDOM_HYDRODYNAMICS_SEAKEEPING_HDB_SOURCES} PARENT_SCOPE)
