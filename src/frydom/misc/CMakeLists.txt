message(STATUS "    ...misc")

set(MISC_FILES_LOCAL
        FrEigen.h
        FrEigen.cpp
        FrInterp1d.h
        FrInterp1d.cpp
        FrLinspace.h
        FrLookupTable1D.h
        FrMathUtils.h
        FrRecorder.h
        FrRecorder.cpp
        FrTriangleMeshConnected.h
        FrTriangleMeshConnected.cpp
        FrUtils.h
        FrUtils.cpp
)

set(MISC_FILES "")
foreach(file ${MISC_FILES_LOCAL})
    list(APPEND MISC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MISC_FILES ${MISC_FILES} PARENT_SCOPE)
