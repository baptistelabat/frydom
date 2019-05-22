message(STATUS "    ...Asset")

add_subdirectory(shape)

set(SRC_FILES
        FrAsset.cpp
        FrAssetOwner.cpp
        FrNodeAsset.cpp
        FrForceAsset.cpp
        FrGridAsset.cpp
        FrSeabedGridAsset.cpp
        FrFreeSurfaceGridAsset.cpp
        FrCatenaryLineAsset.cpp
        )

set(FRYDOM_ASSET_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_ASSET_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

list(APPEND FRYDOM_ASSET_SOURCES ${FRYDOM_SHAPE_SOURCES})

set(FRYDOM_ASSET_SOURCES ${FRYDOM_ASSET_SOURCES} PARENT_SCOPE)
