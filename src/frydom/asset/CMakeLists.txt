message(STATUS "    ...Asset")

set(SRC_FILES
        FrAsset.cpp
        FrAssetOwner.cpp
        FrForceAsset.cpp
        FrAssetClumpWeight.cpp
        FrGridAsset.cpp
        FrSeabedGridAsset.cpp
        FrFreeSurfaceGridAsset.cpp
        FrCatenaryLineAsset.cpp
        FrAssetBuoy.cpp
        FrAssetComponent.cpp
)

set(FRYDOM_ASSET_SOURCES "")
foreach(file ${SRC_FILES})
    list(APPEND FRYDOM_ASSET_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(FRYDOM_ASSET_SOURCES ${FRYDOM_ASSET_SOURCES} PARENT_SCOPE)
