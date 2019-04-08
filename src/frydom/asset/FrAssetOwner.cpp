// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#include <chrono/assets/ChTriangleMeshShape.h>
#include <chrono/assets/ChColorAsset.h>

#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/shape/FrBoxShape.h"
#include "frydom/shape/FrCylinderShape.h"
#include "frydom/shape/FrSphereShape.h"
#include "frydom/shape/FrTriangleMeshShape.h"

#include "FrAssetOwner.h"

#include "FrAsset.h"

namespace frydom{

//    double FrAssetOwner::GetTime() {
//        GetChronoItem_ptr()->GetChTime();
//    }

    void FrAssetOwner::AddBoxShape(double xSize, double ySize, double zSize) {
        auto shape = std::make_shared<FrBoxShape>(xSize, ySize, zSize);
        m_boxShapes.push_back(shape);
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddCylinderShape(double radius, double height) {
        auto shape = std::make_shared<FrCylinderShape>(radius, height);
        m_cylinderShapes.push_back(shape);
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddSphereShape(double radius) {
        auto shape = std::make_shared<FrSphereShape>(radius);
        m_sphereShapes.push_back(shape);
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddMeshAsset(std::string obj_filename) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        AddMeshAsset(mesh);
    }

    void FrAssetOwner::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<FrTriangleMeshShape>(mesh);
        m_meshShapes.push_back(shape);
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddAsset(std::shared_ptr<FrAsset> asset) {
        m_assets.push_back(asset);
        GetChronoItem_ptr()->AddAsset(asset->GetChronoAsset());
    }

    FrAssetOwner::BoxShapeConstContainer FrAssetOwner::GetBoxShapes() const {
        FrAssetOwner::BoxShapeConstContainer result;
        for (const auto& shape : m_boxShapes) {
            result.push_back(std::const_pointer_cast<const FrBoxShape>(shape));
        }
        return result;
    }

    FrAssetOwner::CylinderShapeConstContainer FrAssetOwner::GetCylinderShapes() const {
        FrAssetOwner::CylinderShapeConstContainer result;
        for (const auto& shape : m_cylinderShapes) {
            result.push_back(std::const_pointer_cast<const FrCylinderShape>(shape));
        }
        return result;
    }

    FrAssetOwner::SphereShapeConstContainer FrAssetOwner::GetSphereShapes() const {
        FrAssetOwner::SphereShapeConstContainer result;
        for (const auto& shape : m_sphereShapes) {
            result.push_back(std::const_pointer_cast<const FrSphereShape>(shape));
        }
        return result;
    }

    FrAssetOwner::TriangleMeshShapeConstContainer FrAssetOwner::GetMeshAssets() const {
        FrAssetOwner::TriangleMeshShapeConstContainer result;
        for (const auto& shape : m_meshShapes) {
            result.push_back(std::const_pointer_cast<const FrTriangleMeshShape>(shape));
        }
        return result;
    }

    void FrAssetOwner::SetColor(NAMED_COLOR colorName) {
        SetColor(FrColor(colorName));
    }

    void FrAssetOwner::SetColor(const FrColor& color) {
        auto colorAsset = std::make_shared<chrono::ChColorAsset>(
                chrono::ChColor(color.R, color.G, color.B));
        GetChronoItem_ptr()->AddAsset(colorAsset);
    }

    void FrAssetOwner::UpdateAsset() {
        // StepFinalize of assets
        auto assetIter = asset_begin();
        for (; assetIter != asset_end(); assetIter++) {
            (*assetIter)->StepFinalize();
        }
    }

    // Asset linear iterators
    FrAssetOwner::AssetIter FrAssetOwner::asset_begin() {
        return m_assets.begin();
    }

    FrAssetOwner::ConstAssetIter FrAssetOwner::asset_begin() const {
        return m_assets.cbegin();
    }

    FrAssetOwner::AssetIter FrAssetOwner::asset_end() {
        return m_assets.end();
    }

    FrAssetOwner::ConstAssetIter FrAssetOwner::asset_end() const {
        return m_assets.cend();
    }


}// end namespace frydom
