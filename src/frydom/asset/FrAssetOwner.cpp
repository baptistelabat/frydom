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
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/shape/FrBoxShape.h"

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
        auto shape = std::make_shared<chrono::ChCylinderShape>();
        shape->GetCylinderGeometry().p1 = chrono::ChVector<double>(0., -height*0.5, 0.);
        shape->GetCylinderGeometry().p2 = chrono::ChVector<double>(0.,  height*0.5, 0.);
        shape->GetCylinderGeometry().rad = radius;
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddSphereShape(double radius) {
        auto shape = std::make_shared<chrono::ChSphereShape>();
        shape->GetSphereGeometry().rad = radius;
        GetChronoItem_ptr()->AddAsset(shape);
    }


    void FrAssetOwner::AddMeshAsset(std::string obj_filename) {
        auto mesh = std::make_shared<FrTriangleMeshConnected>();
        mesh->LoadWavefrontMesh(obj_filename);
        AddMeshAsset(mesh);
    }

    void FrAssetOwner::AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh) {
        auto shape = std::make_shared<chrono::ChTriangleMeshShape>();
        shape->SetMesh(mesh);
        GetChronoItem_ptr()->AddAsset(shape);
    }

    void FrAssetOwner::AddAsset(std::shared_ptr<FrAsset> asset) {
        m_assets.push_back(asset);
        GetChronoItem_ptr()->AddAsset(asset->GetChronoAsset());
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
