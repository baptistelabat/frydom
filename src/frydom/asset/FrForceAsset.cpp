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


#include "FrForceAsset.h"

namespace frydom {

//    namespace internal {
//
//        FrForceAssetBase_::FrForceAssetBase_(frydom::FrForceAsset_ *forceAsset) : m_frydomForceAsset(
//                forceAsset) {
//            SetDrawMode(eCh_GlyphType::GLYPH_VECTOR);
//        }
//
//        void FrForceAssetBase_::Update(chrono::ChPhysicsItem *updater, const chrono::ChCoordsys<> &coords) {
//            m_frydomForceAsset->Update();
//        }
//
//
//    } // end namespace internal


    FrForceAsset_::FrForceAsset_(FrForce_* force) : m_force(force), m_CharacteristicLength(0.005), FrAsset(){
        m_symbolscolor = chrono::ChColor(0, 0.5, 0.5, 0);
    }

    void FrForceAsset_::Initialize() {
        auto point = internal::Vector3dToChVector(m_force->GetForceApplicationPointInBody(NWU));
        auto forcevect = internal::Vector3dToChVector(m_force->GetForceInBody(NWU)) * m_CharacteristicLength;

        auto glyphAsset = std::make_shared<chrono::ChGlyphs>();
        glyphAsset->SetDrawMode(chrono::ChGlyphs::eCh_GlyphType::GLYPH_VECTOR);

        glyphAsset->SetGlyphVector(0, point, forcevect, m_symbolscolor);
        glyphAsset->SetGlyphsSize(20);  // Ne semble pas avoir d'effet dans Irrlicht

        m_chronoAsset->AddAsset(glyphAsset);
    }

    void FrForceAsset_::Update() {
        // TODO: voir à passer dans le StepFinalize ?

        // Get the glyph asset form the AssetLevel
        auto GlyphAsset = dynamic_cast<chrono::ChGlyphs*> (m_chronoAsset->GetAssetN(0).get());

        // Here, the asset point is automatically following the motion but the force has to be updated
        auto point = internal::Vector3dToChVector(m_force->GetForceApplicationPointInWorld(NWU));
        auto forcevect = internal::Vector3dToChVector(m_force->GetForceInWorld(NWU)) * m_CharacteristicLength;

        GlyphAsset->SetGlyphVector(0, point, forcevect, m_symbolscolor);

    }

//    std::shared_ptr<chrono::ChAsset> FrForceAsset_::GetChronoAsset() {
//        return m_chronoAsset;
//    }
} // end namespace frydom