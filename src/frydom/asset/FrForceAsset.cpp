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

    namespace internal {

        FrForceAssetBase_::FrForceAssetBase_(frydom::FrForceAsset_ *forceAsset) : m_frydomForceAsset(
                forceAsset) {
            SetDrawMode(eCh_GlyphType::GLYPH_VECTOR);
        }

        void FrForceAssetBase_::Update(chrono::ChPhysicsItem *updater, const chrono::ChCoordsys<> &coords) {
            m_frydomForceAsset->Update();
        }


    } // end namespace internal






    FrForceAsset_::FrForceAsset_(FrForce_* force){
        m_force = force;
        // TODO: ajouter couleur
        m_chronoAsset = std::make_shared<internal::FrForceAssetBase_>(this);

        m_CharacteristicLength = 0.005;
        m_symbolscolor = chrono::ChColor(0, 0.5, 0.5, 0);

        auto point = internal::Vector3dToChVector(force->GetForceApplicationPointInBody(NWU));
        auto forcevect = internal::Vector3dToChVector(force->GetForceInBody(NWU)) * m_CharacteristicLength;
        m_chronoAsset->SetGlyphVector(0, point, forcevect, m_symbolscolor);
        m_chronoAsset->SetGlyphsSize(20);  // Ne semble pas avoir d'effet dans Irrlicht

    }

    void FrForceAsset_::Update() {

        // Here, the asset point is automatically following the motion but the force has to be updated
        auto point = internal::Vector3dToChVector(m_force->GetForceApplicationPointInWorld(NWU));
        auto forcevect = internal::Vector3dToChVector(m_force->GetForceInWorld(NWU)) * m_CharacteristicLength;

        m_chronoAsset->SetGlyphVector(0, point, forcevect, m_symbolscolor);

    }

    std::shared_ptr<chrono::ChAsset> FrForceAsset_::GetChronoAsset() {
        return m_chronoAsset;
    }
} // end namespace frydom