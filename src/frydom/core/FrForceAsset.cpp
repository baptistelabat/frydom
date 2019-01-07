//
// Created by frongere on 14/06/17.
//

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






    FrForceAsset_::FrForceAsset_(std::shared_ptr<FrForce_> force){
        m_force = force;
        // TODO: ajouter couleur
        m_chronoAsset = std::make_shared<internal::FrForceAssetBase_>(this);

        auto point = internal::Vector3dToChVector(force->GetForceApplicationPointInBody(NWU));
        auto forcevect = internal::Vector3dToChVector(force->GetForceInBody(NWU));
        m_chronoAsset->SetGlyphVector(0, point, forcevect);
        m_chronoAsset->SetGlyphsSize(20);  // Ne semble pas avoir d'effet dans Irrlicht

    }

    void FrForceAsset_::Update() {

        // Here, the asset point is automatically following the motion but the force has to be updated
        m_chronoAsset->SetGlyphVector(0, internal::Vector3dToChVector(m_force->GetForceApplicationPointInWorld(NWU)),
                                      internal::Vector3dToChVector(m_force->GetForceInWorld(NWU)));

    }

    std::shared_ptr<chrono::ChAsset> FrForceAsset_::GetChronoAsset() {
        return m_chronoAsset;
    }
} // end namespace frydom