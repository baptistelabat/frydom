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


#ifndef FRYDOM_FRFORCEASSET_H
#define FRYDOM_FRFORCEASSET_H

#include "chrono/assets/ChGlyphs.h"
#include "chrono/physics/ChBody.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/asset/FrAsset.h"


// Cet asset doit etre ajoute directement au corps qui contient la force
// On peut faire de telle maniere que quand on ajoute l'asset a la force, c'est au corps de la force qu'il est ajoute...

namespace frydom {

    /**
     * \class FrForceAsset
     * \brief Class to display the loads.
     */
    class FrForceAsset : public chrono::ChGlyphs {

    private:
        std::shared_ptr<FrForce> m_force;  //< The force that this asset represents
        double OrderOfMagnitude;
        bool adaptive_OOM;

        double CharacteristicLength;
        bool inverse_direction;
        // TODO ajouter flag pour dire si on affiche la force qui s'applique ou la force delivree pour la visu (propulseurs...)

    public:
        FrForceAsset(std::shared_ptr<FrForce> myforce)  // TODO: ajouter couleur
                : m_force(myforce){
            // TODO: migrer vers l'implementation dans le cpp

            SetDrawMode(eCh_GlyphType::GLYPH_VECTOR);

            auto point = myforce->GetVrelpoint();
            auto forcevect = myforce->GetRelForce();
            SetGlyphVector(0, point, forcevect);
            SetGlyphsSize(20);  // Ne semble pas faire de difference avec Irrlicht


        }

        void Update(chrono::ChPhysicsItem* updater,
                    const chrono::ChCoordsys<>& coords) {

            // Here, the asset point is automatically following the motion but the force has to be updated
            SetGlyphVector(0, m_force->GetVpoint(), m_force->GetForce());

        }

    };












    //>>>>>>>>>>>>>>>>>>>>>>> REFACTORING



    /**
     * \class FrForceAsset_
     * \brief Class to display the loads.
     */
    class FrForceAsset_;


    class FrForceAsset_ : public FrAsset {

    private:

        FrForce_* m_force;  //< The force that this asset represents
        double OrderOfMagnitude;
        bool adaptive_OOM;

        double m_CharacteristicLength;
        chrono::ChColor m_symbolscolor;
        bool inverse_direction;

        // TODO ajouter flag pour dire si on affiche la force qui s'applique ou la force delivree pour la visu (propulseurs...)

    public:

        explicit FrForceAsset_(FrForce_* force);

        void Initialize() override;

        void Update() override;

        void StepFinalize() override {};

        friend void FrBody_::RemoveExternalForce(std::shared_ptr<frydom::FrForce_>);

    };




}  // end of frydom namespace


#endif //FRYDOM_FRFORCEASSET_H
