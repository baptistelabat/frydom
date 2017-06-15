//
// Created by frongere on 14/06/17.
//

#ifndef FRYDOM_FRFORCEASSET_H
#define FRYDOM_FRFORCEASSET_H

#include "chrono/assets/ChGlyphs.h"
#include "chrono/physics/ChBody.h"
#include "FrForce.h"


// Cet asset doit etre ajoute directement au corps qui contient la force
// On peut faire de telle maniere que quand on ajoute l'asset a la force, c'est au corps de la force qu'il est ajoute...

namespace frydom {

    class FrForceAsset : public chrono::ChGlyphs {

    private:
        std::shared_ptr<FrForce> m_force;  //< The force that this asset represents
        double adim_coeff;

    public:
        FrForceAsset(std::shared_ptr<FrForce> myforce)  // TODO: ajouter couleur
                : m_force(myforce){

            SetDrawMode(eCh_GlyphType::GLYPH_VECTOR);

            auto mass = myforce->GetBody()->GetMass();
            adim_coeff = mass;

            auto point = myforce->GetVrelpoint();
            auto forcevect = myforce->GetRelForce();
//            auto forcevect = myforce->GetRelForce() / adim_coeff;
            SetGlyphVector(0, point, forcevect);


        }

        void Update(chrono::ChPhysicsItem* updater, const chrono::ChCoordsys<>& coords) {
            std::cout << "update asset" << std::endl;
        }

    };

}  // end of frydom namespace


#endif //FRYDOM_FRFORCEASSET_H
