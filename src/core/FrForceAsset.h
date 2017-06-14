//
// Created by frongere on 14/06/17.
//

#ifndef FRYDOM_FRFORCEASSET_H
#define FRYDOM_FRFORCEASSET_H

#include "chrono/assets/ChLineShape.h"
#include "FrForce.h"

// Cet asset doit etre ajoute directement au corps qui contient la force
// On peut faire de telle maniere que quand on ajoute l'asset a la force, c'est au corps de la force qu'il est ajoute...

namespace frydom {

    class FrForceAsset : public chrono::ChLineShape {

    private:
        std::shared_ptr<FrForce> m_force;  //< The force that this asset represents

    public:
        FrForceAsset(std::shared_ptr<FrForce> myforce)
                : m_force(myforce) {};

    };

}  // end of frydom namespace


#endif //FRYDOM_FRFORCEASSET_H
