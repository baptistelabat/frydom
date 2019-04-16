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

#include "chrono/assets/ChColor.h"

#include "frydom/asset/FrAsset.h"

// TODO : Cet asset doit etre ajoute directement au corps qui contient la force
// On peut faire de telle maniere que quand on ajoute l'asset a la force, c'est au corps de la force qu'il est ajoute...

namespace frydom {

    // Forward declaration
    class FrForce;


    /**
     * \class FrForceAsset
     * \brief Class to display the loads.
     */
    class FrForceAsset : public FrAsset {

    private:

        FrForce* m_force;  //< The force that this asset represents
        double OrderOfMagnitude;
        bool adaptive_OOM;

        double m_CharacteristicLength;
        chrono::ChColor m_symbolscolor;
        bool inverse_direction;

        // TODO ajouter flag pour dire si on affiche la force qui s'applique ou la force delivree pour la visu (propulseurs...)

    public:

        explicit FrForceAsset(FrForce* force);

        void SetSize(double size);

        void Initialize() override;

        void StepFinalize() override;

        friend void FrBody::RemoveExternalForce(std::shared_ptr<frydom::FrForce>);

    };

}  // end namespace frydom


#endif //FRYDOM_FRFORCEASSET_H
