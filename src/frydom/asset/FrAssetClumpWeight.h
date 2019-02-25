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


#ifndef FRYDOM_FRASSETCLUMPWEIGHT_H
#define FRYDOM_FRASSETCLUMPWEIGHT_H

#include "chrono/physics/ChBodyEasy.h"
#include "FrAssetComponent.h"


namespace frydom {

    /**
     * \class FrAssetClumpWeight
     * \brief Class for the clump weight assets.
     */
    class FrAssetClumpWeight : public FrAssetComponent {
        // FIXME : ne pas reposer sur les objets chrono!!!

    public:

        FrAssetClumpWeight(double mradius,chrono::ChColor mcolor,chrono::ChVector<> P1, chrono::ChVector<> P2);

        FrAssetClumpWeight(double mradius,chrono::ChVector<> P1, chrono::ChVector<> P2);

        explicit FrAssetClumpWeight(double mradius);

        FrAssetClumpWeight();

    };
}
#endif //FRYDOM_FRASSETCLUMPWEIGHT_H
