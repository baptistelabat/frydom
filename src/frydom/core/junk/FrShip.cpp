// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#include "FrShip.h"

namespace frydom {



    void FrShip::Update(bool update_assets) {

        // Update parent
        FrHydroBody::Update(update_assets);
    }

    void FrShip::StepFinalize(){
        FrHydroBody::StepFinalize();
    }


}  // end namespace frydom
