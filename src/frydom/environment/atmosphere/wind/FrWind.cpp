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


#include "FrWind.h"

#include "frydom/environment/atmosphere/FrAtmosphere.h"


namespace frydom {

    FrEnvironment* FrWind::GetEnvironment() const {
        return m_atmosphere->GetEnvironment();
    }

}  // end namespace frydom
