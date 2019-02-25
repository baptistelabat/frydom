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

#include "FrCurrent.h"

#include "frydom/environment/ocean/FrOcean_.h"

namespace frydom {

    FrEnvironment_* FrCurrent_::GetEnvironment() const {
        return m_ocean->GetEnvironment();
    }

}  // end namespace frydom
