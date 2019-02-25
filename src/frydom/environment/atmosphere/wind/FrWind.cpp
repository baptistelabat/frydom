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

#include "chrono/core/ChMatrixDynamic.h"  // TODO : voir pourquoi on est oblige d'inclure

#include "frydom/core/common/FrFrame.h"

#include "frydom/environment/atmosphere/FrAtmosphere_.h"


namespace frydom {

    FrEnvironment_* FrWind_::GetEnvironment() const {
        return m_atmosphere->GetEnvironment();
    }

}  // end namespace frydom
