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


#include "FrSeabedGridAsset.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"

#include <frydom/environment/ocean/seabed/FrSeabed.h>

namespace frydom{
    FrSeabedGridAsset::FrSeabedGridAsset(FrBody_ *body, FrSeabed_* seabed): FrGridAsset(body)  {
        m_seabed = seabed;
        SetGridColor(Chocolate);
    }

    double FrSeabedGridAsset::GetGridHeight() const {return m_seabed->GetBathymetry(NWU);}
}
