//
// Created by Lucas Letournel on 06/12/18.
//

#include "FrSeabedGridAsset.h"

#include "frydom/core/FrBody.h"
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
