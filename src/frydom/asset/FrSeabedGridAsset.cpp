//
// Created by Lucas Letournel on 06/12/18.
//

#include "FrSeabedGridAsset.h"

#include <frydom/environment/ocean/seabed/FrSeabed.h>

namespace frydom{
    FrSeabedGridAsset::FrSeabedGridAsset(FrSeabed_ *seabed) {
        m_seabed = seabed;
        SetGridColor(Chocolate);
    }

    double FrSeabedGridAsset::GetGridHeight() const {return m_seabed->GetBathymetry();}
}
