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


#include "FrSeabedGridAsset.h"

#include "frydom/environment/ocean/seabed/FrSeabed.h"


namespace frydom{

    template <typename OffshoreSystemType>
    FrSeabedGridAsset<OffshoreSystemType>::FrSeabedGridAsset(FrSeabed<OffshoreSystemType>* seabed): FrGridAsset()  {
        m_seabed = seabed;
        SetGridColor(Chocolate);
    }

    template <typename OffshoreSystemType>
    double FrSeabedGridAsset<OffshoreSystemType>::GetGridHeight() const {return m_seabed->GetBathymetry(NWU);}

}  // end namespace frydom
