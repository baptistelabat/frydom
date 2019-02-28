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


#ifndef FRYDOM_FRSEABEDGRIDASSET_H
#define FRYDOM_FRSEABEDGRIDASSET_H

#include "FrGridAsset.h"

namespace frydom {

    // Forward declaration
    class FrSeabed;

    /**
     * \class FrSeabedGridAsset
     * \brief Class to display the seabed grid.
     */
    class FrSeabedGridAsset : public FrGridAsset {

        FrSeabed* m_seabed;    ///> Seabed containing this asset

    public:
        /// Default constructor
        /// \param seabed pointer ot the seabed
        explicit FrSeabedGridAsset(FrSeabed* seabed);

        /// Get the seabed grid height
        /// \return
        double GetGridHeight() const override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRSEABEDGRIDASSET_H
