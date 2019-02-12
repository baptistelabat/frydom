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


#ifndef FRYDOM_FRSEABEDGRIDASSET_H
#define FRYDOM_FRSEABEDGRIDASSET_H

#include "FrGridAsset.h"

namespace frydom {
    class FrSeabed_;

    /**
     * \class FrSeabedGridAsset
     * \brief Class to display the seabed grid.
     */
    class FrSeabedGridAsset : public FrGridAsset {

        FrSeabed_* m_seabed;    ///> Seabed containing this asset
    public:
        /// Default constructor
        /// \param body body containing this asset (usually WorldBody)
        /// \param seabed pointer ot the seabed
        explicit FrSeabedGridAsset(FrBody_* body, FrSeabed_* seabed);

        /// Get the seabed grid height
        /// \return
        double GetGridHeight() const override;

    };

}
#endif //FRYDOM_FRSEABEDGRIDASSET_H
