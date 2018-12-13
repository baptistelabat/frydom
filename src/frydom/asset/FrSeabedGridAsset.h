//
// Created by Lucas Letournel on 06/12/18.
//

#ifndef FRYDOM_FRSEABEDGRIDASSET_H
#define FRYDOM_FRSEABEDGRIDASSET_H

#include "FrGridAsset.h"

namespace frydom {
    class FrSeabed_;

    class FrSeabedGridAsset : public FrGridAsset {

        FrSeabed_* m_seabed;    ///> Seabed containing this asset
    public:
        /// Default constructor
        /// \param body body containing this asset (usually WorldBody)
        explicit FrSeabedGridAsset(FrBody_* body, FrSeabed_* seabed);

        /// Get the seabed grid height
        /// \return
        double GetGridHeight() const override;

    };

}
#endif //FRYDOM_FRSEABEDGRIDASSET_H
