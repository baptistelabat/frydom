//
// Created by Lucas Letournel on 06/12/18.
//

#ifndef FRYDOM_FRSEABEDGRIDASSET_H
#define FRYDOM_FRSEABEDGRIDASSET_H

#include "FrGridAsset.h"

namespace frydom {
    class FrSeabed_;

    class FrSeabedGridAsset : public FrGridAsset {
        /// Seabed containing this asset
        FrSeabed_* m_seabed;
    public:
        /// Default constructor
        /// \param seabed seabed containing this asset
        explicit FrSeabedGridAsset(FrSeabed_* seabed);

        /// Get the seabed grid height
        /// \return
        double GetGridHeight() const override;

    };

}
#endif //FRYDOM_FRSEABEDGRIDASSET_H
