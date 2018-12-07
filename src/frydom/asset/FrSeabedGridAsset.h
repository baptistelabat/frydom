//
// Created by Lucas Letournel on 06/12/18.
//

#ifndef FRYDOM_FRSEABEDGRIDASSET_H
#define FRYDOM_FRSEABEDGRIDASSET_H

#include "FrGridAsset.h"

namespace frydom {
    class FrSeabed_;

    class FrSeabedGridAsset : public FrGridAsset {
        FrSeabed_* m_seabed;
    public:
        explicit FrSeabedGridAsset(FrSeabed_* seabed);

        double GetGridHeight() const override;

    };

}
#endif //FRYDOM_FRSEABEDGRIDASSET_H
