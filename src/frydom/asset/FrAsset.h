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


#ifndef FRYDOM_FRASSET_H
#define FRYDOM_FRASSET_H

#include <chrono/assets/ChAsset.h>
#include "frydom/core/body/FrBody.h"

namespace frydom {

    /**
     * \class FrAsset
     * \brief
     */
    class FrAsset {

    protected:

        virtual std::shared_ptr<chrono::ChAsset> GetChronoAsset() = 0;

    public:

        friend void FrBody_::AddAsset(std::shared_ptr<FrAsset>);

    };

}   // end namespace frydom

#endif //FRYDOM_FRASSET_H
