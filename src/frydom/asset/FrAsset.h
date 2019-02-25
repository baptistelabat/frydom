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


#ifndef FRYDOM_FRASSET_H
#define FRYDOM_FRASSET_H

#include <chrono/assets/ChAsset.h>
#include "frydom/core/body/FrBody.h"

namespace frydom {

    class FrAsset;

    namespace internal{

        struct FrAssetBase_ : public chrono::ChAssetLevel {

            FrAsset * m_frydomAsset;

            explicit FrAssetBase_(FrAsset * asset);

            void Update(chrono::ChPhysicsItem* updater, const chrono::ChCoordsys<>& coords) override;

        };

    }

    /**
     * \class FrAsset
     * \brief
     */
    class FrAsset : public FrObject {

    protected:
        std::shared_ptr<internal::FrAssetBase_> m_chronoAsset;

        std::shared_ptr<chrono::ChAsset> GetChronoAsset();

    public:

        FrAsset();

        virtual void Update() = 0;

        friend void FrBody_::AddAsset(std::shared_ptr<FrAsset>);
    };

}   // end namespace frydom

#endif //FRYDOM_FRASSET_H
