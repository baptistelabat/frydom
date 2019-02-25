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


#ifndef FRYDOM_FRASSETCOMPONENT_H
#define FRYDOM_FRASSETCOMPONENT_H

#include <chrono/assets/ChColorAsset.h>
#include <chrono/assets/ChVisualization.h>
#include "frydom/core/common/FrObject.h"

namespace frydom {

    /**
     * \class FrAssetComponent
     * \brief Class the assets of all components (buoys, clump weights, etc.).
     */
    class FrAssetComponent : public FrObject {

    protected:

        std::shared_ptr<chrono::ChColorAsset> m_color;
        std::shared_ptr<chrono::ChVisualization> m_shape;

    public:

        std::shared_ptr<chrono::ChColorAsset> GetColorAsset();

        std::shared_ptr<chrono::ChVisualization> GetShapeAsset();

        chrono::ChColor GetColor();  // FIXME : pas de chrono !!!!

        void Initialize() override;

        void StepFinalize()override;

    };
}

#endif //FRYDOM_FRASSETCOMPONENT_H