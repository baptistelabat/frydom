//
// Created by Lucas Letournel on 07/08/18.
//

#ifndef FRYDOM_FRASSETCOMPONENT_H
#define FRYDOM_FRASSETCOMPONENT_H

#include <chrono/assets/ChColorAsset.h>
#include <chrono/assets/ChVisualization.h>
#include "frydom/core/FrObject.h"

namespace frydom {

    class FrAssetComponent : public FrObject {

    protected:

        std::shared_ptr<chrono::ChColorAsset> m_color;
        std::shared_ptr<chrono::ChVisualization> m_shape;

    public:

        std::shared_ptr<chrono::ChColorAsset> GetColorAsset() {return m_color;}

        std::shared_ptr<chrono::ChVisualization> GetShapeAsset() {return m_shape;}

        chrono::ChColor GetColor(){return m_color->GetColor();}

        void Initialize()override {};

        void StepFinalize()override {};

    };
}

#endif //FRYDOM_FRASSETCOMPONENT_H
