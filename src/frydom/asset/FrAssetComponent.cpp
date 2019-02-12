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


#include "FrAssetComponent.h"


namespace frydom {


    std::shared_ptr<chrono::ChColorAsset> FrAssetComponent::GetColorAsset() {return m_color;}

    std::shared_ptr<chrono::ChVisualization> FrAssetComponent::GetShapeAsset() {return m_shape;}

    chrono::ChColor FrAssetComponent::GetColor() {return m_color->GetColor();}

    void FrAssetComponent::Initialize() {}

    void FrAssetComponent::StepFinalize() {}
}  // end namespace frydom
