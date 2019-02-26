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

#include "chrono/assets/ChColor.h"  // TODO : retirer et utiliser FrColor !!!


// Forward declarations
namespace chrono {
    class ChColorAsset;
    class ChVisualization;
}


namespace frydom {

    /**
     * \class FrAssetComponent
     * \brief Class the assets of all components (buoys, clump weights, etc.).
     */
    class FrAssetComponent {

        // FIXME : ne pas reposer sur les objets chrono !!!

    protected:

        std::shared_ptr<chrono::ChColorAsset> m_color;
        std::shared_ptr<chrono::ChVisualization> m_shape;

    public:

        std::shared_ptr<chrono::ChColorAsset> GetColorAsset();

        std::shared_ptr<chrono::ChVisualization> GetShapeAsset();

        chrono::ChColor GetColor();  // FIXME : pas de chrono !!!!

    };

}  // end namespace frydom

#endif //FRYDOM_FRASSETCOMPONENT_H
