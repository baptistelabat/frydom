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


#ifndef FRYDOM_FRCATENARYLINEASSET_H
#define FRYDOM_FRCATENARYLINEASSET_H

#include <tuple>
#include <memory>
#include <vector>

#include "frydom/asset/FrAsset.h"


// Chrono forward declaration
namespace chrono {
    class ChLineShape;
}

namespace frydom {

    // Forward declarations:
    class FrCatenaryLine;

    /**
     * \class FrCatenaryLineAsset
     * \brief Class for a catenary line asset, using chrono::ChLineShape in aggregation
     * Line elements of ChLineShape are updated in position and color (related to the tension) in this class
     */
    class FrCatenaryLineAsset : public FrAsset {

    private:

        FrCable *m_catenaryLine;    ///< Catenary line containing this asset

//        using Triplet = std::tuple<double, double, std::shared_ptr<chrono::ChLineShape>>;
//        std::vector<Triplet> m_elements;    ///< container of elements based on ChLineShape
        using Triplet = std::tuple<double, double, unsigned int>;
        std::vector<Triplet> m_elements;

        double m_maxTension = 0;            ///< max tension cached value for the color visualization

    public:

        /// Catenary line asset constructor
        /// \param line catenary line containing this asset
        explicit FrCatenaryLineAsset(FrCable * line);

        /// Initialize the asset by creating the elements
        void Initialize() override;

        /// Update the state of the asset, at the end of a time step
        void StepFinalize() override;

    private:

        /// Initialize the max tension value for the color visualization
        void InitRangeTensionColor();

        /// Make a tuple, based on the starting and ending lagrangian coordinates and the corresponding element
        /// \param s0 starting lagrangian coordinate of the element
        /// \param s1 ending lagrangiand coordinate of the element
        /// \param index index of the element based on ChLineShape
        /// \return
        static Triplet make_triplet(double s0, double s1, unsigned int index) {
            return std::make_tuple(s0, s1, index);
        }

    };


}  //end namespace frydom

#endif //FRYDOM_FRCATENARYLINEASSET_H
