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


// Chrono forward declaration
namespace chrono {
    class ChLineShape;
}

namespace frydom {

    // Forward declarations:
    class FrCatenaryLine;

    /**
     * \class FrCatenaryLineAsset_
     * \brief Class for a catenary line asset, using chrono::ChLineShape in aggregation
     * Line elements of ChLineShape are updated in position and color (related to the tension) in this class
     */
    class FrCatenaryLineAsset_ {

        // TODO : ne pas reposer sur les objets chrono!!!

    private:

        FrCatenaryLine *m_catenaryLine;    ///< Catenary line containing this asset

        using Triplet = std::tuple<double, double, std::shared_ptr<chrono::ChLineShape>>;
        std::vector<Triplet> m_elements;    ///< container of elements based on ChLineShape

        double m_maxTension = 0;            ///< max tension cached value for the color visualization

    public:

        /// Catenary line asset constructor
        /// \param line catenary line containing this asset
        explicit FrCatenaryLineAsset_(FrCatenaryLine * line);

        /// Update the elements position and color
        void Update();

        /// Initialize the asset by creating the elements
        void Initialize();

    private:

        /// Initialize the max tension value for the color visualization
        void InitRangeTensionColor();

        /// Make a tuple, based on the starting and ending lagrangian coordinates and the corresponding element
        /// \param s0 starting lagrangian coordinate of the element
        /// \param s1 ending lagrangiand coordinate of the element
        /// \param lineShape element based on ChLineShape
        /// \return
        static Triplet make_triplet(double s0, double s1, std::shared_ptr<chrono::ChLineShape> lineShape) {
            return std::make_tuple(s0, s1, lineShape);
        }

    };


}  //end namespace frydom

#endif //FRYDOM_FRCATENARYLINEASSET_H
