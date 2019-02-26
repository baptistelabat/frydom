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


#ifndef FRYDOM_FRWINDFORCE_H
#define FRYDOM_FRWINDFORCE_H

#include "frydom/core/force/FrForce.h"
#include "MathUtils/LookupTable1D.h"

namespace frydom {

    /**
     * \class FrWindForce_
     * \brief Class for computing the wind loads.
     */
    class FrWindForce_ : public FrForce {


    private:
        mathutils::LookupTable1D<double, double> m_table;

    public:

        /// Default constructor
        FrWindForce_() = default;

        /// Constructor from the yaml file
        explicit FrWindForce_(std::string yamlFile);

        /// Update the state of the force
        void Update(double time) override;

        void Initialize() override {};

        void StepFinalize() override;;

    private:

        /// Read the drag and lift coefficient from yaml file
        void ReadTable(std::string yamlFile);

    };


}  // end namespace frydom


#endif //FRYDOM_FRWINDFORCE_H
