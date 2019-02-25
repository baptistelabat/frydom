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


#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "frydom/core/force/FrForce.h"
#include "FrCurrentPolarCoeffs.h"

namespace frydom {

    /**
     * \class FrCurrentForce_
     * \brief Class for computing the current loads.
     */
    class FrCurrentForce_ : public FrForce_ {

    private:
        FrCurrentPolarCoeffs m_coeffsTable;

    public:

        /// Default constructor
        FrCurrentForce_() = default;

        /// Constructor from YAML file
        explicit FrCurrentForce_(std::string yamlFile);

        /// Update the state of the force
        void Update(double time) override;

        void Initialize() override {};

        void StepFinalize() override;;

    };

}  // end namespace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
