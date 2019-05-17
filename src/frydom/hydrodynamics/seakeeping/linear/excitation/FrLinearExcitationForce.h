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


#ifndef FRYDOM_FRLINEAREXCITATIONFORCE_H
#define FRYDOM_FRLINEAREXCITATIONFORCE_H

#include <memory>
#include <vector>

#include "MathUtils/Matrix66.h"
#include "frydom/core/force/FrForce.h"
#include "FrLinearExcitationForceBase.h"

namespace frydom {

    // Forward declaration
    class FrHydroDB;
    class FrBody;
    class FrEquilibriumFrame;

    /**
     * \class FrLinearExcitationForce
     * \brief Class for computing the linear excitation loads.
     */

    class FrLinearExcitationForce : public FrLinearExcitationForceBase {

    public:

        /// Constructor.
        explicit FrLinearExcitationForce(std::shared_ptr<FrHydroDB> HDB) : FrLinearExcitationForceBase(HDB) {};

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearExcitationForce"; }

        /// Method to initialize the linear excitation force
        void Initialize() override;

        Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

        Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    private:

        /// Compute the linear excitation force
        /// \param time Current time of the simulation from beginning, in seconds
        void Compute(double time) override;

    };

    std::shared_ptr<FrLinearExcitationForce>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
