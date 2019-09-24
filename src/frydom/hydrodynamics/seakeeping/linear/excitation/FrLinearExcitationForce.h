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
#include "FrLinearHDBForce.h"

namespace frydom {

    // Forward declaration
    class FrHydroDB;
    class FrBody;
    class FrEquilibriumFrame;

    /**
     * \class FrLinearExcitationForce
     * \brief Class for computing the linear excitation loads.
     */

    class FrLinearExcitationForce : public FrLinearHDBForce {

    public:

        /// Constructor.
        explicit FrLinearExcitationForce(const std::shared_ptr<FrHydroDB>& HDB) : FrLinearHDBForce(HDB) {};

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearExcitationForce"; }

        Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

        Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    };

    std::shared_ptr<FrLinearExcitationForce>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEAREXCITATIONFORCE_H
