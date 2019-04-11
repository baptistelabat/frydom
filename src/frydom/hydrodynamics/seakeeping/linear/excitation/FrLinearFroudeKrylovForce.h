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


#ifndef FRYDOM_FRLINEARFROUDEKRYLOVFORCE_H
#define FRYDOM_FRLINEARFROUDEKRYLOVFORCE_H

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
     * \class FrLinearFroudeKrylovForce
     * \brief Class for computing the linear Froude-Krylov loads.
     */
    class FrLinearFroudeKrylovForce : public FrLinearExcitationForceBase {

    public:

        /// Constructor.
        explicit FrLinearFroudeKrylovForce(std::shared_ptr<FrHydroDB> HDB) : FrLinearExcitationForceBase(HDB) {};

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearFroudeKrylovForce"; }

        void Initialize() override;

        /// This function is called at the end of the time step, after the last step of the integration scheme.
        void StepFinalize() override;

        Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

        Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    private:

        /// Compute the linear Froude-Krylov force
        /// \param time Current time of the simulation from beginning, in seconds
        void Compute(double time) override;

    };

    std::shared_ptr<FrLinearFroudeKrylovForce>
    make_linear_froude_krylov_force(std::shared_ptr<FrHydroDB> HDB, std::shared_ptr<FrBody> body);


}  // end namespace frydom

#endif //FRYDOM_FRLINEARFROUDEKRYLOVFORCE_H
