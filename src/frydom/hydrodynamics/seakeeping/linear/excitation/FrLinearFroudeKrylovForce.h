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
#include "FrLinearHDBForce.h"

namespace frydom {

    // Forward declaration
    class FrHydroDB;

    class FrBody;

    class FrEquilibriumFrame;

    /**
     * \class FrLinearFroudeKrylovForce
     * \brief Class for computing the linear Froude-Krylov loads.
     */
    class FrLinearFroudeKrylovForce : public FrLinearHDBForce {

     public:

      /// Constructor.
      FrLinearFroudeKrylovForce(const std::string &name,
                                FrBody* body,
                                const std::shared_ptr<FrHydroDB> &HDB);;

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "LinearFroudeKrylovForce"; }

      Eigen::MatrixXcd GetHDBData(unsigned int iangle) const override;

      Eigen::VectorXcd GetHDBData(unsigned int iangle, unsigned int iforce) const override;

    };

    std::shared_ptr<FrLinearFroudeKrylovForce>
    make_linear_froude_krylov_force(const std::string &name,
                                    std::shared_ptr<FrBody> body,
                                    std::shared_ptr<FrHydroDB> HDB);


}  // end namespace frydom

#endif //FRYDOM_FRLINEARFROUDEKRYLOVFORCE_H
