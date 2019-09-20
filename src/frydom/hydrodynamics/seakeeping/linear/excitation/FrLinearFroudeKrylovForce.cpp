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

#include "FrLinearFroudeKrylovForce.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

    template<typename OffshoreSystemType>
    void FrLinearFroudeKrylovForce<OffshoreSystemType>::Initialize() {

      // Initialization of the parent class.
      FrLinearExcitationForceBase<OffshoreSystemType>::Initialize();

    }

    template<typename OffshoreSystemType>
    Eigen::MatrixXcd FrLinearFroudeKrylovForce<OffshoreSystemType>::GetHDBData(unsigned int iangle) const {

      auto BEMBody = this->m_HDB->GetBody(this->m_body);

      return BEMBody->GetFroudeKrylov(iangle);

    }

    template<typename OffshoreSystemType>
    Eigen::VectorXcd FrLinearFroudeKrylovForce<OffshoreSystemType>::GetHDBData(unsigned int iangle, unsigned int iforce) const {

      auto BEMBody = this->m_HDB->GetBody(this->m_body);

      return BEMBody->GetFroudeKrylov(iangle, iforce);

    }

    template<typename OffshoreSystemType>
    void FrLinearFroudeKrylovForce<OffshoreSystemType>::Compute(double time) {

      // This function computes the linear Froude-Krylov forces from Nemoh results.

      this->Compute_F_HDB();
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearFroudeKrylovForce<OffshoreSystemType>>
    make_linear_froude_krylov_force(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB, std::shared_ptr<FrBody<OffshoreSystemType>> body) {

      // This function creates the linear Froude-Krylov force object.

      // Construction of the excitation force object from the HDB.
      auto LinFKForce = std::make_shared<FrLinearFroudeKrylovForce>(HDB);

      // Add the excitation force object as an external force to the body.
      body->AddExternalForce(LinFKForce); // Initialization of m_body.

      return LinFKForce;

    }

}  // end namespace frydom
