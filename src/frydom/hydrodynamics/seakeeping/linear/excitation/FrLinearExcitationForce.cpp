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

#include "FrLinearExcitationForce.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOceanInc.h"

namespace frydom {

    template<typename OffshoreSystemType>
    void FrLinearExcitationForce<OffshoreSystemType>::Initialize() {

      // Initialization of the parent class.
      FrLinearExcitationForceBase<OffshoreSystemType>::Initialize();

    }

    template<typename OffshoreSystemType>
    Eigen::MatrixXcd FrLinearExcitationForce<OffshoreSystemType>::GetHDBData(unsigned int iangle) const {

      auto BEMBody = this->m_HDB->GetBody(this->m_body);

      return BEMBody->GetExcitation(iangle);

    }

    template<typename OffshoreSystemType>
    Eigen::VectorXcd FrLinearExcitationForce<OffshoreSystemType>::GetHDBData(unsigned int iangle, unsigned int iforce) const {

      auto BEMBody = this->m_HDB->GetBody(this->m_body);

      return BEMBody->GetExcitation(iangle, iforce);

    }

    template<typename OffshoreSystemType>
    void FrLinearExcitationForce<OffshoreSystemType>::Compute(double time) {

      // This function computes the linear excitation forces from Nemoh results.

      Compute_F_HDB();

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrLinearExcitationForce<OffshoreSystemType>>
    make_linear_excitation_force(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB, std::shared_ptr<FrBody<OffshoreSystemType>> body) {

      // This function creates the linear excitation force object.

      // Construction of the excitation force object from the HDB.
      auto excitationForce = std::make_shared<FrLinearExcitationForce>(HDB);

      // Add the excitation force object as an external force to the body.
      body->AddExternalForce(excitationForce); // Initialization of m_body.

      return excitationForce;

    }

}  // end namespace frydom
