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


#include "FrMorisonForce.h"

#include "frydom/hydrodynamics/morison/FrMorisonModel.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrNode.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrMorisonSingleElement<OffshoreSystemType> *
    FrMorisonForce<OffshoreSystemType>::SetSingleElementModel(FrBody<OffshoreSystemType> *body) {
      m_model = std::make_shared<FrMorisonSingleElement>(body);
      return dynamic_cast<FrMorisonSingleElement<OffshoreSystemType> *>(m_model.get());
    }

    template<typename OffshoreSystemType>
    FrMorisonCompositeElement<OffshoreSystemType> *
    FrMorisonForce<OffshoreSystemType>::SetCompositeElementModel(FrBody<OffshoreSystemType> *body) {
      m_model = std::make_shared<FrMorisonCompositeElement>(body);
      return dynamic_cast<FrMorisonCompositeElement<OffshoreSystemType> *>(m_model.get());
    }

    template<typename OffshoreSystemType>
    void FrMorisonForce<OffshoreSystemType>::Compute(double time) {

      m_model->Update(time);

      SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
      SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);
    }

    template<typename OffshoreSystemType>
    void FrMorisonForce<OffshoreSystemType>::Initialize() {

      FrForce<OffshoreSystemType>::Initialize();
      m_model->Initialize();
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrMorisonForce<OffshoreSystemType>>
    make_morison_force(std::shared_ptr<FrMorisonElement<OffshoreSystemType>> model,
                       std::shared_ptr<FrBody<OffshoreSystemType>> body) {
      assert(body.get() == model->GetNode()->GetBody());
      auto MorisonForce = std::make_shared<FrMorisonForce>(model);
      body->AddExternalForce(MorisonForce);
      return MorisonForce;
    }

}  // end namespace frydom
