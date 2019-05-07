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

    FrMorisonSingleElement* FrMorisonForce::SetSingleElementModel(FrBody* body) {
        m_model = std::make_shared<FrMorisonSingleElement>(body);
        return dynamic_cast<FrMorisonSingleElement*>(m_model.get());
    }

    FrMorisonCompositeElement* FrMorisonForce::SetCompositeElementModel(FrBody* body) {
        m_model = std::make_shared<FrMorisonCompositeElement>(body);
        return dynamic_cast<FrMorisonCompositeElement*>(m_model.get());
    }

    void FrMorisonForce::Compute(double time) {

        m_model->Update(time);

        SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
        SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);
    }

    void FrMorisonForce::Initialize() {

        FrForce::Initialize();
        m_model->Initialize();
    }

    std::shared_ptr<FrMorisonForce>
    make_morison_force(std::shared_ptr<FrMorisonElement> model, std::shared_ptr<FrBody> body){
        assert(body.get() == model->GetNode()->GetBody());
        auto MorisonForce = std::make_shared<FrMorisonForce>(model);
        body->AddExternalForce(MorisonForce);
        return MorisonForce;
    }

}  // end namespace frydom
