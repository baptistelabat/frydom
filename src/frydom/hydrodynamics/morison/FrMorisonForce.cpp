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
//#include "frydom/hydrodynamics/morison/FrMorisonModel.h"
//#include "frydom/core/body/FrBody.h"

namespace frydom {

    FrMorisonSingleElement_* FrMorisonForce_::SetSingleElementModel(FrBody_* body) {
        m_model = std::make_shared<FrMorisonSingleElement_>(body);
        return dynamic_cast<FrMorisonSingleElement_*>(m_model.get());
    }

    FrMorisonCompositeElement_* FrMorisonForce_::SetCompositeElementModel(FrBody_* body) {
        m_model = std::make_shared<FrMorisonCompositeElement_>(body);
        return dynamic_cast<FrMorisonCompositeElement_*>(m_model.get());
    }

    void FrMorisonForce_::Update(double time) {

        m_model->Update(time);

        SetForceInWorldAtCOG(m_model->GetForceInWorld(NWU), NWU);
        SetTorqueInBodyAtCOG(m_model->GetTorqueInBody(), NWU);
    }

    void FrMorisonForce_::Initialize() {

        FrForce_::Initialize();
        m_model->Initialize();
    }

    void FrMorisonForce_::StepFinalize() {
        FrForce_::StepFinalize();
    }

    std::shared_ptr<FrMorisonForce_>
    make_morison_force(std::shared_ptr<FrMorisonElement_> model, std::shared_ptr<FrBody_> body){
        assert(body.get() == model->GetNode()->GetBody());
        auto MorisonForce = std::make_shared<FrMorisonForce_>(model);
        body->AddExternalForce(MorisonForce);
        return MorisonForce;
    }

}  // end namespace frydom
