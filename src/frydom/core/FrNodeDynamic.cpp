//
// Created by camille on 05/06/18.
//

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/FrNodeDynamic.h"
#include "frydom/core/FrSpringDampingForce.h"

namespace frydom {

    FrNodeDynamic::FrNodeDynamic() : FrHydroBody() {
        SetMass(1.);
    }

    void FrNodeDynamic::SetSpringDamping(chrono::ChFrameMoving<>* ref_node,
                                   const double T0,
                                   const double psi) {
        m_force = std::make_shared<FrSpringDampingForce>(ref_node, T0, psi);
        AddForce(m_force);
        SetPos_dt(ref_node->GetPos_dt());
        Set3DOF_ON();
    }

    void FrNodeDynamic::SetSteadyMotion(chrono::ChVector<double> velocity) {
        SetPos_dt(velocity);
    }


    //void FrNodeDynamic::Update(bool update_assets) {
    //    FrHydroBody::Update(update_assets);
    //}


}