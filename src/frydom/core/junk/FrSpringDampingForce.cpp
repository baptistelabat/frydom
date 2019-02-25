//// ==========================================================================
//// FRyDoM - frydom-ce.org
////
//// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
//// All rights reserved.
////
//// Use of this source code is governed by a GPLv3 license that can be found
//// in the LICENSE file of FRyDoM.
////
//// ==========================================================================
//
//
//#include "frydom/core/common/FrNode.h"
//#include "FrSpringDampingForce.h"
//
//#include "frydom/core/body/FrBody.h"
//
//namespace frydom {
//
//    FrSpringDampingForce::FrSpringDampingForce(chrono::ChFrameMoving<>* ref_node, const double T0, const double psi) {
//        SetNode(ref_node);
//        SetParam(T0, psi);
//    }
//
//    void FrSpringDampingForce::SetParam(const double T0, const double psi) {
//        m_w0 = 2.*M_PI / T0;
//        m_psi = psi;
//    }
//
//    void FrSpringDampingForce::Initialize() {
//        auto mybody = GetBody();
//        m_damping = 2. * mybody->GetMass() * m_psi * m_w0;
//        m_stiffness = m_w0 * m_w0;
//    }
//
//    void FrSpringDampingForce::UpdateState() {
//
//        auto position = GetBody()->GetPos();
//        auto velocity = GetBody()->GetPos_dt();
//        auto node_pos = m_node->GetPos();
//
//        force = -m_stiffness * (position - node_pos);
//        force -= m_damping * velocity;
//
//    }
//
//}
