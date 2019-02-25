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
//#include "frydom/core/FrOffshoreSystem.h"
//#include "frydom/core/junk/FrNodeDynamic.h"
//#include "frydom/core/junk/FrSpringDampingForce.h"
//
//namespace frydom {
//
//    FrNodeDynamic::FrNodeDynamic() : FrHydroBody() {
//        SetMass(1.);
//    }
//
//    FrNodeDynamic::FrNodeDynamic(chrono::ChFrameMoving<>* ref_node, double T0, double psi) {
//        SetMass(1.);
//        SetSpringDamping(ref_node, T0, psi);
//    }
//
//    void FrNodeDynamic::SetSpringDamping(chrono::ChFrameMoving<>* ref_node,
//                                   const double T0,
//                                   const double psi) {
//        m_force = std::make_shared<FrSpringDampingForce>(ref_node, T0, psi);
//        AddForce(m_force);
//        SetPos_dt(ref_node->GetPos_dt());
//        //Set3DOF_ON();
//    }
//
//    void FrNodeDynamic::SetSteadyMotion(chrono::ChVector<double> velocity) {
//        SetPos_dt(velocity);
//    }
//
//
//    FrNodeMeanMotion::FrNodeMeanMotion() : FrHydroBody() {
//        m_velocitiesREC = std::make_unique<FrVelocityRecorder>();
//        m_positionsREC = std::make_unique<FrPositionRecorder>();
//    }
//
//
//    FrNodeMeanMotion::FrNodeMeanMotion(FrBody* body, double tmax) : FrHydroBody() {
//        m_velocitiesREC = std::make_unique<FrVelocityRecorder>();
//        m_positionsREC = std::make_unique<FrPositionRecorder>();
//        AttachedBody(body);
//        SetTmax(tmax);
//    }
//
//    void FrNodeMeanMotion::Initialize() {
//
//        Set3DOF_ON();
//
//        auto dt = GetSystem()->GetStep();
//
//        if (m_tmax > DBL_EPSILON && dt > DBL_EPSILON) {
//            m_size = uint(m_tmax/dt) + 1;
//        } else if (m_size > 0 && m_tmax < DBL_EPSILON) {
//            m_tmax = (m_size -1) * dt;
//        }
//
//        m_velocitiesREC->SetBody(m_body);
//        m_velocitiesREC->SetSize(m_size);
//        m_velocitiesREC->Initialize();
//
//        m_positionsREC->SetBody(m_body);
//        m_positionsREC->SetSize(m_size);
//        m_positionsREC->Initialize();
//
//        //SetPos(m_body->GetFrame_REF_to_abs().GetPos());
//        //SetPos_dt(m_body->GetFrame_REF_to_abs().GetPos_dt());
//
//        FrHydroBody::Initialize();
//
//    }
//
//    void FrNodeMeanMotion::Update(bool update_asset) {
//
//        m_velocitiesREC->RecordVelocity();
//        m_positionsREC->RecordPosition();
//
//        m_nstep = m_velocitiesREC->GetNStep();
//
//        auto vx = m_velocitiesREC->GetRecordOnDOF(0);
//        auto vy = m_velocitiesREC->GetRecordOnDOF(1);
//        //auto x = m_positionsREC->GetRecordOnDOF(0);
//        //auto y = m_positionsREC->GetRecordOnDOF(1);
//
//
//        //auto mean_vx = std::accumulate(vx.begin(), vx.end(), 0)/vx.size();
//        //auto mean_vy = std::accumulate(vy.begin(), vy.end(), 0)/vy.size();
//        //auto mean_x = std::accumulate(x.begin(), x.end(), 0)/m_size;
//        //auto mean_y = std::accumulate(y.begin(), y.end(), 0)/m_size;
//
//        double mean_vx = 0.;
//        double mean_vy = 0.;
//        double mean_x = 0.;
//        double mean_y = 0.;
//        for (unsigned int i=0; i<m_nstep; ++i) {
//            mean_vx += vx[i];
//            mean_vy += vy[i];
//            //mean_x += x[i];
//            //mean_y += y[i];
//        }
//        mean_vx = mean_vx/m_nstep;
//        mean_vy = mean_vy/m_nstep;
//        //mean_x = mean_x/m_nstep;
//        //mean_y = mean_y/m_nstep;
//
//        //SetPos( chrono::ChVector<double>(mean_x, mean_y, 0.));
//        SetPos_dt( chrono::ChVector<double>( mean_vx, mean_vy, 0.));
//        SetPos_dtdt(chrono::ChVector<double>(0., 0., 0.));
//
//        FrHydroBody::Update(update_asset);
//    }
//
//    //void FrNodeDynamic::Update(bool update_assets) {
//    //    FrHydroBody::Update(update_assets);
//    //}
//
//
//}
