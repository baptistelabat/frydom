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
//#include "FrVelocityRecorder.h"
//
//
//namespace frydom {
//
//    FrVelocityRecorder::FrVelocityRecorder(FrBody *body) : m_body(body) {}
//
//    void FrVelocityRecorder::SetSize(unsigned int size) { m_size = size; }
//
//    unsigned int FrVelocityRecorder::GetSize() const { return m_size; }
//
//    unsigned int FrVelocityRecorder::GetNStep() const { return std::min(m_nstep, m_size); }
//
//    FrBody *FrVelocityRecorder::GetBody() const { return m_body; }
//
//    void FrVelocityRecorder::Initialize() {
//
//        // Initializing every 6 recorders (6 DOF record) with size m_size (full buffer) and values 0.
//        m_velocities.reserve(6);
//        for (unsigned int i=0; i<6; i++) {
//            m_velocities.emplace_back(boost::circular_buffer<double>(m_size, m_size, 0.));
//        }
//    }
//
//    double FrVelocityRecorder::GetTime() const { return m_lastTime; }
//
//    chrono::ChVector<double> FrVelocityRecorder::GetLinearVelocity() const {
//        return m_body->GetPos_dt();
//    }
//
//    chrono::ChVector<double> FrVelocityRecorder::GetAngularVelocity() const {
//        auto angular_velocity = m_body->GetWvel_loc();
//        return m_body->TransformDirectionLocalToParent(angular_velocity);
//    }
//
//    void FrVelocityRecorder::RecordVelocity() {
//
//        auto currentTime = m_body->GetChTime();
//        if (m_lastTime == currentTime) return;
//        m_lastTime = currentTime;
//
//        auto linear_velocity = GetLinearVelocity();
//        auto angular_velocity = GetAngularVelocity();
//
//        // Note taht we use here push_front in order to get a conveniently organized  buffer
//        // for the calculation of hydrodynamic radiation convolutions
//        m_velocities[0].push_front(linear_velocity[0]);
//        m_velocities[1].push_front(linear_velocity[1]);
//        m_velocities[2].push_front(linear_velocity[2]);
//
//        m_velocities[3].push_front(angular_velocity[0]);
//        m_velocities[4].push_front(angular_velocity[1]);
//        m_velocities[5].push_front(angular_velocity[2]);
//
//        m_nstep += 1;
//
//    }
//
//    boost::circular_buffer<double> FrVelocityRecorder::GetRecordOnDOF(unsigned int iDOF) const {
//        return m_velocities[iDOF];
//    }
//
//    void FrVelocityRecorder::StepFinalize() {}
//
//    void FrVelocityRecorder::SetBody(FrBody *body) { m_body=body; }
//
//
//}  // end namespace frydom
