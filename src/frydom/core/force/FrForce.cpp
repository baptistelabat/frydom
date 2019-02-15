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



#include "FrForce.h"

#include "frydom/core/body/FrBody.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/asset/FrForceAsset.h"


namespace frydom{

//    void FrForce::GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const {
//        body_force = force;
//        body_torque = moment;
//    }




//    void FrForce::UpdateApplicationPoint() {
//
//        auto my_body = GetBody();
//        auto vmotion = chrono::VNULL;
//
//        if (move_x)
//            vmotion.x() = move_x->Get_y(ChTime);
//        if (move_y)
//            vmotion.y() = move_y->Get_y(ChTime);
//        if (move_z)
//            vmotion.z() = move_z->Get_y(ChTime);
//
//        switch (frame) {
//            case WORLD:
//                vpoint = Vadd(restpos, vmotion);                // Uw
//                vrelpoint = my_body->Point_World2Body(vpoint);  // Uo1 = [A]'(Uw-Xo1)
//                break;
//            case BODY:
//                vrelpoint = Vadd(restpos, vmotion);             // Uo1
//                vpoint = my_body->Point_Body2World(vrelpoint);  // Uw = Xo1+[A]Uo1
//                break;
//        }
//    }
//
//    void FrForce::UpdateChronoForce() {
//
//        double modforce;
//        auto my_body = GetBody();
//        auto vectforce = chrono::VNULL;
//        auto xyzforce = chrono::VNULL;
//
//        modforce = mforce * modula->Get_y(ChTime);
//
//        if (f_x)
//            xyzforce.x() = f_x->Get_y(ChTime);
//        if (f_y)
//            xyzforce.y() = f_y->Get_y(ChTime);
//        if (f_z)
//            xyzforce.z() = f_z->Get_y(ChTime);
//
//        switch (align) {
//            case WORLD_DIR:
//                vreldir = my_body->TransformDirectionParentToLocal(vdir);
//                vectforce = Vmul(vdir, modforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//            case BODY_DIR:
//                vdir = my_body->TransformDirectionLocalToParent(vreldir);
//                vectforce = Vmul(vdir, modforce);
//                xyzforce = my_body->TransformDirectionLocalToParent(xyzforce);
//                vectforce = Vadd(vectforce, xyzforce);
//                break;
//        }
//
//        force += vectforce;                                           // Fw
//        relforce = my_body->TransformDirectionParentToLocal(force);  // Fo1 = [A]'Fw
//    }

    void FrForce::SetLog() {

        if (m_logPrefix == "") { SetLogPrefix(); }  // Set default log prefix if not defined

        //m_log.AddField("time","s","Current time of the simulation",&ChTime);
        m_log.AddField(m_logPrefix + "FX", "N", "Force in x-direction", &force.x());
        m_log.AddField(m_logPrefix + "FY", "N", "Force in y-direction", &force.y());
        m_log.AddField(m_logPrefix + "FZ", "N", "Force in z-direction", &force.z());
        m_log.AddField(m_logPrefix + "MX", "N.m", "Moment along x-direction", &moment.x());
        m_log.AddField(m_logPrefix + "MY", "N.m", "Moment along y-direction", &moment.y());
        m_log.AddField(m_logPrefix + "MZ", "N.m", "Moment along z-direction", &moment.z());

        m_log.AddCSVSerializer();
    }

    void FrForce::InitializeLogs() {

        m_log.Initialize();
        m_log.Send();
    }

    void FrForce::UpdateLogs() {
        m_log.Serialize();
        m_log.Send();
    }





















    // REFACTORING -------------6>>>>>>>>>>>>>>>>>

    namespace internal {

        _FrForceBase::_FrForceBase(FrForce_ *force) : m_frydomForce(force) {}

        void _FrForceBase::UpdateState() {

            // Calling the FRyDoM interface for Update
            m_frydomForce->Update(ChTime);

            // Limitation of the force and torque
            if (m_frydomForce->GetLimit()) {

                double limit = m_frydomForce->GetMaxForceLimit();
                double magn = force.Length();
                if (magn > limit) {
                    force *= limit / magn;
                }

                limit = m_frydomForce->GetMaxTorqueLimit();
                magn = m_torque.Length();
                if (magn > limit) {
                    m_torque *= limit / magn;
                }
            }
        }

        void _FrForceBase::GetBodyForceTorque(chrono::ChVector<double> &body_force,
                                              chrono::ChVector<double> &body_torque) const {
            body_force = force;    // In absolute coordinates
            body_torque = m_torque; // In body coordinates expressed at COG
        }

        void _FrForceBase::GetForceInWorldNWU(Force &body_force) const {
            body_force = internal::ChVectorToVector3d<Force>(force);
        }

        void _FrForceBase::GetTorqueInBodyNWU(Torque &body_torque) const {
            body_torque = internal::ChVectorToVector3d<Torque>(m_torque);
        }

        void _FrForceBase::SetForceInWorldNWU(const Force &body_force) {
            force = internal::Vector3dToChVector(body_force);
        }

        void _FrForceBase::SetTorqueInBodyNWU(const Torque &body_torque) {
            m_torque = internal::Vector3dToChVector(body_torque);
        }

    }  // end namespace internal


    // FrForce_ methods implementations

    FrForce_::FrForce_() {
        m_chronoForce = std::make_shared<internal::_FrForceBase>(this);
    }

//    FrForce_::~FrForce_(){
//        m_forceAsset=nullptr;
//    }

    void FrForce_::Initialize() {
        if (m_isForceAsset) {
            assert(m_forceAsset==nullptr);
            auto ForceAsset = std::make_shared<FrForceAsset_>(this);
            m_forceAsset = ForceAsset.get();
            m_body->AddAsset(ForceAsset);
        }
    }

    std::shared_ptr<chrono::ChForce> FrForce_::GetChronoForce() {
        return m_chronoForce;
    }

    FrOffshoreSystem_* FrForce_::GetSystem() {
        return m_body->GetSystem();
    }


    bool FrForce_::IsForceAsset() {
        return m_isForceAsset;
    }

    void FrForce_::SetIsForceAsset(bool isAsset) {
        m_isForceAsset = isAsset;
    }

    void FrForce_::SetMaxForceLimit(double fmax) {
        m_forceLimit = fmax;
    }

    double FrForce_::GetMaxForceLimit() const {
        return m_forceLimit;
    }

    void FrForce_::SetMaxTorqueLimit(double tmax) {
        m_torqueLimit = tmax;
    }

    double FrForce_::GetMaxTorqueLimit() const {
        return m_torqueLimit;
    }

    void FrForce_::SetLimit(bool val) {
        m_limitForce = val;
    }

    bool FrForce_::GetLimit() const {
        return m_limitForce;
    }

    Position FrForce_::GetForceApplicationPointInWorld(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVpoint());
    }

    Position FrForce_::GetForceApplicationPointInBody(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVrelpoint());
    }

    void FrForce_::GetForceInWorld(Force &force, FRAME_CONVENTION fc) const {
        m_chronoForce->GetForceInWorldNWU(force);  // NWU

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(force);
        }
    }

    Force FrForce_::GetForceInWorld(FRAME_CONVENTION fc) const {
        Force force;
        GetForceInWorld(force, fc);
        return force;
    }

    void FrForce_::GetForceInWorld(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetForceInWorld(fc);
        fx = force[0];
        fy = force[1];
        fz = force[2];
    }

    void FrForce_::GetForceInBody(Force &force, FRAME_CONVENTION fc) const {
        GetForceInWorld(force, fc);
        m_body->ProjectVectorInBody<Force>(force, fc);
    }

    Force FrForce_::GetForceInBody(FRAME_CONVENTION fc) const {
        Force force;
        GetForceInBody(force, fc);
        return force;
    }

    void FrForce_::GetForceInBody(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetForceInBody(fc);
        fx = force[0];
        fy = force[1];
        fz = force[2];
    }

    void FrForce_::GetTorqueInWorldAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
        GetTorqueInBodyAtCOG(torque, fc);
        m_body->ProjectVectorInWorld<Torque>(torque, fc);
    }

    Torque FrForce_::GetTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
        Torque torque;
        GetTorqueInWorldAtCOG(torque, fc);
        return torque;
    }

    void FrForce_::GetTorqueInWorldAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        Torque torque = GetTorqueInWorldAtCOG(fc);
        mx = torque[0];
        my = torque[1];
        mz = torque[2];
    }

    void FrForce_::GetTorqueInBodyAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
        m_chronoForce->GetTorqueInBodyNWU(torque);

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Torque>(torque);
        }
    }

    Torque FrForce_::GetTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
        Torque torque;
        GetTorqueInBodyAtCOG(torque, fc);
        return torque;
    }

    void FrForce_::GetTorqueInBodyAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        Torque torque = GetTorqueInBodyAtCOG(fc);
        mx = torque[0];
        my = torque[1];
        mz = torque[2];
    }

    double FrForce_::GetForceNorm() const {
        return GetForceInWorld(NWU).norm();
    }

    double FrForce_::GetTorqueNormAtCOG() const {
        return GetTorqueInBodyAtCOG(NWU).norm();
    }

    // =================================================================================================================
    // Protected methods implementations
    // =================================================================================================================

    void FrForce_::SetForceInWorldAtCOG(const Force &worldForce, FRAME_CONVENTION fc) {
        auto forceTmp = worldForce;
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(forceTmp);  // In NWU
        }

        m_chronoForce->SetForceInWorldNWU(forceTmp);
    }

    void FrForce_::SetForceInWorldAtPointInBody(const Force &worldForce, const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(worldForce, fc);

        // Calculating the moment created by the force applied at point bodyPos
        Position GP = bodyPos - m_body->GetCOG(fc); // In body coordinates following the fc convention

        Torque body_torque = GP.cross(m_body->ProjectVectorInBody<Force>(worldForce, fc));

        SetTorqueInBodyAtCOG(body_torque, fc);
    }

    void FrForce_::SetForceInWorldAtPointInWorld(const Force &worldForce, const Position &worldPos, FRAME_CONVENTION fc) {
        // Getting the local position of the point
        Position bodyPos = m_body->GetPointPositionInBody(worldPos, fc);
        SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
    }

    void FrForce_::SetForceInBody(const Force &bodyForce, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), fc);
    }

    void FrForce_::SetForceInBodyAtPointInBody(const Force& bodyForce, const Position& bodyPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInBody(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), bodyPos, fc);
    }

    void FrForce_::SetForceInBodyAtPointInWorld(const Force& bodyForce, const Position& worldPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInWorld(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), worldPos, fc);
    }

    void FrForce_::SetTorqueInWorldAtCOG(const Torque& worldTorque, FRAME_CONVENTION fc) {
        SetTorqueInBodyAtCOG(m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce_::SetTorqueInBodyAtCOG(const Torque& bodyTorque, FRAME_CONVENTION fc) {
        auto torqueTmp = bodyTorque;
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Torque>(torqueTmp);  // In NWU
        }

        m_chronoForce->SetTorqueInBodyNWU(torqueTmp);
    }


    void FrForce_::SetForceTorqueInWorldAtCOG(const Force& worldForce, const Torque& worldTorque, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(worldForce, fc);
        SetTorqueInWorldAtCOG(worldTorque, fc);
    }

    void FrForce_::SetForceTorqueInBodyAtCOG(const Force& bodyForce, const Torque& bodyTorque, FRAME_CONVENTION fc) {
        SetForceInBody(bodyForce, fc);
        SetTorqueInBodyAtCOG(bodyTorque, fc);
    }

    void FrForce_::SetForceTorqueInWorldAtPointInBody(const Force &worldForce, const Torque &worldTorque,
                                                      const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce_::SetForceTorqueInWorldAtPointInWorld(const Force &worldForce, const Torque &worldTorque,
                                                       const Position &worldPoint, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInWorld(worldForce, worldPoint, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce_::SetForceTorqueInBodyAtPointInBody(const Force &bodyForce, const Torque &bodyTorque,
                                                     const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInBodyAtPointInBody(bodyForce, bodyPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }

    void FrForce_::SetForceTorqueInBodyAtPointInWorld(const Force &bodyForce, const Torque &bodyTorque,
                                                      const Position &worldPos, FRAME_CONVENTION fc) {
        SetForceInBodyAtPointInWorld(bodyForce, worldPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }


}  // end namespace frydom
