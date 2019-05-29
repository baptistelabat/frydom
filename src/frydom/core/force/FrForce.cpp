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

#include "frydom/asset/FrForceAsset.h"

#include "frydom/IO/FrPathManager.h"

#include "frydom/utils/FrSerializerFactory.h"


namespace frydom{

    namespace internal {

        FrForceBase::FrForceBase(FrForce *force) : m_frydomForce(force) {}

        void FrForceBase::UpdateState() {

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

        void FrForceBase::GetBodyForceTorque(chrono::ChVector<double> &body_force,
                                              chrono::ChVector<double> &body_torque) const {
            body_force = force;    // In absolute coordinates
            body_torque = m_torque; // In body coordinates expressed at COG
        }

        void FrForceBase::GetForceInWorldNWU(Force &body_force) const {
            body_force = internal::ChVectorToVector3d<Force>(force);
        }

        void FrForceBase::GetTorqueInBodyNWU(Torque &body_torque) const {
            body_torque = internal::ChVectorToVector3d<Torque>(m_torque);
        }

        void FrForceBase::SetForceInWorldNWU(const Force &body_force) {
            force = internal::Vector3dToChVector(body_force);
        }

        void FrForceBase::SetTorqueInBodyNWU(const Torque &body_torque) {
            m_torque = internal::Vector3dToChVector(body_torque);
        }

    }  // end namespace frydom::internal


    // FrForce methods implementations

    FrForce::FrForce() {

        m_chronoForce = std::make_shared<internal::FrForceBase>(this);
        SetLogged(true);
        
    }

    void FrForce::Initialize() {

        // This subroutine initializes the object FrForce.

        if (m_showAsset) {
            m_asset->Initialize();
            m_body->AddAsset(m_asset);
        }
    }

    std::shared_ptr<chrono::ChForce> FrForce::GetChronoForce() {
        return m_chronoForce;
    }

    FrOffshoreSystem* FrForce::GetSystem() {
        return m_body->GetSystem();
    }

    void FrForce::AddFields() {

        if (IsLogged()) {

            // Add the fields to be logged
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return m_chronoForce->GetChTime(); });

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInBody","N", fmt::format("force in body reference frame in {}", GetLogFrameConvention()),
                    [this]() {return GetForceInBody(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInBodyAtCOG","Nm", fmt::format("torque at COG in body reference frame in {}", GetLogFrameConvention()),
                    [this]() {return GetTorqueInBodyAtCOG(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInWorld","N", fmt::format("force in world reference frame in {}", GetLogFrameConvention()),
                    [this]() {return GetForceInWorld(GetLogFrameConvention());});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInWorldAtCOG","Nm", fmt::format("torque at COG in world reference frame in {}", GetLogFrameConvention()),
                    [this]() {return GetTorqueInWorldAtCOG(GetLogFrameConvention());});

        }
    }


    bool FrForce::IsForceAsset() {
        return m_showAsset;
    }

    void FrForce::ShowAsset(bool isAsset) {
        m_showAsset = isAsset;
        if (isAsset) {
            assert(m_asset==nullptr);
            m_asset = std::make_shared<FrForceAsset>(this);
        }
    }

    void FrForce::SetMaxForceLimit(double fmax) {
        m_forceLimit = fmax;
    }

    double FrForce::GetMaxForceLimit() const {
        return m_forceLimit;
    }

    void FrForce::SetMaxTorqueLimit(double tmax) {
        m_torqueLimit = tmax;
    }

    double FrForce::GetMaxTorqueLimit() const {
        return m_torqueLimit;
    }

    void FrForce::SetLimit(bool val) {
        m_limitForce = val;
    }

    bool FrForce::GetLimit() const {
        return m_limitForce;
    }

    Position FrForce::GetForceApplicationPointInWorld(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVpoint());
    }

    Position FrForce::GetForceApplicationPointInBody(FRAME_CONVENTION fc) const {
        return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVrelpoint());
    }

    void FrForce::GetForceInWorld(Force &force, FRAME_CONVENTION fc) const {
        m_chronoForce->GetForceInWorldNWU(force);  // NWU

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(force);
        }
    }

    Force FrForce::GetForceInWorld(FRAME_CONVENTION fc) const {
        Force force;
        GetForceInWorld(force, fc);
        return force;
    }

    void FrForce::GetForceInWorld(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetForceInWorld(fc);
        fx = force[0];
        fy = force[1];
        fz = force[2];
    }

    void FrForce::GetForceInBody(Force &force, FRAME_CONVENTION fc) const {
        GetForceInWorld(force, fc);
        m_body->ProjectVectorInBody<Force>(force, fc);
    }

    Force FrForce::GetForceInBody(FRAME_CONVENTION fc) const {
        Force force;
        GetForceInBody(force, fc);
        return force;
    }

    void FrForce::GetForceInBody(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
        auto force = GetForceInBody(fc);
        fx = force[0];
        fy = force[1];
        fz = force[2];
    }

    void FrForce::GetTorqueInWorldAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
        GetTorqueInBodyAtCOG(torque, fc);
        m_body->ProjectVectorInWorld<Torque>(torque, fc);
    }

    Torque FrForce::GetTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
        Torque torque;
        GetTorqueInWorldAtCOG(torque, fc);
        return torque;
    }

    void FrForce::GetTorqueInWorldAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        Torque torque = GetTorqueInWorldAtCOG(fc);
        mx = torque[0];
        my = torque[1];
        mz = torque[2];
    }

    void FrForce::GetTorqueInBodyAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
        m_chronoForce->GetTorqueInBodyNWU(torque);

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Torque>(torque);
        }
    }

    Torque FrForce::GetTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
        Torque torque;
        GetTorqueInBodyAtCOG(torque, fc);
        return torque;
    }

    void FrForce::GetTorqueInBodyAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
        Torque torque = GetTorqueInBodyAtCOG(fc);
        mx = torque[0];
        my = torque[1];
        mz = torque[2];
    }

    double FrForce::GetForceNorm() const {
        return GetForceInWorld(NWU).norm();
    }

    double FrForce::GetTorqueNormAtCOG() const {
        return GetTorqueInBodyAtCOG(NWU).norm();
    }

    // =================================================================================================================
    // Protected methods implementations
    // =================================================================================================================

    void FrForce::SetForceInWorldAtCOG(const Force &worldForce, FRAME_CONVENTION fc) {

        /// This subroutine sets a force expressed in the world at the CoG body in Chrono.

        auto forceTmp = worldForce;

        // Transformation if not in NWU.
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Force>(forceTmp);  // In NWU
        }

        m_chronoForce->SetForceInWorldNWU(forceTmp);
    }

    void FrForce::SetForceInWorldAtPointInBody(const Force &worldForce, const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(worldForce, fc);

        // Calculating the moment created by the force applied at point bodyPos
        Position GP = bodyPos - m_body->GetCOG(fc); // In body coordinates following the fc convention

        Torque body_torque = GP.cross(m_body->ProjectVectorInBody<Force>(worldForce, fc));

        SetTorqueInBodyAtCOG(body_torque, fc);
    }

    void FrForce::SetForceInWorldAtPointInWorld(const Force &worldForce, const Position &worldPos, FRAME_CONVENTION fc) {
        // Getting the local position of the point
        Position bodyPos = m_body->GetPointPositionInBody(worldPos, fc);
        SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
    }

    void FrForce::SetForceInBody(const Force &bodyForce, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), fc);
    }

    void FrForce::SetForceInBodyAtPointInBody(const Force& bodyForce, const Position& bodyPos, FRAME_CONVENTION fc) {

        SetForceInWorldAtPointInBody(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), bodyPos, fc);
    }

    void FrForce::SetForceInBodyAtPointInWorld(const Force& bodyForce, const Position& worldPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInWorld(m_body->ProjectVectorInWorld<Force>(bodyForce, fc), worldPos, fc);
    }

    void FrForce::SetTorqueInWorldAtCOG(const Torque& worldTorque, FRAME_CONVENTION fc) {
        SetTorqueInBodyAtCOG(m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce::SetTorqueInBodyAtCOG(const Torque& bodyTorque, FRAME_CONVENTION fc) {

        /// This subroutine sets a torque expressed in the world at the CoG body in Chrono.

        auto torqueTmp = bodyTorque;

        // Transformation if not in NWU.
        if (IsNED(fc)) {
            internal::SwapFrameConvention<Torque>(torqueTmp);  // In NWU
        }

        m_chronoForce->SetTorqueInBodyNWU(torqueTmp);
    }


    void FrForce::SetForceTorqueInWorldAtCOG(const Force& worldForce, const Torque& worldTorque, FRAME_CONVENTION fc) {
        SetForceInWorldAtCOG(worldForce, fc);
        SetTorqueInWorldAtCOG(worldTorque, fc);
    }

    void FrForce::SetForceTorqueInBodyAtCOG(const Force& bodyForce, const Torque& bodyTorque, FRAME_CONVENTION fc) {
        SetForceInBody(bodyForce, fc);
        SetTorqueInBodyAtCOG(bodyTorque, fc);
    }

    void FrForce::SetForceTorqueInWorldAtPointInBody(const Force &worldForce, const Torque &worldTorque,
                                                      const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce::SetForceTorqueInWorldAtPointInWorld(const Force &worldForce, const Torque &worldTorque,
                                                       const Position &worldPoint, FRAME_CONVENTION fc) {
        SetForceInWorldAtPointInWorld(worldForce, worldPoint, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    void FrForce::SetForceTorqueInBodyAtPointInBody(const Force &bodyForce, const Torque &bodyTorque,
                                                     const Position &bodyPos, FRAME_CONVENTION fc) {
        SetForceInBodyAtPointInBody(bodyForce, bodyPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }

    void FrForce::SetForceTorqueInBodyAtPointInWorld(const Force &bodyForce, const Torque &bodyTorque,
                                                      const Position &worldPos, FRAME_CONVENTION fc) {
        SetForceInBodyAtPointInWorld(bodyForce, worldPos, fc);
        SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }

    FrBody *FrForce::GetBody() const {
        return m_body;
    }

    FrForceAsset *FrForce::GetAsset() {
        return m_asset.get();
    }

    bool FrForce::IsActive() const {return m_isActive;}

    void FrForce::SetActive(bool active) { m_isActive = active;}

    void FrForce::Update(double time) {

        if (IsActive())
            Compute(time);
        else
            SetForceTorqueInBodyAtCOG(Force(), Torque(), NWU);

    }

    std::string FrForce::BuildPath(const std::string &rootPath) {

        auto objPath = fmt::format("{}/Forces", rootPath);

        auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

        // Add a serializer
        m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));

        return objPath;

    }


}  // end namespace frydom
