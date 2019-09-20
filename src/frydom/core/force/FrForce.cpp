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


namespace frydom {

    namespace internal {

        template<typename OffshoreSystemType>
        FrForceBase<OffshoreSystemType>::FrForceBase(FrForce<OffshoreSystemType> *force) : m_frydomForce(force) {}

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::UpdateState() {

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

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::GetBodyForceTorque(chrono::ChVector<double> &body_force,
                                                                 chrono::ChVector<double> &body_torque) const {
          body_force = force;    // In absolute coordinates
          body_torque = m_torque; // In body coordinates expressed at COG
        }

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::GetForceInWorldNWU(Force &body_force) const {
          body_force = internal::ChVectorToVector3d<Force>(force);
        }

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::GetTorqueInBodyNWU(Torque &body_torque) const {
          body_torque = internal::ChVectorToVector3d<Torque>(m_torque);
        }

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::SetForceInWorldNWU(const Force &body_force) {
          force = internal::Vector3dToChVector(body_force);
        }

        template<typename OffshoreSystemType>
        void FrForceBase<OffshoreSystemType>::SetTorqueInBodyNWU(const Torque &body_torque) {
          m_torque = internal::Vector3dToChVector(body_torque);
        }

    }  // end namespace frydom::internal


    // FrForce methods implementations
    template<typename OffshoreSystemType>
    FrForce<OffshoreSystemType>::FrForce() {

      m_chronoForce = std::make_shared<internal::FrForceBase>(this);
      this->SetLogged(true);

    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::Initialize() {

      // This subroutine initializes the object FrForce.

      if (m_showAsset) {
        m_asset->Initialize();
        m_body->AddAsset(m_asset);
      }
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChForce> FrForce<OffshoreSystemType>::GetChronoForce() {
      return m_chronoForce;
    }

    template<typename OffshoreSystemType>
    FrOffshoreSystem<OffshoreSystemType> *FrForce<OffshoreSystemType>::GetSystem() {
      return m_body->GetSystem();
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::AddFields() {

      if (this->IsLogged()) {

        // Add the fields to be logged
        this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                                   [this]() { return m_chronoForce->GetChTime(); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInBody", "N", fmt::format("force in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInBody(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInBodyAtCOG", "Nm",
             fmt::format("torque at COG in body reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetTorqueInBodyAtCOG(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("ForceInWorld", "N", fmt::format("force in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetForceInWorld(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("TorqueInWorldAtCOG", "Nm",
             fmt::format("torque at COG in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetTorqueInWorldAtCOG(this->GetLogFrameConvention()); });

      }
    }

    template<typename OffshoreSystemType>
    bool FrForce<OffshoreSystemType>::IsForceAsset() {
      return m_showAsset;
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::ShowAsset(bool isAsset) {
      m_showAsset = isAsset;
      if (isAsset) {
        assert(m_asset == nullptr);
        m_asset = std::make_shared<FrForceAsset>(this);
      }
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetMaxForceLimit(double fmax) {
      m_forceLimit = fmax;
    }

    template<typename OffshoreSystemType>
    double FrForce<OffshoreSystemType>::GetMaxForceLimit() const {
      return m_forceLimit;
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetMaxTorqueLimit(double tmax) {
      m_torqueLimit = tmax;
    }

    template<typename OffshoreSystemType>
    double FrForce<OffshoreSystemType>::GetMaxTorqueLimit() const {
      return m_torqueLimit;
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetLimit(bool val) {
      m_limitForce = val;
    }

    template<typename OffshoreSystemType>
    bool FrForce<OffshoreSystemType>::GetLimit() const {
      return m_limitForce;
    }

    template<typename OffshoreSystemType>
    Position FrForce<OffshoreSystemType>::GetForceApplicationPointInWorld(FRAME_CONVENTION fc) const {
      return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVpoint());
    }

    template<typename OffshoreSystemType>
    Position FrForce<OffshoreSystemType>::GetForceApplicationPointInBody(FRAME_CONVENTION fc) const {
      return internal::ChVectorToVector3d<Position>(m_chronoForce->GetVrelpoint());
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetForceInWorld(Force &force, FRAME_CONVENTION fc) const {
      m_chronoForce->GetForceInWorldNWU(force);  // NWU

      if (IsNED(fc)) {
        internal::SwapFrameConvention<Force>(force);
      }
    }

    template<typename OffshoreSystemType>
    Force FrForce<OffshoreSystemType>::GetForceInWorld(FRAME_CONVENTION fc) const {
      Force force;
      GetForceInWorld(force, fc);
      return force;
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetForceInWorld(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
      auto force = GetForceInWorld(fc);
      fx = force[0];
      fy = force[1];
      fz = force[2];
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetForceInBody(Force &force, FRAME_CONVENTION fc) const {
      GetForceInWorld(force, fc);
      m_body->template ProjectVectorInBody<Force>(force, fc);
    }

    template<typename OffshoreSystemType>
    Force FrForce<OffshoreSystemType>::GetForceInBody(FRAME_CONVENTION fc) const {
      Force force;
      GetForceInBody(force, fc);
      return force;
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetForceInBody(double &fx, double &fy, double &fz, FRAME_CONVENTION fc) const {
      auto force = GetForceInBody(fc);
      fx = force[0];
      fy = force[1];
      fz = force[2];
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetTorqueInWorldAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
      GetTorqueInBodyAtCOG(torque, fc);
      m_body->template ProjectVectorInWorld<Torque>(torque, fc);
    }

    template<typename OffshoreSystemType>
    Torque FrForce<OffshoreSystemType>::GetTorqueInWorldAtCOG(FRAME_CONVENTION fc) const {
      Torque torque;
      GetTorqueInWorldAtCOG(torque, fc);
      return torque;
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::GetTorqueInWorldAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
      Torque torque = GetTorqueInWorldAtCOG(fc);
      mx = torque[0];
      my = torque[1];
      mz = torque[2];
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetTorqueInBodyAtCOG(Torque &torque, FRAME_CONVENTION fc) const {
      m_chronoForce->GetTorqueInBodyNWU(torque);

      if (IsNED(fc)) {
        internal::SwapFrameConvention<Torque>(torque);
      }
    }

    template<typename OffshoreSystemType>
    Torque FrForce<OffshoreSystemType>::GetTorqueInBodyAtCOG(FRAME_CONVENTION fc) const {
      Torque torque;
      GetTorqueInBodyAtCOG(torque, fc);
      return torque;
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::GetTorqueInBodyAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const {
      Torque torque = GetTorqueInBodyAtCOG(fc);
      mx = torque[0];
      my = torque[1];
      mz = torque[2];
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::GetTorqueInWorldAtPointInWorld(Torque &torque, const Position &worldPoint,
                                                                FRAME_CONVENTION fc) const {

      Position PG = m_body->GetCOGPositionInWorld(fc) - worldPoint;

      torque = GetTorqueInWorldAtCOG(fc) + PG.cross(GetForceInWorld(fc));

    }

    template<typename OffshoreSystemType>
    Torque
    FrForce<OffshoreSystemType>::GetTorqueInWorldAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {

      Torque torque;
      GetTorqueInWorldAtPointInWorld(torque, worldPoint, fc);
      return torque;

    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetTorqueInWorldAtPointInBody(Torque &torque, const Position &bodyPoint,
                                                                    FRAME_CONVENTION fc) const {

      GetTorqueInWorldAtPointInWorld(torque, m_body->GetPointPositionInWorld(bodyPoint, fc), fc);

    }

    template<typename OffshoreSystemType>
    Torque
    FrForce<OffshoreSystemType>::GetTorqueInWorldAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {

      Torque torque;
      GetTorqueInWorldAtPointInBody(torque, bodyPoint, fc);
      return torque;

    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetTorqueInBodyAtPointInWorld(Torque &torque, const Position &worldPoint,
                                                                    FRAME_CONVENTION fc) const {

      GetTorqueInWorldAtPointInWorld(torque, worldPoint, fc);
      m_body->ProjectVectorInBody(torque, fc);

    }

    template<typename OffshoreSystemType>
    Torque
    FrForce<OffshoreSystemType>::GetTorqueInBodyAtPointInWorld(const Position &worldPoint, FRAME_CONVENTION fc) const {

      Torque torque;
      GetTorqueInBodyAtPointInWorld(torque, worldPoint, fc);
      return torque;

    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::GetTorqueInBodyAtPointInBody(Torque &torque, const Position &bodyPoint,
                                                                   FRAME_CONVENTION fc) const {

      GetTorqueInWorldAtPointInBody(torque, bodyPoint, fc);
      m_body->ProjectVectorInBody(torque, fc);

    }

    template<typename OffshoreSystemType>
    Torque
    FrForce<OffshoreSystemType>::GetTorqueInBodyAtPointInBody(const Position &bodyPoint, FRAME_CONVENTION fc) const {

      Torque torque;
      GetTorqueInBodyAtPointInBody(torque, bodyPoint, fc);
      return torque;

    }

    template<typename OffshoreSystemType>
    double FrForce<OffshoreSystemType>::GetForceNorm() const {
      return GetForceInWorld(NWU).norm();
    }

    template<typename OffshoreSystemType>
    double FrForce<OffshoreSystemType>::GetTorqueNormAtCOG() const {
      return GetTorqueInBodyAtCOG(NWU).norm();
    }

    // =================================================================================================================
    // Protected methods implementations
    // =================================================================================================================
    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceInWorldAtCOG(const Force &worldForce, FRAME_CONVENTION fc) {

      /// This subroutine sets a force expressed in the world at the CoG body in Chrono.

      auto forceTmp = worldForce;

      // Transformation if not in NWU.
      if (IsNED(fc)) {
        internal::SwapFrameConvention<Force>(forceTmp);  // In NWU
      }

      m_chronoForce->SetForceInWorldNWU(forceTmp);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceInWorldAtPointInBody(const Force &worldForce, const Position &bodyPos,
                                                                   FRAME_CONVENTION fc) {
      SetForceInWorldAtCOG(worldForce, fc);

      // Calculating the moment created by the force applied at point bodyPos
      Position GP = bodyPos - m_body->GetCOG(fc); // In body coordinates following the fc convention

      Torque body_torque = GP.cross(m_body->template ProjectVectorInBody<Force>(worldForce, fc));

      SetTorqueInBodyAtCOG(body_torque, fc);
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::SetForceInWorldAtPointInWorld(const Force &worldForce, const Position &worldPos,
                                                               FRAME_CONVENTION fc) {
      // Getting the local position of the point
      Position bodyPos = m_body->GetPointPositionInBody(worldPos, fc);
      SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceInBody(const Force &bodyForce, FRAME_CONVENTION fc) {
      SetForceInWorldAtCOG(m_body->template ProjectVectorInWorld<Force>(bodyForce, fc), fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceInBodyAtPointInBody(const Force &bodyForce, const Position &bodyPos,
                                                                  FRAME_CONVENTION fc) {

      SetForceInWorldAtPointInBody(m_body->template ProjectVectorInWorld<Force>(bodyForce, fc), bodyPos, fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceInBodyAtPointInWorld(const Force &bodyForce, const Position &worldPos,
                                                                   FRAME_CONVENTION fc) {
      SetForceInWorldAtPointInWorld(m_body->template ProjectVectorInWorld<Force>(bodyForce, fc), worldPos, fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetTorqueInWorldAtCOG(const Torque &worldTorque, FRAME_CONVENTION fc) {
      SetTorqueInBodyAtCOG(m_body->template ProjectVectorInBody<Torque>(worldTorque, fc), fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetTorqueInBodyAtCOG(const Torque &bodyTorque, FRAME_CONVENTION fc) {

      /// This subroutine sets a torque expressed in the world at the CoG body in Chrono.

      auto torqueTmp = bodyTorque;

      // Transformation if not in NWU.
      if (IsNED(fc)) {
        internal::SwapFrameConvention<Torque>(torqueTmp);  // In NWU
      }

      m_chronoForce->SetTorqueInBodyNWU(torqueTmp);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceTorqueInWorldAtCOG(const Force &worldForce, const Torque &worldTorque,
                                                                 FRAME_CONVENTION fc) {
      SetForceInWorldAtCOG(worldForce, fc);
      SetTorqueInWorldAtCOG(worldTorque, fc);
    }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetForceTorqueInBodyAtCOG(const Force &bodyForce, const Torque &bodyTorque,
                                                                FRAME_CONVENTION fc) {
      SetForceInBody(bodyForce, fc);
      SetTorqueInBodyAtCOG(bodyTorque, fc);
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::SetForceTorqueInWorldAtPointInBody(const Force &worldForce, const Torque &worldTorque,
                                                                    const Position &bodyPos, FRAME_CONVENTION fc) {
      SetForceInWorldAtPointInBody(worldForce, bodyPos, fc);
      SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->template ProjectVectorInBody<Torque>(worldTorque, fc),
                           fc);
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::SetForceTorqueInWorldAtPointInWorld(const Force &worldForce, const Torque &worldTorque,
                                                                     const Position &worldPoint, FRAME_CONVENTION fc) {
      SetForceInWorldAtPointInWorld(worldForce, worldPoint, fc);
      SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + m_body->template ProjectVectorInBody<Torque>(worldTorque, fc),
                           fc);
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::SetForceTorqueInBodyAtPointInBody(const Force &bodyForce, const Torque &bodyTorque,
                                                                   const Position &bodyPos, FRAME_CONVENTION fc) {
      SetForceInBodyAtPointInBody(bodyForce, bodyPos, fc);
      SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }

    template<typename OffshoreSystemType>
    void
    FrForce<OffshoreSystemType>::SetForceTorqueInBodyAtPointInWorld(const Force &bodyForce, const Torque &bodyTorque,
                                                                    const Position &worldPos, FRAME_CONVENTION fc) {
      SetForceInBodyAtPointInWorld(bodyForce, worldPos, fc);
      SetTorqueInBodyAtCOG(GetTorqueInBodyAtCOG(fc) + bodyTorque, fc);
    }

    template<typename OffshoreSystemType>
    FrBody<OffshoreSystemType> *FrForce<OffshoreSystemType>::GetBody() const {
      return m_body;
    }

    template<typename OffshoreSystemType>
    FrForceAsset<OffshoreSystemType> *FrForce<OffshoreSystemType>::GetAsset() {
      return m_asset.get();
    }

    template<typename OffshoreSystemType>
    bool FrForce<OffshoreSystemType>::IsActive() const { return m_isActive; }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::SetActive(bool active) { m_isActive = active; }

    template<typename OffshoreSystemType>
    void FrForce<OffshoreSystemType>::Update(double time) {

      if (IsActive())
        Compute(time);
      else
        SetForceTorqueInBodyAtCOG(Force(), Torque(), NWU);

    }

    template<typename OffshoreSystemType>
    std::string FrForce<OffshoreSystemType>::BuildPath(const std::string &rootPath) {

      auto objPath = fmt::format("{}/Forces", rootPath);

      auto logPath = this->GetPathManager()->BuildPath(objPath,
                                                       fmt::format("{}_{}.csv", GetTypeName(), this->GetShortenUUID()));

      // Add a serializer
      this->m_message->AddSerializer(FrSerializerFactory<OffshoreSystemType>::instance().Create(this, logPath));

      return objPath;

    }


}  // end namespace frydom
