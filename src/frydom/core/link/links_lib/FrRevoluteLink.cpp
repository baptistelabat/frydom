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


#include "FrRevoluteLink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

#include "frydom/core/link/links_lib/actuators/FrAngularActuator.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrRevoluteLink<OffshoreSystemType>::FrRevoluteLink(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1, const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                                   FrOffshoreSystem<OffshoreSystemType> *system) : FrLink<OffshoreSystemType>(node1, node2, system) {
      this->m_chronoLink->SetLinkType(REVOLUTE);
    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::SetSpringDamper(double stiffness, double damping) {
      m_stiffness = stiffness;
      m_damping = damping;
    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::SetRestAngle(double restAngle) {
      this->m_frame2WRT1_reference.SetRotZ_RADIANS(restAngle, NWU);
      UpdateCache();
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetRestAngle() const {
      return m_restAngle;
    }

    template<typename OffshoreSystemType>
    const Direction FrRevoluteLink<OffshoreSystemType>::GetLinkAxisInWorld(FRAME_CONVENTION fc) const {
      return this->GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetLinkAngle() const {
      return m_totalLinkAngle - m_restAngle;
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetRelativeLinkAngle() const {
      return fmod(m_totalLinkAngle, MU_2PI) - m_restAngle;
    }

    template<typename OffshoreSystemType>
    int FrRevoluteLink<OffshoreSystemType>::GetNbTurns() const {
      return int((GetLinkAngle() - GetRelativeLinkAngle()) / MU_2PI);
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetLinkAngularVelocity() const {
      return m_linkAngularVelocity;
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetLinkAngularAcceleration() const {
      return m_linkAngularAcceleration;
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetLinkTorque() const {
      return this->GetLinkTorqueOnBody2InFrame2AtOrigin2(NWU).GetMz();
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetLinkPower() const {
      return GetLinkAngularVelocity() * GetLinkTorque();
    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::Initialize() {
      // Initialization of the constraint part
      FrLink<OffshoreSystemType>::Initialize();

      // Initialization of the motor part
      if (this->m_actuator) {
        this->m_actuator->Initialize();
      }

    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::Update(double time) {

      FrLink<OffshoreSystemType>::Update(time);

      double lastRelativeAngle = GetRelativeLinkAngle() + m_restAngle; // Making it relative to x, not the rest angle
      double updatedRelativeAngle = GetUpdatedRelativeAngle();

      // TODO : voir a definir un RotationVector dans FrVector...
      // Computing the angle increment between current relative angle and the last relative angle to increment
      // the total link angle
      double angleIncrement;
      if (fabs(updatedRelativeAngle + MU_2PI - lastRelativeAngle) < fabs(updatedRelativeAngle - lastRelativeAngle)) {
        angleIncrement = updatedRelativeAngle + MU_2PI - lastRelativeAngle;
      } else if (fabs(updatedRelativeAngle - MU_2PI - lastRelativeAngle) <
                 fabs(updatedRelativeAngle - lastRelativeAngle)) {
        angleIncrement = updatedRelativeAngle - MU_2PI - lastRelativeAngle;
      } else {
        angleIncrement = updatedRelativeAngle - lastRelativeAngle;
      }

      m_totalLinkAngle += angleIncrement;

      m_linkAngularVelocity = this->GetAngularVelocityOfNode2WRTNode1(NWU).GetWz();
      m_linkAngularAcceleration = this->GetAngularAccelerationOfNode2WRTNode1(NWU).GetWzp();

      UpdateForces(time);

    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::UpdateForces(double time) {

      if (this->IsMotorized()) return;

      // Default spring damper force model
      Force force;
      Torque torque;

      torque.GetMz() = -m_stiffness * GetLinkAngle() - m_damping * GetLinkAngularVelocity();

      // Set the link force
      this->SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    template<typename OffshoreSystemType>
    FrAngularActuator<OffshoreSystemType> *FrRevoluteLink<OffshoreSystemType>::Motorize(ACTUATOR_CONTROL control) {
      this->m_actuator = std::make_shared<FrAngularActuator<OffshoreSystemType>>(this, control);
      this->GetSystem()->Add(this->m_actuator);
      return dynamic_cast<FrAngularActuator<OffshoreSystemType> *>(this->m_actuator.get());
    }

    template<typename OffshoreSystemType>
    double FrRevoluteLink<OffshoreSystemType>::GetUpdatedRelativeAngle() const {
      return mathutils::Normalize__PI_PI(this->m_chronoLink->c_frame2WRT1.GetRotation().GetRotationVector(NWU)[2]);
//        return mathutils::Normalize__PI_PI(this->m_chronoLink->GetRelAngle()); // INFO : fonctionne bien moins bien que ci-dessus !
    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::UpdateCache() {
      // Updating the rest angle
      m_restAngle = mathutils::Normalize__PI_PI(this->m_frame2WRT1_reference.GetRotation().GetAngle());
      // TODO : ne pas prendre GetAngle mais la composante z de RotationVector

      // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrRevoluteLink<OffshoreSystemType>>
    make_revolute_link(std::shared_ptr<FrNode<OffshoreSystemType>> node1, std::shared_ptr<FrNode<OffshoreSystemType>> node2, FrOffshoreSystem<OffshoreSystemType> *system) {
      auto link = std::make_shared<FrRevoluteLink>(node1, node2, system);
      system->AddLink(link);
      return link;
    }

    template<typename OffshoreSystemType>
    void FrRevoluteLink<OffshoreSystemType>::Clamp() {

      if (this->IsMotorized()) this->GetSystem()->RemoveLink(this->m_actuator);

      // brake motorization instantiation
      this->m_actuator = std::make_shared<FrAngularActuator<OffshoreSystemType>>(this, POSITION);
      this->m_actuator->Initialize();
      this->GetSystem()->Add(this->m_actuator);

      auto angle = this->GetNode2OrientationWRTNode1().GetAngle();

      this->m_actuator->SetMotorFunction(FrConstantFunction(angle));

    }


}  // end namespace frydom
