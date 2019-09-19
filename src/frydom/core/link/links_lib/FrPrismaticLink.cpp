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

#include "FrPrismaticLink.h"

#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/functions/FrFunctionsInc.h"

#include "frydom/core/link/links_lib/actuators/FrLinearActuator.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrPrismaticLink::FrPrismaticLink(std::shared_ptr<frydom::FrNode> node1, std::shared_ptr<frydom::FrNode> node2,
                                     frydom::FrOffshoreSystem *system) : FrLink(node1, node2, system) {
      m_chronoLink->SetLinkType(PRISMATIC);
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::SetSpringDamper(double stiffness, double damping) {
      m_stiffness = stiffness;
      m_damping = damping;
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::SetRestLength(double restLength) {
      m_frame2WRT1_reference.SetZ(restLength, NWU);
      UpdateCache();
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetRestLength() const {
      return m_restLength;
    }

    template<typename OffshoreSystemType>
    const Direction FrPrismaticLink::GetLinkDirectionInWorld(FRAME_CONVENTION fc) const {
      return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetLinkPosition() const {
      return m_linkPosition - m_restLength;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetLinkVelocity() const {
      return m_linkVelocity;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetLinkAcceleration() const {
      return m_linkAcceleration;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetLinkForce() const {
      return GetLinkForceOnBody2InFrame2AtOrigin2(NWU).GetFz();
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink::GetLinkPower() const {
      return GetLinkVelocity() * GetLinkForce();
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::Update(double time) {
      FrLink::Update(time); // It is mandatory to invoke this before all update operations from frydom

      // Update total link measure
      m_linkPosition = GetNode2PositionWRTNode1(NWU).GetZ();
      m_linkVelocity = GetVelocityOfNode2WRTNode1(NWU).GetVz();
      m_linkAcceleration = GetAccelerationOfNode2WRTNode1(NWU).GetAccZ();

      UpdateForces(time);

    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::UpdateForces(double time) {

      if (IsMotorized()) return;

      Force force;
      Torque torque;
      force.GetFz() = -m_stiffness * GetLinkPosition() - m_damping * GetLinkVelocity();

      SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::UpdateCache() {
      m_restLength = m_frame2WRT1_reference.GetZ(NWU);
      // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    template<typename OffshoreSystemType>
    FrLinearActuator *FrPrismaticLink::Motorize(ACTUATOR_CONTROL control) {

      m_actuator = std::make_shared<FrLinearActuator>(this, control);
      GetSystem()->Add(m_actuator);
      return dynamic_cast<FrLinearActuator *>(m_actuator.get());

    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink::Clamp() {

      if (IsMotorized()) GetSystem()->RemoveLink(m_actuator);

      // brake motorization instantiation
      m_actuator = std::make_shared<FrLinearActuator>(this, POSITION);
      m_actuator->Initialize();
      GetSystem()->Add(m_actuator);

      m_actuator->SetMotorFunction(FrConstantFunction(GetNode2PositionWRTNode1(NWU).GetZ()));

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrPrismaticLink>
    make_prismatic_link(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2, FrOffshoreSystem *system) {
      auto link = std::make_shared<FrPrismaticLink>(node1, node2, system);
      system->AddLink(link);


      return link;
    }


}  // end namespace frydom
