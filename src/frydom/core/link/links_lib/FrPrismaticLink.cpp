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

#include "frydom/logging/FrTypeNames.h"

namespace frydom {

  FrPrismaticLink::FrPrismaticLink(const std::string &name,
                                   FrOffshoreSystem *system,
                                   std::shared_ptr<frydom::FrNode> node1,
                                   std::shared_ptr<frydom::FrNode> node2) :
      FrLink(name, TypeToString(this), system, node1, node2) {

    m_chronoLink->SetLinkType(PRISMATIC);
  }

  void FrPrismaticLink::SetSpringDamper(double stiffness, double damping) {
    m_stiffness = stiffness;
    m_damping = damping;
  }

  void FrPrismaticLink::SetRestLength(double restLength) {
    m_frame2WRT1_reference.SetZ(restLength, NWU);
    UpdateCache();
  }

  double FrPrismaticLink::GetRestLength() const {
    return m_restLength;
  }

  const Direction FrPrismaticLink::GetLinkDirectionInWorld(FRAME_CONVENTION fc) const {
    return GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
  }

  double FrPrismaticLink::GetLinkPosition() const {
    return m_linkPosition - m_restLength;
  }

  double FrPrismaticLink::GetLinkVelocity() const {
    return m_linkVelocity;
  }

  double FrPrismaticLink::GetLinkAcceleration() const {
    return m_linkAcceleration;
  }

  double FrPrismaticLink::GetLinkForce() const {
    return GetSpringDamperForceOnNode2(NWU).GetFz();
  }

  double FrPrismaticLink::GetLinkPower() const {
    return GetLinkVelocity() * GetLinkForce();
  }

  void FrPrismaticLink::Update(double time) {
    FrLink::Update(time); // It is mandatory to invoke this before all update operations from frydom

    // Update total link measure
    m_linkPosition = GetNode2PositionWRTNode1(NWU).GetZ();
    m_linkVelocity = GetVelocityOfNode2WRTNode1(NWU).GetVz();
    m_linkAcceleration = GetAccelerationOfNode2WRTNode1(NWU).GetAccZ();

    UpdateForces(time);

  }

  void FrPrismaticLink::UpdateForces(double time) {

    if (IsMotorized()) return;

    Force force;
    Torque torque;
    force.GetFz() = m_stiffness * GetLinkPosition() + m_damping * GetLinkVelocity();

    SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
  }

  void FrPrismaticLink::UpdateCache() {
    m_restLength = m_frame2WRT1_reference.GetZ(NWU);
    // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
  }

  FrLinearActuator *FrPrismaticLink::Motorize(const std::string &name, ACTUATOR_CONTROL control) {

    m_actuator = std::make_shared<FrLinearActuator>(name, this, control);
    GetSystem()->Add(m_actuator);
    return dynamic_cast<FrLinearActuator *>(m_actuator.get());

  }

  void FrPrismaticLink::Clamp(const std::string &name) {

    if (IsMotorized()) GetSystem()->Remove(m_actuator);

    // brake motorization instantiation
    m_actuator = std::make_shared<FrLinearActuator>(name, this, POSITION);
    m_actuator->Initialize();
    GetSystem()->Add(m_actuator);

    m_actuator->SetMotorFunction(FrConstantFunction(GetNode2PositionWRTNode1(NWU).GetZ()));

  }

  std::shared_ptr<FrPrismaticLink>
  make_prismatic_link(const std::string &name,
                      FrOffshoreSystem *system,
                      std::shared_ptr<FrNode> node1,
                      std::shared_ptr<FrNode> node2) {

    auto link = std::make_shared<FrPrismaticLink>(name, system, node1, node2);
    system->Add(link);

    return link;
  }


}  // end namespace frydom
