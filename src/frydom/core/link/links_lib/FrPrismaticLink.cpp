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
    FrPrismaticLink<OffshoreSystemType>::FrPrismaticLink(std::shared_ptr<frydom::FrNode<OffshoreSystemType>> node1,
                                                         std::shared_ptr<frydom::FrNode<OffshoreSystemType>> node2,
                                                         frydom::FrOffshoreSystem<OffshoreSystemType> *system)
        : FrLink<OffshoreSystemType>(node1, node2, system) {
      this->m_chronoLink->SetLinkType(PRISMATIC);
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::SetSpringDamper(double stiffness, double damping) {
      m_stiffness = stiffness;
      m_damping = damping;
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::SetRestLength(double restLength) {
      this->m_frame2WRT1_reference.SetZ(restLength, NWU);
      UpdateCache();
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetRestLength() const {
      return m_restLength;
    }

    template<typename OffshoreSystemType>
    const Direction FrPrismaticLink<OffshoreSystemType>::GetLinkDirectionInWorld(FRAME_CONVENTION fc) const {
      return this->GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetLinkPosition() const {
      return m_linkPosition - m_restLength;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetLinkVelocity() const {
      return m_linkVelocity;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetLinkAcceleration() const {
      return m_linkAcceleration;
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetLinkForce() const {
      return this->GetLinkForceOnBody2InFrame2AtOrigin2(NWU).GetFz();
    }

    template<typename OffshoreSystemType>
    double FrPrismaticLink<OffshoreSystemType>::GetLinkPower() const {
      return GetLinkVelocity() * GetLinkForce();
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::Update(double time) {
      FrLink<OffshoreSystemType>::Update(time); // It is mandatory to invoke this before all update operations from frydom

      // Update total link measure
      m_linkPosition = this->GetNode2PositionWRTNode1(NWU).GetZ();
      m_linkVelocity = this->GetVelocityOfNode2WRTNode1(NWU).GetVz();
      m_linkAcceleration = this->GetAccelerationOfNode2WRTNode1(NWU).GetAccZ();

      UpdateForces(time);

    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::UpdateForces(double time) {

      if (this->IsMotorized()) return;

      Force force;
      Torque torque;
      force.GetFz() = -m_stiffness * GetLinkPosition() - m_damping * GetLinkVelocity();

      this->SetLinkForceTorqueOnBody2InFrame2AtOrigin2(force, torque);
    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::UpdateCache() {
      m_restLength = this->m_frame2WRT1_reference.GetZ(NWU);
      // FIXME : attention si la liaison n'est pas resolue !!! Ca ne fonctionne pas
    }

    template<typename OffshoreSystemType>
    FrLinearActuator<OffshoreSystemType> *FrPrismaticLink<OffshoreSystemType>::Motorize(ACTUATOR_CONTROL control) {

      this->m_actuator = std::make_shared<FrLinearActuator>(this, control);
      this->GetSystem()->Add(this->m_actuator);
      return dynamic_cast<FrLinearActuator<OffshoreSystemType> *>(this->m_actuator.get());

    }

    template<typename OffshoreSystemType>
    void FrPrismaticLink<OffshoreSystemType>::Clamp() {

      if (this->IsMotorized()) this->GetSystem()->RemoveLink(this->m_actuator);

      // brake motorization instantiation
      this->m_actuator = std::make_shared<FrLinearActuator>(this, POSITION);
      this->m_actuator->Initialize();
      this->GetSystem()->Add(this->m_actuator);

      this->m_actuator->SetMotorFunction(FrConstantFunction(this->GetNode2PositionWRTNode1(NWU).GetZ()));

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrPrismaticLink<OffshoreSystemType>>
    make_prismatic_link(std::shared_ptr<FrNode<OffshoreSystemType>> node1,
                        std::shared_ptr<FrNode<OffshoreSystemType>> node2, FrOffshoreSystem<OffshoreSystemType>* system) {
      auto link = std::make_shared<FrPrismaticLink>(node1, node2, system);
      system->AddLink(link);


      return link;
    }


}  // end namespace frydom
