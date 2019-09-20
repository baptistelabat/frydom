//
// Created by lletourn on 07/05/19.
//

#include "FrAngularActuator.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"
#include "frydom/core/common/FrNode.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrAngularActuator<OffshoreSystemType>::FrAngularActuator(FrLink<OffshoreSystemType> *actuatedLink,
                                                             ACTUATOR_CONTROL control) : FrActuator<OffshoreSystemType>(
        actuatedLink) {

      switch (control) {
        case POSITION :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationAngle>();
          break;
        case VELOCITY :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationSpeed>();
          break;
        case FORCE :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorRotationTorque>();
          break;
      }
      m_chronoActuator->SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::FREE);

    }

    template<typename OffshoreSystemType>
    void FrAngularActuator<OffshoreSystemType>::SetMotorFunction(const FrFunctionBase &function) {

      auto chronoFunctionInterface = internal::FrFunctionChronoInterface<OffshoreSystemType>(function);

      m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChLink> FrAngularActuator<OffshoreSystemType>::GetChronoLink() {
      return m_chronoActuator;
    }

    template<typename OffshoreSystemType>
    chrono::ChLinkMotorRotation *FrAngularActuator<OffshoreSystemType>::GetChronoItem_ptr() const {
      return m_chronoActuator.get();
    }

    template<typename OffshoreSystemType>
    void FrAngularActuator<OffshoreSystemType>::Initialize() {

      // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
      m_chronoActuator->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), true,
                                   internal::FrFrame2ChFrame(this->GetNode2()->GetFrameWRT_COG_InBody()),
                                   internal::FrFrame2ChFrame(this->GetNode1()->GetFrameWRT_COG_InBody()));

    }

    template<typename OffshoreSystemType>
    double FrAngularActuator<OffshoreSystemType>::GetMotorPower() const {
      return m_chronoActuator->GetMotorTorque() * m_chronoActuator->GetMotorRot_dt();
    }

    template<typename OffshoreSystemType>
    Force FrAngularActuator<OffshoreSystemType>::GetMotorForceInNode(FRAME_CONVENTION fc) const {
      return {};
    }

    template<typename OffshoreSystemType>
    Torque FrAngularActuator<OffshoreSystemType>::GetMotorTorqueInNode(FRAME_CONVENTION fc) const {
      return {0., 0., m_chronoActuator->GetMotorTorque()};
    }


} // end namespace frydom
