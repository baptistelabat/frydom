//
// Created by lletourn on 07/05/19.
//

#include "FrLinearActuator.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"

#include "frydom/core/math/functions/FrFunctionsInc.h"
#include "frydom/core/common/FrNode.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrLinearActuator<OffshoreSystemType>::FrLinearActuator(FrLink<OffshoreSystemType> *actuatedLink,
                                                           ACTUATOR_CONTROL control) : FrActuator<OffshoreSystemType>(
        actuatedLink) {

      switch (control) {
        case POSITION :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearPosition>();
          break;
        case VELOCITY :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearSpeed>();
          break;
        case FORCE :
          m_chronoActuator = std::make_shared<chrono::ChLinkMotorLinearForce>();
          break;
      }
      m_chronoActuator->SetGuideConstraint(chrono::ChLinkMotorLinear::GuideConstraint::FREE);

    }

    template<typename OffshoreSystemType>
    void FrLinearActuator<OffshoreSystemType>::SetMotorFunction(const FrFunctionBase &function) {

      auto chronoFunctionInterface = internal::FrFunctionChronoInterface<OffshoreSystemType>(function);

      m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

    }

    template<typename OffshoreSystemType>
    std::shared_ptr<chrono::ChLink> FrLinearActuator<OffshoreSystemType>::GetChronoLink() {
      return m_chronoActuator;
    }

    template<typename OffshoreSystemType>
    chrono::ChLinkMotorLinear *FrLinearActuator<OffshoreSystemType>::GetChronoItem_ptr() const {
      return m_chronoActuator.get();
    }

    template<typename OffshoreSystemType>
    void FrLinearActuator<OffshoreSystemType>::Initialize() {

      // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
      // ChLinkMotorLinear motorized along the x axis, while ChLinkLock::Prismatic is along z axis...
      auto frame1 = this->GetNode1()->GetFrameWRT_COG_InBody();
      frame1.RotY_RADIANS(MU_PI_2, NWU, true);
      auto frame2 = this->GetNode2()->GetFrameWRT_COG_InBody();
      frame2.RotY_RADIANS(MU_PI_2, NWU, true);

      m_chronoActuator->Initialize(this->GetChronoBody2(), this->GetChronoBody1(), true,
                                   internal::FrFrame2ChFrame(frame2), internal::FrFrame2ChFrame(frame1));

    }

    template<typename OffshoreSystemType>
    double FrLinearActuator<OffshoreSystemType>::GetMotorPower() const {
      return m_chronoActuator->GetMotorForce() * m_chronoActuator->GetMotorPos_dt();
    }

    template<typename OffshoreSystemType>
    Force FrLinearActuator<OffshoreSystemType>::GetMotorForceInNode(FRAME_CONVENTION fc) const {
      return {0., 0., m_chronoActuator->GetMotorForce()};
    }

    template<typename OffshoreSystemType>
    Torque FrLinearActuator<OffshoreSystemType>::GetMotorTorqueInNode(FRAME_CONVENTION fc) const {
      return {};
    }


} // end namespace frydom
