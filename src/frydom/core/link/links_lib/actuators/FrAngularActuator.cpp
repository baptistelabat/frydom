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


    FrAngularActuator::FrAngularActuator(FrLink *actuatedLink, ACTUATOR_CONTROL control) : FrActuator(actuatedLink) {

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

    }

    void FrAngularActuator::SetMotorFunction(const FrFunctionBase &function) {

        auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

        m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

    }

    std::shared_ptr<chrono::ChLink> FrAngularActuator::GetChronoLink() {
        return m_chronoActuator;
    }

    chrono::ChLinkMotorRotation *FrAngularActuator::GetChronoItem_ptr() const {
        return m_chronoActuator.get();
    }

    void FrAngularActuator::Initialize() {

        // IMPORTANT : in FRyDoM the first node is the master and the second one the slave, as opposed to Chrono !!!
        m_chronoActuator->Initialize(GetChronoBody2(), GetChronoBody1(), true,
                                     internal::FrFrame2ChFrame(GetNode2()->GetFrameWRT_COG_InBody()),
                                     internal::FrFrame2ChFrame(GetNode1()->GetFrameWRT_COG_InBody()));

    }

    double FrAngularActuator::GetMotorPower() const {
        return m_chronoActuator->GetMotorTorque() * m_chronoActuator->GetMotorRot_dt();
    }

    Force FrAngularActuator::GetMotorForceInMarker(FRAME_CONVENTION fc) const {
        return {};
    }

    Torque FrAngularActuator::GetMotorTorqueInMarker(FRAME_CONVENTION fc) const {
        return {0.,0.,m_chronoActuator->GetMotorTorque()};
    }


} // end namespace frydom