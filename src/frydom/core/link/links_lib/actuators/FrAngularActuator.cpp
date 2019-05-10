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

    Torque FrAngularActuator::GetMotorTorqueInWorld(FRAME_CONVENTION fc) const {

        auto MotorForce = m_chronoActuator->GetMotorTorque();

        return MotorForce * GetNode1()->GetFrameInWorld().GetZAxisInParent(fc);


//        auto coordSys = m_chronoActuator->GetLinkRelativeCoords();
//
//        m_chronoActuator->GetLinkAbsoluteCoords();
//
//        auto ReacForceInworld = internal::ChVectorToVector3d<Force>(coordSys.TransformDirectionLocalToParent(m_chronoActuator->Get_react_force()));
//
//        if (IsNED(fc)) internal::SwapFrameConvention(ReacForceInworld);
//
//        return ReacForceInworld;
    }

    void FrAngularActuator::Initialize() {

        m_chronoActuator->Initialize(GetChronoBody1(), GetChronoBody2(), true,
                                     internal::FrFrame2ChFrame(GetNode1()->GetFrameWRT_COG_InBody()),
                                     internal::FrFrame2ChFrame(GetNode2()->GetFrameWRT_COG_InBody()));

    }


} // end namespace frydom