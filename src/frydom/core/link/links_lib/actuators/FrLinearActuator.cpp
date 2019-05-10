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


    FrLinearActuator::FrLinearActuator(FrLink *actuatedLink, ACTUATOR_CONTROL control) : FrActuator(actuatedLink) {

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

    }

    void FrLinearActuator::SetMotorFunction(const FrFunctionBase &function) {

        auto chronoFunctionInterface = internal::FrFunctionChronoInterface(function);

        m_chronoActuator->SetMotorFunction(chronoFunctionInterface.GetChronoFunction());

    }

    std::shared_ptr<chrono::ChLink> FrLinearActuator::GetChronoLink() {
        return m_chronoActuator;
    }

    chrono::ChLinkMotorLinear *FrLinearActuator::GetChronoItem_ptr() const {
        return m_chronoActuator.get();
    }

    Force FrLinearActuator::GetMotorForceInWorld(FRAME_CONVENTION fc) const {

        auto MotorForce = m_chronoActuator->GetMotorForce();

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

    void FrLinearActuator::Initialize() {

        // ChLinkMotorLinear motorized along the x axis, while ChLinkLock::Prismatic is along z axis...
        auto frame1 = GetNode1()->GetFrameWRT_COG_InBody();
        frame1.RotY_RADIANS(MU_PI_2,NWU,true);
        auto frame2 = GetNode2()->GetFrameWRT_COG_InBody();
        frame2.RotY_RADIANS(MU_PI_2,NWU,true);

        m_chronoActuator->Initialize(GetChronoBody1(), GetChronoBody2(), true,
                   internal::FrFrame2ChFrame(frame1), internal::FrFrame2ChFrame(frame2));

    }


} // end namespace frydom