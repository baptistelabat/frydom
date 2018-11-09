//
// Created by frongere on 03/07/17.
//

#include "FrWind.h"

#include "chrono/core/ChMatrixDynamic.h"  // TODO : voir pourquoi on est oblige d'inclure
#include "frydom/environment/field/FrUniformCurrentField.h"

#include "frydom/core/FrFrame.h"

namespace frydom {

    Velocity FrWind_::GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel, FRAME_CONVENTION fc) {
        Velocity worldRelCurrentVel = GetWorldFluxVelocity(frame.GetPosition(fc), fc) - worldVel;
        return frame.GetQuaternion().GetInverse().Rotate(worldRelCurrentVel, fc);
    }

    FrUniformWind_::FrUniformWind_(FrEnvironment_ *environment) : m_environment(environment) {
        m_uniformField = std::make_unique<FrUniformCurrentField_>();
    }

    FrUniformCurrentField_* FrUniformWind_::GetField() {
        return m_uniformField.get();
    }

    void FrUniformWind_::Update(double time) {
        m_uniformField->Update(time);
    }

    Velocity FrUniformWind_::GetWorldFluxVelocity(const Position &absPos, FRAME_CONVENTION fc) {
        return m_uniformField->GetWorldFluxVelocity(fc);
    }

    void FrUniformWind_::Initialize() {
        m_uniformField->Initialize();
    }

    void FrUniformWind_::StepFinalize() {
        m_uniformField->StepFinalize();
    }

}  // end namespace frydom