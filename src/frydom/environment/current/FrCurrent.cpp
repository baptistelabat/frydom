// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base for marine current modeling
//
// =============================================================================


#include "FrCurrent.h"

#include "chrono/core/ChMatrixDynamic.h"  // TODO : voir pourquoi on est oblige d'inclure
#include "frydom/environment/field/FrUniformCurrentField.h"

#include "frydom/core/FrFrame.h"

namespace frydom {

    Velocity FrCurrent_::GetRelativeVelocityInFrame(const FrFrame_& frame, const Velocity& worldVel, FRAME_CONVENTION fc) {
        Velocity worldRelCurrentVel = GetWorldFluxVelocity(frame.GetPosition(fc), fc) - worldVel;
        return frame.GetQuaternion().GetInverse().Rotate(worldRelCurrentVel, fc);
    }

    FrUniformCurrent_::FrUniformCurrent_(FrEnvironment_ *environment) : m_environment(environment) {
        m_uniformField = std::make_unique<FrUniformCurrentField_>();
    }

    FrUniformCurrentField_* FrUniformCurrent_::GetField() {
        return m_uniformField.get();
    }

    void FrUniformCurrent_::Update(double time) {
        m_uniformField->Update(time);
    }

    Velocity FrUniformCurrent_::GetWorldFluxVelocity(const Position &absPos, FRAME_CONVENTION fc) {
        return m_uniformField->GetWorldFluxVelocity(fc);
    }

    void FrUniformCurrent_::Initialize() {
        m_uniformField->Initialize();
    }

    void FrUniformCurrent_::StepFinalize() {
        m_uniformField->StepFinalize();
    }

}  // end namespace frydom
