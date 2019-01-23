//
// Created by camille on 13/11/18.
//


#include "FrFlowBase.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/functions/FrFunction.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/atmosphere/FrAtmosphere_.h"
#include "FrUniformField.h"

namespace frydom {

    // -----------------------------------------------------------
    // FLOW BASE
    // -----------------------------------------------------------

    FrFlowBase::FrFlowBase() {
        m_field = std::make_unique<FrUniformField>();
    };

    Velocity FrFlowBase::GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const {
        return m_field->GetFluxVelocityInWorld(worldPos, fc) * c_ramp;
    }

    Velocity FrFlowBase::GetRelativeVelocityInFrame(const FrFrame_ &frame, const Velocity &worldVel,
                                                    FRAME_CONVENTION fc) const {
        Velocity fluxVelocityInWorld = GetFluxVelocityInWorld(frame.GetPosition(fc), fc) - worldVel;
        if (IsNED(fc)) { internal::SwapFrameConvention(fluxVelocityInWorld); }
        return frame.GetQuaternion().GetInverse().Rotate(fluxVelocityInWorld, NWU);
    }

    template <class T>
    void FrFlowBase::NewField() {
        m_field = std::make_unique<T>();
    }

    void FrFlowBase::MakeFieldUniform() {
        m_field = std::make_unique<FrUniformField>();
    }

    template <class T>
    T* FrFlowBase::GetField() const {
        return dynamic_cast<T*>(m_field.get());
    }

    FrUniformField* FrFlowBase::GetFieldUniform() const {
        return dynamic_cast<FrUniformField*>(m_field.get());
    }

    void FrFlowBase::Initialize() {
        m_field->Initialize();
        c_ramp = GetEnvironment()->GetTimeRamp()->GetFunctionValue();
    }

    void FrFlowBase::Update(double time) {
        m_field->Update(time);
    }

    void FrFlowBase::StepFinalize() {
        m_field->StepFinalize();
        c_ramp = GetEnvironment()->GetTimeRamp()->GetFunctionValue();
    }

    // ---------------------------------------------------------
    // WIND
    // ---------------------------------------------------------

    FrEnvironment_* FrWind_::GetEnvironment() const {
        return m_atmosphere->GetEnvironment();
    }

    // ---------------------------------------------------------
    // CURRENT
    // ---------------------------------------------------------

    FrEnvironment_* FrCurrent_::GetEnvironment() const {
        return m_ocean->GetEnvironment();
    }
}
