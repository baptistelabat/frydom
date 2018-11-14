//
// Created by camille on 13/11/18.
//


#include "FrFlowBase.h"
#include "frydom/core/FrFrame.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/field/FrUniformField.h"

namespace frydom {

    FrFlowBase::FrFlowBase(FrEnvironment_ *environment) : m_environment(environment) {
        m_field = std::make_unique<FrUniformField>();
    };

    Velocity FrFlowBase::GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const {
        return m_field->GetFluxVelocityInWorld(worldPos, fc);
    }

    Velocity FrFlowBase::GetRelativeVelocityInFrame(const FrFrame_ &frame, const Velocity &worldVel,
                                                    FRAME_CONVENTION fc) const {
        Velocity fluxVelocityInWorld = GetFluxVelocityInWorld(frame.GetPosition(fc), fc);
        return frame.GetQuaternion().GetInverse().Rotate(fluxVelocityInWorld, fc);
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
    }

    void FrFlowBase::Update(double time) {
        m_field->Update(time);
    }

    void FrFlowBase::StepFinalize() {
        m_field->StepFinalize();
    }
}