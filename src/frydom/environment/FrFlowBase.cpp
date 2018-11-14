//
// Created by camille on 13/11/18.
//

#include "FrFlowBase.h"
#include "frydom/environment/field/FrFieldBase.h"

namespace frydom {


    Velocity FrFlowBase::GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const {
        return m_field->GetFluxVelocityInWorld(worldPos, fc);
    }

    Velocity FrFlowBase::GetRelativeVelocityInFrame(const FrFrame_ &frame, const Velocity &worldVel,
                                                    FRAME_CONVENTION fc) const {
        Velocity fluxVelocityInWorld = GetFluxVelocityInWorld(frame.GetPosition(fc), fc);
        return frame.GetQuaternion().GetInverse().Rotate(fluxVelocityInWorld, fc);
    }

    template <class Field>
    void FrFlowBase::NewField() {
        m_field = std::make_unique<Field>();
    }

    template <class Field>
    Field* FrFlowBase::GetField() const {
        return dynamic_cast<Field*>(m_field.get());
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