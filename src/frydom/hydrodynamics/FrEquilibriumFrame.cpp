//
// Created by camille on 20/11/18.
//

#include "FrEquilibriumFrame.h"

namespace frydom {


    // ---------------------------------------------------------------------
    // Equilibrium frame
    // ---------------------------------------------------------------------

    FrEquilibriumFrame_::FrEquilibriumFrame_(FrBody_* body) : m_body(body) {
    }




    // -----------------------------------------------------------------------
    // Equilibrium frame with spring damping restoring force
    // -----------------------------------------------------------------------

    FrEqFrameSpringDamping_::FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi)
        : FrEquilibriumFrame_(body) { this->SetSpringDamping(T0, psi); }


    void FrEqFrameSpringDamping_::SetSpringDamping(const double T0, const double psi) {

        m_w0 = 2.*M_PI / T0;
        m_psi = psi;

        m_damping = 2. * m_psi * m_w0;
        m_stiffness = m_w0 * m_w0;
    }

    void FrEqFrameSpringDamping_::Update(double time) {

        auto bodyPosition = m_body->GetPosition(NWU);
        auto bodyVelocity = m_body->GetVelocityInWorld(NWU);
        auto position = GetPosition(NWU);

        Force force;
        force = -(bodyPosition - position) * m_stiffness - (bodyVelocity - m_velocity) * m_damping;

        m_velocity += force * (time - m_prevTime);
        position += m_velocity * (time - m_prevTime);

        this->SetPosition(position, NWU);
        m_prevTime = time;
    }

    // ----------------------------------------------------------------
    // Equilibrium frame with updated mean velocity
    // ----------------------------------------------------------------



}
