//
// Created by camille on 20/11/18.
//

#ifndef FRYDOM_FREQUILIBRIUMFRAME_H
#define FRYDOM_FREQUILIBRIUMFRAME_H

#include "frydom/core/FrFrame.h"
#include "frydom/core/FrVector.h"
#include "frydom/core/FrPhysicsItem.h"
#include "frydom/core/FrBody.h"

namespace frydom {

    class FrEquilibriumFrame_ : public FrFrame_,
                                public FrPhysicsItem_ {


    protected:

        FrBody_* m_body;

        Velocity m_velocity;
        AngularVelocity m_angularVelocity;

    public:

        FrEquilibriumFrame_(FrBody_* body);

        //void SetVelocity(const Velocity& velocity);
        //void SetAngularVelocity(const AngularVelocity& angularVelocity);

        void Update(double time) override { };

    };


    class FrEqFrameSpringDamping_ : public FrEquilibriumFrame_ {

    private:
        double m_w0;
        double m_psi;
        double m_damping;
        double m_stiffness;
        FrFrame_* m_frame;
        double m_prevTime;

    public:

        FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi);

        void SetSpringDamping(const double T0 = 60., const double psi = 0.5);

        void Update(double time) override;

    };


    /*
    class FrEqFrameMeanMotion_ : public FrEquilibriumFrame_ {

    private:

    public:



    };
    */
}

#endif //FRYDOM_FREQUILIBRIUMFRAME_H
