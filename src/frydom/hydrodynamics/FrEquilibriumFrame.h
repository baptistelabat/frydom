//
// Created by camille on 20/11/18.
//

#ifndef FRYDOM_FREQUILIBRIUMFRAME_H
#define FRYDOM_FREQUILIBRIUMFRAME_H

#include <frydom/utils/FrRecorder.h>
#include "frydom/core/FrFrame.h"
#include "frydom/core/FrVector.h"
#include "frydom/core/FrPhysicsItem.h"
#include "frydom/core/FrBody.h"

namespace frydom {





    class FrEquilibriumFrame_ : public FrFrame_,
                                public FrPhysicsItem_ {

    protected:
        FrBody_* m_body = nullptr;               ///< Link to the body to which the equilibrium frame if applied
        Velocity m_velocity;                     ///< translational velocity of the frame in world coordinates
        double m_angularVelocity;                ///< angular velocity of the frame around Z-direction
        bool m_initSpeedFromBody = true;         ///< Initialize the frame position, orientation and velocity according
        bool m_initPositionFromBody;             ///< to the body during the initialization stage


    public:

        FrEquilibriumFrame_() : FrFrame_() { };

        FrEquilibriumFrame_(FrBody_* body, bool initPos = true) : FrFrame_(), m_body(body), m_initPositionFromBody(initPos) { };

        FrEquilibriumFrame_(const Position& pos, const FrRotation_& rotation, FRAME_CONVENTION fc, FrBody_* body)
                : FrFrame_(pos, rotation, fc), m_body(body), m_initPositionFromBody(false) { }

        FrEquilibriumFrame_(const Position& pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc, FrBody_* body)
                : FrFrame_(pos, quaternion, fc), m_body(body), m_initPositionFromBody(false) { }

        FrEquilibriumFrame_(const FrFrame_& otherFrame, FrBody_* body)
                : FrFrame_(otherFrame), m_body(body), m_initPositionFromBody(false) { }

        void SetBody(FrBody_* body, bool initPos = true);

        void SetVelocity(const Velocity& velocity);

        void SetAngularVelocity(const double& angularVelocity);

        void Update(double time) override { }

        void Initialize() override;

        void StepFinalize() override { }

    };






    class FrEqFrameSpringDamping_ : public FrEquilibriumFrame_ {

    private:
        double m_w0 = 0;
        double m_psi = 0;
        double m_damping = 0;
        double m_stiffness = 0;
        double m_prevTime = 0;

    public:

        FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi, bool initPos = true);

        FrEqFrameSpringDamping_(const Position &pos, const FrRotation_ &rotation,
                                FRAME_CONVENTION fc, FrBody_* body, double T0, double psi);

        FrEqFrameSpringDamping_(const Position& pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc,
                               FrBody_* body, double T0, double psi);

        FrEqFrameSpringDamping_(const FrFrame_& otherFrame, FrBody_* body,
                                double T0, double psi);

        void SetSpringDamping(const double T0 = 60., const double psi = 0.5);

        double GetDamping() const { return m_damping; };

        void SetDamping(const double damping) { m_damping = damping; }

        double GetStiffness() const { return m_stiffness; }

        void SetStiffness(const double stiffness) { m_stiffness = stiffness; }

        void Update(double time) override;

    };






    class FrEqFrameMeanMotion_ : public FrEquilibriumFrame_ {

    private:

        std::unique_ptr<FrTimeRecorder_<Velocity>> m_TrSpeedRec;
        std::unique_ptr<FrTimeRecorder_<double>> m_AglSpeedRec;
        double m_prevTime;

    public:

        FrEqFrameMeanMotion_(FrBody_* body, double timePersistence, double timeStep, bool initPos = true) ;

        FrEqFrameMeanMotion_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc,
                             FrBody_* body, double timePersistence, double timeStep);

        FrEqFrameMeanMotion_(const Position &pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc,
                             FrBody_* body, double timePersistence, double timeStep);

        FrEqFrameMeanMotion_(const FrFrame_ &otherFrame, FrBody_* body, double timePersistence, double timeStep);

        void Update(double time) override;

    private:

        void SetRecorders(double timePersistence, double timeStep);

    };

}

#endif //FRYDOM_FREQUILIBRIUMFRAME_H
