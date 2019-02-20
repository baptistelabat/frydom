// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================


#ifndef FRYDOM_FREQUILIBRIUMFRAME_H
#define FRYDOM_FREQUILIBRIUMFRAME_H

#include "frydom/utils/FrRecorder.h"
#include "frydom/core/common/FrFrame.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

    /// This class defines a generic equilibrium frame linked with a body.
    ///
    /// The equilibrium frame is a frame with the z-axis pointing upward, internal
    /// velocities and dynamic behaviour. The generic equilibrium frame has
    /// a constant speed in the horizontal plane and eventually a constant rotation
    /// speed around Z. If the position, orientation and velocity of the equilibrium frame
    /// are not defined by the user, they are initialized to the values given by the body at COG
    /// during the initialization stage.

    /**
     * \class FrEquilibriumFrame_
     * \brief This class defines a generic equilibrium frame linked with a body.
     */
    class FrEquilibriumFrame_ : public FrFrame_,
                                public FrPrePhysicsItem_ {

    protected:
        FrBody_* m_body = nullptr;               ///< Link to the body to which the equilibrium frame if applied
        Velocity m_velocity;                     ///< translational velocity of the frame in world coordinates
        double m_angularVelocity;                ///< angular velocity of the frame around Z-direction
        bool m_initSpeedFromBody = false;        ///< Initialize the frame position, orientation and velocity according
        bool m_initPositionFromBody = false;     ///< to the body during the initialization stage

    public:

        /// Constructor of a new equilibrium frame with default position and no velocity
        /// User will must define the body to linked with with the corresponding method
        /// before execution of the simulation
        FrEquilibriumFrame_() : FrFrame_(), FrPrePhysicsItem_(), m_angularVelocity(0.) { };

        /// Constructor of a new equilibrium frame with body linked
        /// \param body Body to which the equilibrium frame is linked
        /// \param initPos Boolean, if true the position of the equilibrium is set to the position of
        /// the body during initialization
        FrEquilibriumFrame_(FrBody_* body, bool initPos = true) : FrFrame_(), FrPrePhysicsItem_(), m_body(body),
                                                                  m_initPositionFromBody(initPos), m_angularVelocity(0.) { };

        /// Constructor of a new equilibrium frame with defined position, rotation and body linked
        /// \param pos Initial position of the equilibrium frame in world coordinates
        /// \param rotation Initial orientation of the equilibrium frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        FrEquilibriumFrame_(const Position& pos, const FrRotation_& rotation, FRAME_CONVENTION fc, FrBody_* body)
                : FrFrame_(pos, rotation, fc), FrPrePhysicsItem_(), m_body(body), m_initPositionFromBody(false) { }

        /// Constructor of a new equilibrium frame with defined position, rotation and body linked
        /// \param pos Initial position of the equilibrium frame in world coordinates
        /// \param quaternion Initial orientation of the quilibrium frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        FrEquilibriumFrame_(const Position& pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc, FrBody_* body)
                : FrFrame_(pos, quaternion, fc), FrPrePhysicsItem_(), m_body(body), m_initPositionFromBody(false) { }

        /// Constructor of a new equilibrium frame from an other frame and body link
        /// \param otherFrame Initial frame definition
        /// \param body Body link
        FrEquilibriumFrame_(const FrFrame_& otherFrame, FrBody_* body)
                : FrFrame_(otherFrame), FrPrePhysicsItem_(), m_body(body), m_initPositionFromBody(false) { }

        /// Define the body to which the equilibrium frame is linked
        /// \param body Body link
        /// \param initPos Boolean, if true the position of the frame is equal to the position of the body during initialization
        void SetBody(FrBody_* body, bool initPos = true);

        /// Set the position of the equilibrium frame equal to the position of the body at COG
        void SetPositionToBodyPosition();

        /// Set the velocity of the equilibrium frame equal to the velocity of the body at COG
        void SetVelocityToBodyVelocity();

        /// Set velocity of the equilibrium frame in world coordinates
        /// \param velocity Velocity vector in world coordinates
        /// \param fc Frame convention
        void SetVelocityInWorld(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Set velocity of the equilibrium frame in world coordinates
        /// \param velocity Velocity vector in frame coordinates
        void SetVelocityInFrame(const Velocity& velocity);

        /// Set angular velocity around Z-direction
        /// \param angularVelocity Angular velocity, in rad/s
        void SetAngularVelocityAroundZ(const double& angularVelocity, FRAME_CONVENTION fc);

        /// Get the velocity vector of the equilibrium frame in world coordinates
        /// \param fc Frame convention
        /// \return Velocity vector
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the velocity vector of the equilibrium frame in frame coordinates
        /// \return Velocity vector
        Velocity GetVelocityInFrame() const;

        Velocity GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const;

        Velocity GetPerturbationVelocityInFrame() const;

        GeneralizedVelocity GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const;

        GeneralizedVelocity GetPerturbationGeneralizedVelocityInFrame() const;

        /// Get the angular velocity of the equilibrium frame around the Z-axis
        /// \param fc Frame convention
        /// \return Angular velocity around Z (vertical)
        double GetAngularVelocityAroundZ(FRAME_CONVENTION fc) const;

        /// Get the angular velocity of the equilibrium frame
        /// \param fc Frame convention
        /// \return Angular velocity vector
        AngularVelocity GetAngularVelocity(FRAME_CONVENTION fc) const;

        AngularVelocity GetAngularPerturbationVelocity(FRAME_CONVENTION fc) const;

        AngularVelocity GetAngularPerturbationVelocityInFrame() const;

        /// The velocity of the frame is initialized from the body velocity
        /// \param is_init Boolean True/Flase
        void InitSpeedFromBody(bool is_init) { m_initSpeedFromBody = is_init; }

        /// The position of the frame is initialized from the body position
        /// \param is_init Boolean True/False
        void InitPositionFromBody(bool is_init) { m_initPositionFromBody = is_init; }

        /// Update the velocity and position of the frame
        /// \param time Current time of the simulation from the beginning
        void Update(double time) override { }

        /// Initialization of the position and velocity of the equilibrium frame
        void Initialize() override;

        /// Method to be applied after each time steps
        void StepFinalize() override { }

    };



    /// This class defines an equilibrium frame with a spring-damping system
    ///
    /// The velocity of the equilibrium frame is solution of a dynamic equation with spring
    /// and damping forces. This system creates a low pass filter on the velocity of the body.
    /// The spring-damping system is defined from T0, the cutoff time in seconds, and
    /// psi the damping rate coefficient.

    /**
     * \class FrEqFrameSpringDamping_
     * \brief This class defines an equilibrium frame with a spring-damping system.
     */
    class FrEqFrameSpringDamping_ : public FrEquilibriumFrame_ {

    private:
        double m_w0 = 0;                    ///< cutoff frequency
        double m_psi = 0;                   ///< damping parameter
        double m_damping = 0;               ///< damping coefficient of the system
        double m_stiffness = 0;             ///< stiffness coefficient of the system
        double m_prevTime = 0;              ///< previous time step

    public:

        /// Constructor of a new equilibrium frame with body link and spring-damping parameters
        /// \param body Body link
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        /// \param initPos If true the frame is initialized with the position of the body
        FrEqFrameSpringDamping_(FrBody_* body, double T0, double psi, bool initPos = true);

        /// Constructor of a new equilibrium frame with body link, position, orientation and spring-dampign parameters
        /// \param pos Position of the frame in world coordinates
        /// \param rotation Rotation of the frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        FrEqFrameSpringDamping_(const Position &pos, const FrRotation_ &rotation,
                                FRAME_CONVENTION fc, FrBody_* body, double T0, double psi);

        /// Constructor of a new equilibrium frame with body link, position, rotation and spring-damping parameters
        /// \param pos Position of the frame in world coordinates
        /// \param quaternion Rotation of the frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        /// \param T0 Cutoff time
        /// \param psi Damping ratio
        FrEqFrameSpringDamping_(const Position& pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc,
                               FrBody_* body, double T0, double psi);

        /// Constructor of a new equilibrium frame from a given frame with body link and spring-damping parameters
        /// \param otherFrame Other frame definition
        /// \param body Body link
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        FrEqFrameSpringDamping_(const FrFrame_& otherFrame, FrBody_* body,
                                double T0, double psi);

        /// Set the spring-damping parameters
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        void SetSpringDamping(const double T0 = 60., const double psi = 0.5);

        /// Get the damping coefficient of the spring-damping system
        /// \return Damping coefficient
        double GetDamping() const { return m_damping; };

        /// Set damping coefficient to the spring-damping system
        /// \param damping Damping coefficient
        void SetDamping(const double damping) { m_damping = damping; }

        /// Get stiffness coefficient of the spring-damping system
        double GetStiffness() const { return m_stiffness; }

        /// Set the stiffness coefficient of the spring-damping system
        /// \param stiffness Stiffness coefficient
        void SetStiffness(const double stiffness) { m_stiffness = stiffness; }

        /// Update velocity and position of the equilibrium frame
        /// \param time Current time of the simulation from beginning
        void Update(double time) override;

    };



    /// This class defines an equilibrium frame with a velocity equal to the mean motion of a body
    ///
    /// The velocity of the frame is equal to the mean value of the body velocity
    /// during a period of time specified by the user. Past velocities are recorded
    /// in a buffer with a specific time stepper.

    // TODO : il faudrait pouvoir retrancher une difference de position moyenne

    /**
     * \class FrEqFrameMeanMotion_
     * \brief This class defines an equilibrium frame with a velocity equal to the mean motion of a body.
     */
    class FrEqFrameMeanMotion_ : public FrEquilibriumFrame_ {

    private:

        std::unique_ptr<FrTimeRecorder_<Velocity>> m_TrSpeedRec;        ///< Recorder of the translational speed of the body
        std::unique_ptr<FrTimeRecorder_<double>> m_AglSpeedRec;         ///< Recorder of the angular speed of the body around the vertical axis Z
        std::unique_ptr<FrTimeRecorder_<Position>> m_ErrPositionRec;    ///< Recorder of the position error
        std::unique_ptr<FrTimeRecorder_<double>> m_ErrAngleRec;
        double m_prevTime;              ///< Previous time recorded in the buffer
        double m_errPosCoeff;           ///< Damping coefficient for position correction
        double m_errAngleCoeff;

    public:

        /// Constructor of a new equilibrium frame with body link and mean function parameters
        /// \param body Body link
        /// \param timePersistence Time windows for the mean velocity computation
        /// \param timeStep Time step of the recorder
        /// \param initPos If true the frame is initialized with the position of the body
        FrEqFrameMeanMotion_(FrBody_* body, double timePersistence, double timeStep, bool initPos = true) ;

        /// Constructor of a new equilibrium frame with body link, position, rotation and mean function parameters
        /// \param pos Position of the frame in world coordinates
        /// \param rotation Rotation of the frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        /// \param timePersistence Time windows for mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc,
                             FrBody_* body, double timePersistence, double timeStep);

        /// Constructor of a new equilibrium frame with body link, position , rotation and mean function parameters
        /// \param pos Position of the frame in world coordinates
        /// \param quaternion Rotation of the frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        /// \param timePersistence Time windows for mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion_(const Position &pos, const FrUnitQuaternion_& quaternion, FRAME_CONVENTION fc,
                             FrBody_* body, double timePersistence, double timeStep);

        /// Constructor of a new equilibrium frame with body link, frame definition and mean function parameters
        /// \param otherFrame Frame definition
        /// \param body Body link
        /// \param timePersistence Time windows for the mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion_(const FrFrame_ &otherFrame, FrBody_* body, double timePersistence, double timeStep);

        void SetPositionCorrection(double timePersistence, double timeStep, double posCoeff, double angleCoeff);

        /// Update position and velocity of the equilibrium frame
        /// \param time Current time of the simulation from beginning
        void Update(double time) override;

    private:

        /// Set the recorder of the body velocity
        /// \param timePersistence Time windows of the recorder
        /// \param timeStep Time step of the recorder
        void SetRecorders(double timePersistence, double timeStep);

    };

}  // end of the namespace frydom

#endif //FRYDOM_FREQUILIBRIUMFRAME_H
