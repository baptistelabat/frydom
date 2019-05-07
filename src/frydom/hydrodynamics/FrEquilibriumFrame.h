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

#include "frydom/core/common/FrFrame.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrPhysicsItem.h"



namespace frydom {

    // Forward declaration
    class FrBody;


    /**
     * \class FrEquilibriumFrame
     * \brief This class defines a generic equilibrium frame linked with a body.
     *
     * The equilibrium frame is a frame with the z-axis pointing upward, internal
     * velocities and dynamic behaviour. The generic equilibrium frame has
     * a constant speed in the horizontal plane and eventually a constant rotation
     * speed around Z. If the position, orientation and velocity of the equilibrium frame
     * are not defined by the user, they are initialized to the values given by the body at COG
     * during the initialization stage.
     *
     */
    class FrEquilibriumFrame : public FrFrame,
                                public FrPrePhysicsItem {

    protected:
        double m_prevTime;
        FrBody* m_body = nullptr;               ///< Link to the body to which the equilibrium frame if applied
        Velocity m_velocity;                     ///< translational velocity of the frame in world coordinates
        double m_angularVelocity;                ///< angular velocity of the frame around Z-direction
        bool m_initSpeedFromBody = false;        ///< Initialize the frame position, orientation and velocity according
        bool m_initPositionFromBody = false;     ///< to the body during the initialization stage

    public:

        /// Constructor of a new equilibrium frame with default position and no velocity
        /// User will must define the body to linked with with the corresponding method
        /// before execution of the simulation
        FrEquilibriumFrame() : FrFrame(), FrPrePhysicsItem(), m_angularVelocity(0.) { };

        /// Constructor of a new equilibrium frame with body linked
        /// \param body Body to which the equilibrium frame is linked
        /// \param initPos Boolean, if true the position of the equilibrium is set to the position of
        /// the body during initialization
        FrEquilibriumFrame(FrBody* body, bool initPos = true) : FrFrame(), FrPrePhysicsItem(), m_body(body),
                                                                  m_initPositionFromBody(initPos), m_angularVelocity(0.) { };

        /// Constructor of a new equilibrium frame with defined position, rotation and body linked
        /// \param pos Initial position of the equilibrium frame in world coordinates
        /// \param rotation Initial orientation of the equilibrium frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        FrEquilibriumFrame(const Position& pos, const FrRotation& rotation, FRAME_CONVENTION fc, FrBody* body)
                : FrFrame(pos, rotation, fc), FrPrePhysicsItem(), m_body(body), m_initPositionFromBody(false) { }

        /// Constructor of a new equilibrium frame with defined position, rotation and body linked
        /// \param pos Initial position of the equilibrium frame in world coordinates
        /// \param quaternion Initial orientation of the quilibrium frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        FrEquilibriumFrame(const Position& pos, const FrUnitQuaternion& quaternion, FRAME_CONVENTION fc, FrBody* body)
                : FrFrame(pos, quaternion, fc), FrPrePhysicsItem(), m_body(body), m_initPositionFromBody(false) { }

        /// Constructor of a new equilibrium frame from an other frame and body link
        /// \param otherFrame Initial frame definition
        /// \param body Body link
        FrEquilibriumFrame(const FrFrame& otherFrame, FrBody* body)
                : FrFrame(otherFrame), FrPrePhysicsItem(), m_body(body), m_initPositionFromBody(false) { }

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "EquilibriumFrame"; }

        /// Define the body to which the equilibrium frame is linked
        /// \param body Body link
        /// \param initPos Boolean, if true the position of the frame is equal to the position of the body during initialization
        void SetBody(FrBody* body, bool initPos = true);

        /// Set the position of the equilibrium frame equal to the position of the body at COG
        void SetPositionToBodyPosition();

        FrFrame GetPerturbationFrame();


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
        /// \param fc Frame convention
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

        /// Initialization of the position and velocity of the equilibrium frame
        void Initialize() override;

        /// Method to be applied after each time steps
        void StepFinalize() override;

        // Logging

        // Initialize the log
        void InitializeLog(const std::string& rootPath) override;

    private:

        /// Update the velocity and position of the frame
        /// \param time Current time of the simulation from the beginning
        void Compute(double time) override;

    };


    /**
     * \class FrEqFrameSpringDamping
     * \brief This class defines an equilibrium frame with a spring-damping system.
     *
     * The velocity of the equilibrium frame is solution of a dynamic equation with spring
     * and damping forces. This system creates a low pass filter on the velocity of the body.
     * The spring-damping system is defined from T0, the cutoff time in seconds, and
     * psi the damping rate coefficient.
     *
     */
    class FrEqFrameSpringDamping : public FrEquilibriumFrame {

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
        FrEqFrameSpringDamping(FrBody* body, double T0, double psi, bool initPos = true);

        /// Constructor of a new equilibrium frame with body link, position, orientation and spring-dampign parameters
        /// \param pos Position of the frame in world coordinates
        /// \param rotation Rotation of the frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        FrEqFrameSpringDamping(const Position &pos, const FrRotation &rotation,
                                FRAME_CONVENTION fc, FrBody* body, double T0, double psi);

        /// Constructor of a new equilibrium frame with body link, position, rotation and spring-damping parameters
        /// \param pos Position of the frame in world coordinates
        /// \param quaternion Rotation of the frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        /// \param T0 Cutoff time
        /// \param psi Damping ratio
        FrEqFrameSpringDamping(const Position& pos, const FrUnitQuaternion& quaternion, FRAME_CONVENTION fc,
                               FrBody* body, double T0, double psi);

        /// Constructor of a new equilibrium frame from a given frame with body link and spring-damping parameters
        /// \param otherFrame Other frame definition
        /// \param body Body link
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        FrEqFrameSpringDamping(const FrFrame& otherFrame, FrBody* body,
                                double T0, double psi);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "EqFrameSpringDamping"; }

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

    private:

        /// Update velocity and position of the equilibrium frame
        /// \param time Current time of the simulation from beginning
        void Compute(double time) override;

    };



    // TODO : il faudrait pouvoir retrancher une difference de position moyenne

    // Forward declaration
    template <typename T>
    class FrTimeRecorder;

    /**
     * \class FrEqFrameMeanMotion
     * \brief This class defines an equilibrium frame with a velocity equal to the mean motion of a body.
     *
     * The velocity of the frame is equal to the mean value of the body velocity
     * during a period of time specified by the user. Past velocities are recorded
     * in a buffer with a specific time stepper.
     */
    class FrEqFrameMeanMotion : public FrEquilibriumFrame {

    private:

        std::unique_ptr<FrTimeRecorder<Velocity>> m_TrSpeedRec;        ///< Recorder of the translational speed of the body
        std::unique_ptr<FrTimeRecorder<double>> m_AglSpeedRec;         ///< Recorder of the angular speed of the body around the vertical axis Z
        std::unique_ptr<FrTimeRecorder<Position>> m_ErrPositionRec;    ///< Recorder of the position error
        std::unique_ptr<FrTimeRecorder<double>> m_ErrAngleRec;
        double m_prevTime;              ///< Previous time recorded in the buffer
        double m_errPosCoeff;           ///< Damping coefficient for position correction
        double m_errAngleCoeff;

    public:

        /// Constructor of a new equilibrium frame with body link and mean function parameters
        /// \param body Body link
        /// \param timePersistence Time windows for the mean velocity computation
        /// \param timeStep Time step of the recorder
        /// \param initPos If true the frame is initialized with the position of the body
        FrEqFrameMeanMotion(FrBody* body, double timePersistence, double timeStep, bool initPos = true) ;

        /// Constructor of a new equilibrium frame with body link, position, rotation and mean function parameters
        /// \param pos Position of the frame in world coordinates
        /// \param rotation Rotation of the frame in world coordinates
        /// \param fc Frame convention
        /// \param body Body link
        /// \param timePersistence Time windows for mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion(const Position &pos, const FrRotation &rotation, FRAME_CONVENTION fc,
                             FrBody* body, double timePersistence, double timeStep);

        /// Constructor of a new equilibrium frame with body link, position , rotation and mean function parameters
        /// \param pos Position of the frame in world coordinates
        /// \param quaternion Rotation of the frame in world coordinates with quaternion
        /// \param fc Frame convention
        /// \param body Body link
        /// \param timePersistence Time windows for mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion(const Position &pos, const FrUnitQuaternion& quaternion, FRAME_CONVENTION fc,
                             FrBody* body, double timePersistence, double timeStep);

        /// Constructor of a new equilibrium frame with body link, frame definition and mean function parameters
        /// \param otherFrame Frame definition
        /// \param body Body link
        /// \param timePersistence Time windows for the mean velocity computation
        /// \param timeStep Time step for the recorder
        FrEqFrameMeanMotion(const FrFrame &otherFrame, FrBody* body, double timePersistence, double timeStep);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "EqFrameMeanMotion"; }

        void SetPositionCorrection(double timePersistence, double timeStep, double posCoeff, double angleCoeff);

    private:

        /// Update position and velocity of the equilibrium frame
        /// \param time Current time of the simulation from beginning
        void Compute(double time) override;

        /// Set the recorder of the body velocity
        /// \param timePersistence Time windows of the recorder
        /// \param timeStep Time step of the recorder
        void SetRecorders(double timePersistence, double timeStep);

    };

}  // end namespace frydom

#endif //FRYDOM_FREQUILIBRIUMFRAME_H
