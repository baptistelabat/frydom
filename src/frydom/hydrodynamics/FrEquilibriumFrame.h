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
    class FrEquilibriumFrame : public FrPrePhysicsItem {

    protected:

        FrBody* m_body;                ///< Link to the body to which the equilibrium frame if applied

        FrFrame m_frame;
        Velocity m_velocity;                     ///< translational velocity of the frame in world coordinates
        double m_angularVelocity;                ///< angular velocity of the frame around Z-direction

        bool m_initSpeedFromBody;        ///< Initialize the frame position, orientation and velocity according
        bool m_initPositionFromBody;     ///< to the body during the initialization stage

        double c_prevTime;

    public:

        /// Constructor of a new equilibrium frame with default position and no velocity
        /// \param body Body to which the equilibrium frame is linked
        explicit FrEquilibriumFrame(FrBody* body);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "EquilibriumFrame"; }

        /// Define the body to which the equilibrium frame is linked
        /// \param body Body link
        /// \param initPos Boolean, if true the position of the frame is equal to the position of the body during initialization
        void SetBody(FrBody* body, bool initPos = true);

        /// Set the position of the equilibrium frame, in the world reference frame
        /// \param Pos position of the equilibrium frame
        /// \param fc frame convention (NED/NWU)
        void SetPositionInWorld(Position Pos, FRAME_CONVENTION fc);

        /// Get the position of the equilibrium frame in the word reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return equilibrium frame position
        Position GetPositionInWorld(FRAME_CONVENTION fc) const;

        /// Set the rotation of the equilibrium frame, in the world reference frame
        /// \param rotation rotation of the equilibrium frame
        void SetRotation(const FrRotation& rotation);

        /// Get the rotation of the equilibrium frame in the word reference frame
        /// \return equilibrium frame rotation
        FrRotation GetRotation() const;

        /// Set the equilibrium reference frame relatively to the world reference frame
        /// \param Pos position of the equilibrium frame
        /// \param fc frame convention (NED/NWU)
        void SetFrameInWorld(const FrFrame& frame);

        /// Get the equilibrium reference frame relatively to the word reference frame
        /// \param fc frame convention (NED/NWU
        /// \return equilibrium reference frame
        FrFrame GetFrameInWorld() const;

        /// Set velocity of the equilibrium frame in the world reference frame
        /// \param velocity Velocity vector in the world reference frame
        /// \param fc Frame convention (NED/NWU)
        void SetVelocityInWorld(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Set velocity of the equilibrium frame in world coordinates
        /// \param velocity Velocity vector in frame coordinates
        void SetVelocityInFrame(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Set angular velocity around Z-direction
        /// \param angularVelocity Angular velocity, in rad/s
        /// \param fc Frame convention
        void SetAngularVelocityAroundZ(const double& angularVelocity, FRAME_CONVENTION fc);

        /// Get the linear velocity of the equilibrium frame in world coordinates
        /// \param fc Frame convention
        /// \return Velocity vector
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the linear velocity of the equilibrium frame in frame coordinates
        /// \return Velocity vector
        Velocity GetVelocityInFrame(FRAME_CONVENTION fc) const;

        FrFrame GetPerturbationFrame();

        /// Get the perturbation linear velocity of the body around the equilibrium frame
        /// \param fc Frame convention
        /// \return Perturbation velocity in world reference frame
        Velocity GetPerturbationVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the perturbation linear velocity of the body around the equilibrium frame
        /// \return Perturbation velocity in local frame
        Velocity GetPerturbationVelocityInFrame(FRAME_CONVENTION fc) const;

        /// Return the perturbation generalized velocity of the body around the equilibrium frame
        /// \param fc Frame convention
        /// \return Perturbation generalized velocity in world
        GeneralizedVelocity GetPerturbationGeneralizedVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Return the perturbation generalized velocity of the body around the equilibrium frame
        /// \return Perturbation generalized velocity in local frame
        GeneralizedVelocity GetPerturbationGeneralizedVelocityInFrame(FRAME_CONVENTION fc) const;

        /// Get the angular velocity of the equilibrium frame around the Z-axis
        /// \param fc Frame convention
        /// \return Angular velocity around Z (vertical)
        double GetAngularVelocityAroundZ(FRAME_CONVENTION fc) const;

        /// Get the angular velocity of the equilibrium frame
        /// \param fc Frame convention
        /// \return Angular velocity vector
        AngularVelocity GetAngularVelocity(FRAME_CONVENTION fc) const;

        AngularVelocity GetAngularPerturbationVelocity(FRAME_CONVENTION fc) const;

        AngularVelocity GetAngularPerturbationVelocityInFrame(FRAME_CONVENTION fc) const;

        /// Initialization of the position and velocity of the equilibrium frame
        void Initialize() override;

        /// Method to be applied after each time steps
        void StepFinalize() override;

        // Logging

        // Initialize the log
        void AddFields() override;

    protected:

        /// Set the position of the equilibrium frame equals to the body COG position
        void SetPositionToBodyCOGPosition();

        /// Set the velocity of the equilibrium frame equal to the body COG velocity
        void SetVelocityToBodyCOGVelocity();

        /// The velocity of the frame is initialized from the body velocity
        /// \param is_init Boolean True/Flase
        void InitSpeedFromBody(bool is_init);

        /// The position of the frame is initialized from the body position
        /// \param is_init Boolean True/False
        void InitPositionFromBody(bool is_init);

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
        FrEqFrameSpringDamping(FrBody* body, double T0, double psi);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "EqFrameSpringDamping"; }

        /// Set the spring-damping parameters
        /// \param T0 Cutoff time period
        /// \param psi Damping ratio
        void SetSpringDamping(double T0 = 60., double psi = 0.5);

        /// Get the damping coefficient of the spring-damping system
        /// \return Damping coefficient
        double GetDamping() const { return m_damping; };

        /// Set damping coefficient to the spring-damping system
        /// \param damping Damping coefficient
        void SetDamping(double damping) { m_damping = damping; }

        /// Get stiffness coefficient of the spring-damping system
        double GetStiffness() const { return m_stiffness; }

        /// Set the stiffness coefficient of the spring-damping system
        /// \param stiffness Stiffness coefficient
        void SetStiffness(double stiffness) { m_stiffness = stiffness; }

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
        FrEqFrameMeanMotion(FrBody* body, double timePersistence, double timeStep) ;

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
