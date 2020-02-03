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
#include "frydom/core/common/FrTreeNode.h"
#include "frydom/logging/FrLoggable.h"


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
  class FrEquilibriumFrame : public FrPrePhysicsItem, public FrLoggable<FrBody> {

   protected:

    FrFrame m_frame;
    std::shared_ptr<FrNode>  m_bodyNode;  ///< Node fixed to the body corresponding to the equilibrium frame when the body is at equilibrium.
    Velocity m_velocity;             ///< translational velocity of the frame in world coordinates
    double m_angularVelocity;        ///< angular velocity of the frame around Z-direction

    bool m_initSpeedFromBody;        ///< Initialize the frame position, orientation and velocity according

    double c_prevTime;

   public:

    /// Constructor of a new equilibrium frame with default position and no velocity
    /// \param body Body to which the equilibrium frame is linked
    explicit FrEquilibriumFrame(const std::string &name, FrBody *body,
        const Position& localPos, const double& rot, FRAME_CONVENTION fc);

    /// Get a pointer to the body to which this frame is attached
    inline FrBody *GetBody() const;

    /// The velocity of the frame is initialized from the body velocity
    /// \param is_init Boolean True/Flase
    void InitializeVelocityFromBody(bool is_init);

    /// Get the position of the equilibrium frame in the word reference frame
    /// \param fc frame convention (NED/NWU)
    /// \return equilibrium frame position
    Position GetPositionInWorld(FRAME_CONVENTION fc) const;

    //Position GetPositionInBody(FRAME_CONVENTION fc) const;

    /// Get the rotation of the equilibrium frame in the word reference frame
    /// \return equilibrium frame rotation
    FrRotation GetRotation() const;

    /// Get the equilibrium reference frame relatively to the word reference frame
    /// \param fc frame convention (NED/NWU)
    /// \return equilibrium reference frame
    FrFrame GetFrame() const;

    /// Set velocity of the equilibrium frame in the world reference frame
    /// \param velocity Velocity vector in the world reference frame
    /// \param fc Frame convention (NED/NWU)
    void SetVelocityInWorld(const Velocity &velocity, FRAME_CONVENTION fc);

    /// Set velocity of the equilibrium frame in world coordinates
    /// \param velocity Velocity vector in frame coordinates
    void SetVelocityInFrame(const Velocity &velocity, FRAME_CONVENTION fc);

    /// Set angular velocity around Z-direction
    /// \param angularVelocity Angular velocity, in rad/s
    /// \param fc Frame convention
    void SetAngularVelocity(const double &angularVelocity, FRAME_CONVENTION fc);

    /// Get the linear velocity of the equilibrium frame in world coordinates
    /// \param fc Frame convention
    /// \return Velocity vector
    Velocity GetFrameVelocityInWorld(FRAME_CONVENTION fc) const;

    /// Get the linear velocity of the equilibrium frame in frame coordinates
    /// \return Velocity vector
    Velocity GetFrameVelocityInFrame(FRAME_CONVENTION fc) const;

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

    /// Get the angular velocity of the equilibrium frame
    /// \param fc Frame convention
    /// \return Angular velocity vector
    AngularVelocity GetFrameAngularVelocity(FRAME_CONVENTION fc) const;

    AngularVelocity GetPerturbationAngularVelocity(FRAME_CONVENTION fc) const;

    AngularVelocity GetPerturbationAngularVelocityInFrame(FRAME_CONVENTION fc) const;

    /// Initialization of the position and velocity of the equilibrium frame
    void Initialize() override;

    /// Method to be applied after each time steps
    void StepFinalize() override;

   protected:

    void DefineLogMessages() override;

    void ApplyFrameProjection();

    void SetAngleRotation(const double& angle, FRAME_CONVENTION fc);

   private:

    void SetEqFramePositionOrientation();

    void SetEqFramePositionOrientation(const Position& localPos, const double& rot, FRAME_CONVENTION fc);

    /// Update the velocity and position of the frame
    /// \param time Current time of the simulation from the beginning
    void Compute(double time) override;

  };

  std::shared_ptr<FrEquilibriumFrame>
  make_equilibrium_frame(const std::string &name, FrOffshoreSystem *system, const std::shared_ptr<FrBody> &body,
      const Position& localPos, const double& rot, FRAME_CONVENTION fc);

  std::shared_ptr<FrEquilibriumFrame>
  make_equilibrium_frame(const std::string& name, FrOffshoreSystem* system, const std::shared_ptr<FrBody>& body);

  /**
   * \class FrEqFrameSpringDamping
   * \brief This class defines an equilibrium frame with a spring-damping system.
   *
   * The velocity of the equilibrium frame is solution of a dynamic equation with spring
   * and damping forces. This system creates a low pass filter on the velocity of the body.
   * The spring-damping system is defined from the cutoff time in seconds, and the damping
   * ratio coefficient.
   *
   */
  class FrEqFrameSpringDamping : public FrEquilibriumFrame {

   private:
    double m_damping = 0;               ///< damping coefficient of the system
    double m_stiffness = 0;             ///< stiffness coefficient of the system
    double m_prevTime = 0;              ///< previous time step

   public:

    /// Constructor of a new equilibrium frame with body link and spring-damping parameters
    /// \param body Body link
    /// \param cutoffTime Cutoff time period
    /// \param dampingRatio Damping ratio
    /// \param initPos If true the frame is initialized with the position of the body
    FrEqFrameSpringDamping(const std::string &name, FrBody *body,
        const Position& localPos, const double& rot, FRAME_CONVENTION fc,
        double cutoffTime, double dampingRatio);

    /// Get the damping coefficient of the spring-damping system
    /// \return Damping coefficient
    double GetDamping() const { return m_damping; };

    /// Get stiffness coefficient of the spring-damping system
    double GetStiffness() const { return m_stiffness; }

   private:

    /// Update velocity and position of the equilibrium frame
    /// \param time Current time of the simulation from beginning
    void Compute(double time) override;

    void SetSpringDamping(double cutoffTime, double dampingRatio);

  };

  std::shared_ptr<FrEqFrameSpringDamping>
  make_spring_damping_equilibrium_frame(const std::string &name,
                                        const std::shared_ptr<FrBody> &body,
                                        const Position& localPos,
                                        const double& rot,
                                        FRAME_CONVENTION fc,
                                        FrOffshoreSystem *system,
                                        double cutoffTime,
                                        double dampingRatio);

  std::shared_ptr<FrEqFrameSpringDamping>
  make_spring_damping_equilibrium_frame(const std::string& name,
                                        const std::shared_ptr<FrBody>& body,
                                        FrOffshoreSystem* system,
                                        double cutoffTime,
                                        double dampingRatio);

  // TODO : il faudrait pouvoir retrancher une difference de position moyenne

  // Forward declaration
  template<typename T>
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
    FrEqFrameMeanMotion(const std::string &name, FrBody *body,
        const Position& localPos, const double& rot, FRAME_CONVENTION fc,
        double timePersistence, double timeStep);

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

  std::shared_ptr<FrEqFrameMeanMotion>
  make_mean_motion_equilibrium_frame(const std::string &name, FrOffshoreSystem *system,
                                     const std::shared_ptr<FrBody> &body,
                                     double timePersistence,
                                     double timeStep);

  std::shared_ptr<FrEqFrameMeanMotion>
  make_mean_motion_equilibrium_frame(const std::string& name, FrOffshoreSystem*,
                                     std::shared_ptr<FrBody>& body,
                                     const Position& localPos,
                                     const double& rot,
                                     FRAME_CONVENTION fc,
                                     double timePersistence,
                                     double timeStep);

}  // end namespace frydom

#endif //FRYDOM_FREQUILIBRIUMFRAME_H
