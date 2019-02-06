//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRREVOLUTELINK_H
#define FRYDOM_FRREVOLUTELINK_H

#include "FrLink.h"


#include "hermes/hermes.h"



namespace frydom {


//    // Forward declaration
//    class FrAngularActuator;

    /// Specialized class for revolute link between two bodies
    class FrRevoluteLink : public FrLink_ {

    private:
        double m_stiffness = 0.; ///> Link rotational stiffness (N)
        double m_damping = 0.;   ///> Link rotational damping (Nm/s)


        double m_restAngle = 0.;  ///> The rest angle (reference) for which the measurement is taken. Between ]-pi, pi]

        double m_totalLinkAngle = 0.;  ///> The total angle of the link, accounting for numti-turn but not on rest angle.
                                       /// Defined on the continuous real line, centered on 0 rad
        double m_linkAngularVelocity = 0.;
        double m_linkAngularAcceleration = 0.;





        // LOG
        hermes::Message l_message;
        double l_time = 0.;
        double l_torque = 0.;
        double l_angleDeg = 0.;
        // LOG

    public:
        /// Constructor from two nodes and a pointer to the system.
        /// It automatically adds the link to the system
        FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        /// Set the spring and damper coefficients
        void SetSpringDamper(double stiffness, double damping);

        /// Set the rest angle of the link
        void SetRestAngle(double restAngle);

        /// Get the rest angle of the link
        double GetRestAngle() const;

        /// Get the direction of the link in world woordinate system
        const Direction GetLinkAxisInWorld(FRAME_CONVENTION fc) const;

        /// Get the current link angle in rad with respect to the rest angle
        /// It keeps track of multiple turns and is always continuous
        double GetLinkAngle() const;

        /// Get the relative angle in rad of the link with respect to the rest angle, not accounting for the number of
        /// turns
        double GetRelativeLinkAngle() const;

        /// Get the accumulated number of turns in the link
        int GetNbTurns() const;

        /// Get the link angular velocity, in rad/s
        double GetLinkAngularVelocity() const;

        /// Get the link angular acceleration, in rad/s*2
        double GetLinkAngularAcceleration() const;

        /// Get the link torque applying on body 2 around z axis
        double GetLinkTorque() const;

        /// Get the power delivered by the force in the link, around the z axis of the link
        double GetLinkPower() const override;

        /// Initialize the link
        void Initialize() override;

        /// Update the link
        void Update(double time) override;

        /// Called after every time step, only once
        void StepFinalize() override;

        /// Compute the link force. Here this is essentially a torque with a default spring damper.
        void UpdateForces(double time); // TODO : mettre en abstrait dans FrLink_ pour que toutes les classes possedent ca

        /// Motorize the link to make it driven // TODO : work in progress
        void MotorizeSpeed();


    private:

        /// Get the link relative angle as obtained by time integration, not accounting for rest angle
        double GetUpdatedRelativeAngle() const;

        /// Update the link cache
        void UpdateCache() override;

    };

    /// Helper function to make it easy to link two nodes by a revolute link
    std::shared_ptr<FrRevoluteLink> make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

}  // end namespace frydom

#endif //FRYDOM_FRREVOLUTELINK_H
