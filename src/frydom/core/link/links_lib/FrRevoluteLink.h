//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRREVOLUTELINK_H
#define FRYDOM_FRREVOLUTELINK_H

#include "FrLink.h"


#include "chrono/physics/ChLinkMotorRotationSpeed.h"



namespace frydom {



    enum MOTOR_TYPE {
        LINEAR_MOTOR,
        ANGULAR_MOTOR,
        SPEED_MOTOR,
        FORCE_MOTOR
    };





    namespace internal {

        struct FrLinkMotorRotationSpeedBase : public chrono::ChLinkMotorRotationSpeed {

            FrLink_* m_frydomLink;

            explicit FrLinkMotorRotationSpeedBase(FrLink_* frydomLink);

            void Initialize();

        };

    }  // end namespace frydom::internal



    class FrRevoluteLink : public FrLink_ {

    private:
        double m_stiffness = 0.; ///> Link rotational stiffness (N)
        double m_damping = 0.;   ///> Link rotational damping (Nm/s)


        double m_restAngle = 0.;

        double m_totalLinkAngle = 0.;  /// The total angle of the link, accounting for numti-turn but not on rest angle
        double m_linkAngularVelocity = 0.;
        double m_linkAngularAcceleration = 0.;

        std::shared_ptr<internal::FrLinkMotorRotationSpeedBase> m_speedMotor;


    public:

        FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        void SetSpringDamper(double stiffness, double damping);

        void SetRestAngle(double restAngle);



        const Direction GetLinkAxisInWorld(FRAME_CONVENTION fc) const;

        /// Get the current link rotation in rad, including errors as well as rest angle etc.
        /// It keeps track of multiple turns
        double GetLinkAngle() const;

        double GetRestAngle() const;

        double GetLinkAngularVelocity() const;

        double GetLinkAngularAcceleration() const;

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

        /// Compute the link force. Here this is essentially a torque with a default spring damper.
        void UpdateForces(double time); // TODO : mettre en abstrait dans FrLink_ pour que toutes les classes possedent ca

        void MotorizeSpeed();

        int GetNbTurns() const;


    private:

        double GetTurnAngle() const;

        void UpdateCache() override;

//        /// Get the angle from the X axis. Does not refer to the rest angle. It keeps track of the number of turns
//        double GetAngleFromX() const;





    };

    std::shared_ptr<FrRevoluteLink> make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

}  // end namespace frydom

#endif //FRYDOM_FRREVOLUTELINK_H
