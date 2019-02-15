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


    /**
     * \class FrRevoluteLink
     * \brief Class for defining a revolute link.
     */
    class FrRevoluteLink : public FrLink_ {

    private:
        double m_stiffness = 0.; ///> Link rotational stiffness (N)
        double m_damping = 0.;   ///> Link rotational damping (Nm/s)

        std::shared_ptr<internal::FrLinkMotorRotationSpeedBase> m_speedMotor;


    public:

        FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        void SetSpringDamper(double stiffness, double damping);

        void SetRestAngle(double restAngle);

        double GetRestAngle() const;

        const Direction GetLinkAxisInWorld(FRAME_CONVENTION fc) const;

        double GetLinkAngle() const;

        double GetLinkAngularVelocity() const;

        double GetLinkAngularAcceleration() const;

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;


        void MotorizeSpeed();


    private:

    };

    std::shared_ptr<FrRevoluteLink> make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

}  // end namespace frydom

#endif //FRYDOM_FRREVOLUTELINK_H
