//
// Created by frongere on 24/08/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H


#include <chrono/physics/ChController.h>
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"

#include "MathUtils/MathUtils.h"

#include "FrNode.h"

namespace frydom {



    class FrActuator  : public FrObject { // TODO mettre dans son propre fichier !!
        // TODO: cette classe de base devra servir pour introduire l'interactivite ...

        void Initialize() override {

        }

        void StepFinalize() override {

        }

    };






    class FrLinkMotorRotationAngle;

    class FrPositionShapeTanh : public chrono::ChFunction_SetpointCallback {

    private:

        FrLinkMotorRotationAngle* m_motor;

        double m_positionOrder = 0.;
        double m_maxVelocity = 0.;
        double m_t0 = 0.;
        double m_eps = 1e-3;

        double c_y0 = 0.;
        double c_a = 0.;
        double c_dy = 0.;
        double c_T = 0.;


    public:

        explicit FrPositionShapeTanh(FrLinkMotorRotationAngle* motor) : m_motor(motor) {
//            m_controller = std::make_unique<chrono::ChControllerPID>();
        }

        void SetMaxVelocity(const double maxvel) { m_maxVelocity = maxvel; }

        void SetPositionOrder(const double pos);

        void SetEps(const double eps) { m_eps = eps; }

        double SetpointCallback(const double t) override;

    };







    class FrLinkMotorRotationAngle : public FrActuator, public chrono::ChLinkMotorRotationAngle {

    private:
        // TODO: permettre de specifier des ordres avec des profils

    public:

        FrLinkMotorRotationAngle() : FrActuator(), chrono::ChLinkMotorRotationAngle() {
            SetAngleFunction(std::make_shared<chrono::ChFunction_Const>(0.));
        }

        void SetAngleOrder(const double angle, const double wMax=MU_RPM2RADS*0.5) {
            auto tanhShape = std::make_shared<FrPositionShapeTanh>(this);
            tanhShape->SetMaxVelocity(wMax);
            tanhShape->SetPositionOrder(angle);

            SetAngleFunction(tanhShape);
        }

        void Initialize(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {

            auto body1 = node1->GetBody();
            auto body2 = node2->GetBody();

            assert(body1 != body2);

            this->Body1 = body1;
            this->Body2 = body2;

            this->mask->SetTwoBodiesVariables(&body1->Variables(), &body2->Variables());

            this->frame1 = *node1.get();
            this->frame2 = *node2.get();
        }

    };

    std::shared_ptr<FrLinkMotorRotationAngle> make_motorRotationAngle(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {
        auto motor = std::make_shared<FrLinkMotorRotationAngle>();
        motor->Initialize(node1, node2);
        return motor;
    }



    class FrLinkMotorRotationSpeed : public FrActuator, public chrono::ChLinkMotorRotationSpeed {

    public:

        FrLinkMotorRotationSpeed() : FrActuator(), chrono::ChLinkMotorRotationSpeed() {
            m_func = std::make_shared<chrono::ChFunction_Const>(0.0); // By default, no speed !
        };

        void Initialize(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {

            auto body1 = node1->GetBody();
            auto body2 = node2->GetBody();

            assert(body1 != body2);

            this->Body1 = body1;
            this->Body2 = body2;

            this->mask->SetTwoBodiesVariables(&body1->Variables(), &body2->Variables());

            this->frame1 = *node1.get();
            this->frame2 = *node2.get();

        }

        // TODO: ajouter les methodes pratiques pour specifier des mouvements prefaits

        void SetConstantSpeed(double w_rads) {
            m_func = std::make_shared<chrono::ChFunction_Const>(w_rads);
        }


    };

    std::shared_ptr<FrLinkMotorRotationSpeed> make_motorRotationSpeed(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {
        auto motor = std::make_shared<FrLinkMotorRotationSpeed>();
        motor->Initialize(node1, node2);
        return motor;
    }


//    class FrPIDController : chrono::ChControllerPID {
//
//
//    };


    class FrLinkMotorRotationTorque;
    // TODO : avoir une bibliotheque de reglages par defaut pour un certain nombre d'actionneurs (genre grues, winch, etc...)


    // Utiliser SetSetpoint(consigne, temps) pour
    class FrRotationMotorRegulator : public chrono::ChFunction_SetpointCallback {

    private:

        std::unique_ptr<chrono::ChControllerPID> m_controller;
        FrLinkMotorRotationTorque* m_motor;

        double m_angleOrder = 0.;


    public:

        explicit FrRotationMotorRegulator(FrLinkMotorRotationTorque* motor) : m_motor(motor) {
            m_controller = std::make_unique<chrono::ChControllerPID>();
        }

        void SetControllerCoeffs(double Kp, double Ki, double Kd) {
            SetKp(Kp);
            SetKi(Ki);
            SetKd(Kd);
        }

        void SetKp(double Kp) { m_controller->P = Kp; }
        void SetKi(double Ki) { m_controller->I = Ki; }
        void SetKd(double Kd) { m_controller->D = Kd; }

//        void SetSetpoint(double setpoint) {
//            chrono::ChFunction_SetpointCallback::SetSetpoint(setpoint, 0.); // We don't care about time information in our case
//        }

        void SetAngleOrder(const double angle) {
            m_angleOrder = angle;
        }


        double SetpointCallback(const double t) override;

    };




    class FrLinkMotorRotationTorque : public FrActuator, public chrono::ChLinkMotorRotationTorque {

    public:

        FrLinkMotorRotationTorque() : chrono::ChLinkMotorRotationTorque() {
            auto regulator = std::make_shared<FrRotationMotorRegulator>(this);
            SetTorqueFunction(regulator);
        }

        void Initialize(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {

            auto body1 = node1->GetBody();
            auto body2 = node2->GetBody();

            assert(body1 != body2);

            this->Body1 = body1;
            this->Body2 = body2;

            this->mask->SetTwoBodiesVariables(&body1->Variables(), &body2->Variables());

            this->frame1 = *node1.get();
            this->frame2 = *node2.get();

        }

        void SetAngleOrder(const double angle, mathutils::ANGLE_UNIT unit = mathutils::DEG) {
            double order = angle;
            if (unit == mathutils::DEG) order *= DEG2RAD;
            dynamic_cast<FrRotationMotorRegulator*>(GetTorqueFunction().get())->SetAngleOrder(order);
        }

        void SetKp(const double Kp) { dynamic_cast<FrRotationMotorRegulator*>(GetTorqueFunction().get())->SetKp(Kp); }
        void SetKi(const double Ki) { dynamic_cast<FrRotationMotorRegulator*>(GetTorqueFunction().get())->SetKi(Ki); }
        void SetKd(const double Kd) { dynamic_cast<FrRotationMotorRegulator*>(GetTorqueFunction().get())->SetKd(Kd); }


        void SetControllerCoeffs(const double Kp, const double Ki, const double Kd) {
            dynamic_cast<FrRotationMotorRegulator*>(GetTorqueFunction().get())->SetControllerCoeffs(Kp, Ki,  Kd);
        }

    };


    std::shared_ptr<FrLinkMotorRotationTorque>
    make_motorControlledRotation(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {
        auto motor = std::make_shared<FrLinkMotorRotationTorque>();
        motor->Initialize(node1, node2);
        return motor;
    }



}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
