//
// Created by frongere on 24/08/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H


#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"

#include "FrNode.h"

namespace frydom {



    class FrActuator  : public FrObject { // TODO mettre dans son propre fichier !!
        // TODO: cette classe de base devra servir pour introduire l'interactivite ...

        void Initialize() override {

        }

        void StepFinalize() override {

        }

    };



    class FrLinkMotorRotationAngle : public FrActuator, public chrono::ChLinkMotorRotationAngle {

    public:

        FrLinkMotorRotationAngle() : chrono::ChLinkMotorRotationAngle() {


        }

        void Initialize(std::shared_ptr<FrNode> node1, std::shared_ptr<FrNode> node2) {

        }

    };




    class FrLinkMotorRotationSpeed : public FrActuator, public chrono::ChLinkMotorRotationSpeed {

    public:

        FrLinkMotorRotationSpeed() : chrono::ChLinkMotorRotationSpeed() {
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




}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
