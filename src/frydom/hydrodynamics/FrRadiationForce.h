//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRRADIATIONFORCE_H
#define FRYDOM_FRRADIATIONFORCE_H

#include "chrono/physics/ChBody.h"

#include <frydom/core/FrForce.h>
#include <frydom/utils/FrRecorder.h>
#include "frydom/core/FrHydroBody.h"

#include "FrRadiationModel.h"
#include "FrHydroDB.h"

namespace frydom {

    // TODO: les forces de radiation doivent pouvoir etre extraites depuis le modele de radiation...

    class FrRadiationForce : public FrForce {

        // FIXME : enum a placer dans RadiationModel
//        enum class Type {
//            CONVOLUTION,
//            STATE_SPACE // Not used...
//        };  // TODO : utiliser !!!

    protected:
        std::shared_ptr<FrRadiationModel> m_radiationModel;  // TODO : il faut que le modele de radiation soit en mesure de generer les forces de radiation
        // Une possibilite serait qu'a l'initialisation du modele de radiation, les forces de radiation soient ajoutees automatiquemet aux corps...

    public:
        FrRadiationForce() = default;

        explicit FrRadiationForce(std::shared_ptr<FrRadiationModel> radiationModel) : m_radiationModel(radiationModel) {}

        void SetRadiationModel(const std::shared_ptr<FrRadiationModel> radiationModel) { m_radiationModel = radiationModel; }

        std::shared_ptr<FrRadiationModel> GetRadiationModel() const { return m_radiationModel; }

    };


    class FrRadiationConvolutionForce : public FrRadiationForce {

    private:



    public:

        explicit FrRadiationConvolutionForce(std::shared_ptr<FrRadiationConvolutionModel> radiationConvolutionModel)
                : FrRadiationForce(radiationConvolutionModel) {}

        void Initialize() override {
            // TODO: comment initialiser une fois que toute la modelisation a ete faite ???

            // Getting the body as a FrHydroBody
//            auto body = dynamic_cast<FrHydroBody*>(GetBody());
//
//            // TODO: cette partie devrait etre faite une bonne fois pour toute sur la HDB... pas refaire a chaque corst hydro...
//
//            // Getting the simulation time step
//            auto timeStep = body->GetSystem()->GetStep();
//
//            // Getting the attached BEMBody
//            auto BEMBody = body->GetBEMBody();
//
//            // Getting the discretization in frequency
//            auto dw = BEMBody->GetHDB()->GetStepFrequency();
//
//            // The maximum length of the impulse response function
//            auto Te = 0.5 * MU_2PI / dw;
//
//            // Minimal size of the recorder
//            auto N = (unsigned int)floor(Te / timeStep);
//
////            // Next power of 2 for the recorder
////            auto N = NextPow2(N);
//            // FIXME: Attention, Te est ici uniquement base sur la discretisation de la HDB mais pas sur un temps physique
//            // d'extinction... --> on prend certainement trop d'echantillons
//            BEMBody->GetHDB()->SetTimeDiscretization(Te, N);  // FIXME : Du coup, on le refait sur la HDB a chaque fois...
//
//            // Initializing circular buffers for the corresponding hydrodynamic body
////            body->InitializeVelocityState(N);
//
//            // TODO: FIN TODO
//
//            // Initilializing the impulse response functions for current body
//            BEMBody->GenerateImpulseResponseFunctions();

        }

        void UpdateState() override {
            // TODO: appeler le Update du RadiationModel

            // Force dans le repere absolu, mmoment dans le repere relatif
//            m_radiationModel->Get
//            force =
        }



    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
