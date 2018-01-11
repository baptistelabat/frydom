//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRRADIATIONFORCE_H
#define FRYDOM_FRRADIATIONFORCE_H

#include "chrono/physics/ChBody.h"
#include <frydom/core/FrForce.h>
#include <frydom/utils/FrRecorder.h>
#include "frydom/core/FrHydroBody.h"
#include "FrHydroDB.h"

namespace frydom {


//    class FrRadiationModel {
//
//    };


    class FrRadiationForce : public FrForce { // TODO : mettre dans un fichier dedie

    public:
        FrRadiationForce() = default;

    };



    class FrRadiationConvolutionForce : public FrRadiationForce {

    private:



    public:
        void Initialize() {
            // TODO: comment initialiser une fois que toute la modelisation a ete faite ???

            // Getting the body as a FrHydroBody
            auto body = dynamic_cast<FrHydroBody*>(GetBody());

            // TODO: cette partie devrait etre faite une bonne fois pour toute sur la HDB... pas refaire a chaque corst hydro...

            // Getting the simulation time step
            auto timeStep = body->GetSystem()->GetStep();

            // Getting the attached BEMBody
            auto BEMBody = body->GetBEMBody();

            // Getting the discretization in frequency
            auto dw = BEMBody->GetHDB()->GetStepFrequency();

            // The maximum length of the impulse response function
            auto Te = 0.5 * MU_2PI / dw;

            // Minimal size of the recorder
            auto N = (unsigned int)floor(Te / timeStep);

//            // Next power of 2 for the recorder
//            auto N = NextPow2(N);
            // FIXME: Attention, Te est ici uniquement base sur la discretisation de la HDB mais pas sur un temps physique
            // d'extinction... --> on prend certainement trop d'echantillons
            BEMBody->GetHDB()->SetTimeDiscretization(Te, N);  // FIXME : Du coup, on le refait sur la HDB a chaque fois...

            // Initializing circular buffers for the corresponding hydrodynamic body
            body->InitializeVelocityState(N);

            // TODO: FIN TODO

            // Initilializing the impulse response functions for current body
            BEMBody->GenerateImpulseResponseFunctions();

        }

        void UpdateState() override {


        }



    };
//    private:
//        FrRecorder<chrono::ChVector<double>> m_linearVelocityRecorder;
//        FrRecorder<chrono::ChVector<double>> m_angularVelocityRecorder;
////        FrRadiationIRFDB m_IRFDB;
//
//    public:
//
////        FrRadiationConvolutionForce() = default;
//
////        FrRadiationConvolutionForce(FrRadiationIRFDB DB) : m_IRFDB(DB) {}
////
////        void SetRadiationIRFDB(FrRadiationIRFDB DB) {
////            m_IRFDB = DB;
////        }
//
//
//        void UpdateState() override {
//
//            // 1- Recording the current state
//            m_linearVelocityRecorder.record(ChTime, Body->GetPos_dt());
//            m_angularVelocityRecorder.record(ChTime, Body->GetWvel_par());
//
//            // 2- Getting the time history for velocities and interpolating it based on impulse response functions
//            // discretization
//
//
//
//        }



//    };





}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
