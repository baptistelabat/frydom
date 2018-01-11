//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRRADIATIONMODEL_H
#define FRYDOM_FRRADIATIONMODEL_H

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/FrObject.h"
#include "FrHydroDB.h"
#include "FrVelocityRecorder.h"

namespace frydom {


    class FrRadiationModel : public FrObject {

    private:

        FrOffshoreSystem* m_system;

        FrHydroDB* m_HDB = nullptr;

        std::vector<FrVelocityRecorder> m_recorders;


    public:

        FrRadiationModel() = default;

        explicit FrRadiationModel(FrHydroDB* HDB) : m_HDB(HDB) {}

        void SetHydroDB(FrHydroDB* HDB) { m_HDB = HDB; }

        FrHydroDB* GetHydroDB() const { return m_HDB; }

        void SetSystem(FrOffshoreSystem* system) { m_system = system; }

        FrOffshoreSystem* GetSystem() const { return m_system; }




        void Initialize() override {

            // Getting the simulation time step
            auto timeStep = m_system->GetStep();

            // Frequency step used in the HDB
            auto freqStep = m_HDB->GetStepFrequency();

            // Maximum usefull time for the impulse response function
            auto Te = 0.5 * MU_2PI / freqStep;

            // Size of recorders that will store Te duration of velocities at timeStep sampling
            auto N = (unsigned int)floor(Te / timeStep);



            // Initialisation of every recorder we need
            auto NbBodies = m_HDB->GetNbBodies();

            m_recorders.reserve(NbBodies);

            std::shared_ptr<FrBEMBody> BEMBody;
            for (unsigned int ibody=0; ibody<NbBodies; ibody++) {

                BEMBody = m_HDB->GetBody(ibody);

//                auto recorder = FrVelocityRecorder();
                FrVelocityRecorder recorder;
                recorder.SetSize(N);

                recorder.Initialize();
//                m_recorders.push_back();


            }



        }



    private:
        unsigned int GetImpulseResponseSize() const {




            auto timeStep = GetSystem()->GetStep();
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



        }


    };



}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
