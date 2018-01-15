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

//    class FrRadiationModel;

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


    // Forward declaration
//    class FrRadiationConvolutionModel;

    class FrRadiationConvolutionForce : public FrRadiationForce {

    private:



    public:

        explicit FrRadiationConvolutionForce(std::shared_ptr<FrRadiationConvolutionModel> radiationConvolutionModel)
                : FrRadiationForce(radiationConvolutionModel) {}

        void Initialize() override {
            m_radiationModel->Initialize();
        }

        void UpdateState() override {
            // TODO: appeler le Update du RadiationModel
            m_radiationModel->Update(ChTime);  // TODO: verifier que le ChTime est le bon temps courant !!

            // Current Hydrodynamic body
            auto hydroBody = dynamic_cast<FrHydroBody*>(GetBody());

            // Get the forces
            m_radiationModel->GetRadiationForce(hydroBody, force, moment);

            moment = hydroBody->Dir_World2Body(moment);  // Moment expressed in the local coordinate frame
            // TODO: verifier que c'est la bonne fonction

            // moment in local
//            force = m_radiationModel->GetRadiationForce(hydroBody);
//            moment = m_radiationModel->GetRadiationMoment(hydroBody);
        }


    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
