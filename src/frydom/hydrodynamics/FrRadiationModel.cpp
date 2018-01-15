//
// Created by frongere on 11/01/18.
//

#include "FrRadiationModel.h"

#include "FrRadiationForce.h"


namespace frydom {


    std::shared_ptr<FrRadiationConvolutionForce>
    FrRadiationConvolutionModel::AddRadiationForceToHydroBody(std::shared_ptr<FrHydroBody> hydroBody) {

        // Getting the bem index of the body
        auto iBEMBody = m_system->GetHydroMapper()->GetBEMBodyIndex(hydroBody);

        auto radiationForce = std::make_shared<FrRadiationConvolutionForce>(shared_from_this());

        // Adding the force to the body
        hydroBody->AddForce(radiationForce);

        return radiationForce;
    }
}