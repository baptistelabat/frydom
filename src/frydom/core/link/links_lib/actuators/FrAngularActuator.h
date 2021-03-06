//
// Created by lletourn on 07/05/19.
//

#ifndef FRYDOM_FRANGACTUATOR_H
#define FRYDOM_FRANGACTUATOR_H


#include "FrActuator.h"

#include "chrono/physics/ChLinkMotorRotation.h"

namespace frydom {

  // Forward declaration
  class FrLink;

  class FrAngularActuator : public FrActuator {
   private:
    std::shared_ptr<chrono::ChLinkMotorRotation> m_chronoActuator;

   public:
    FrAngularActuator(const std::string &name, FrLink *actuatedLink, ACTUATOR_CONTROL control);

    void SetMotorFunction(const FrFunctionBase &function) override;

    Force GetMotorForceInNode(FRAME_CONVENTION fc) const override;

    Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const override;

    double GetMotorPower() const override;

    void Initialize() override;

    void StepFinalize() override {};

   protected:

    void DefineLogMessages() override;

    std::shared_ptr<chrono::ChLink> GetChronoLink() override;

    chrono::ChLinkMotorRotation *GetChronoItem_ptr() const override;


  };

} //end namespace frydom

#endif //FRYDOM_FRANGACTUATOR_H
