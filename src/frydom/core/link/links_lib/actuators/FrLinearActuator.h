//
// Created by lletourn on 07/05/19.
//

#ifndef FRYDOM_FRLINACTUATOR_H
#define FRYDOM_FRLINACTUATOR_H

#include "FrActuator.h"

#include "chrono/physics/ChLinkMotorLinear.h"

namespace frydom {

  // Forward declaration
  class FrLink;

  class FrLinearActuator : public FrActuator {
   private:
    std::shared_ptr<chrono::ChLinkMotorLinear> m_chronoActuator;

   public:
    FrLinearActuator(const std::string &name, FrLink *actuatedLink, ACTUATOR_CONTROL control);

    void SetMotorFunction(const FrFunctionBase &function) override;

    Force GetMotorForceInNode(FRAME_CONVENTION fc) const override;

    Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const override;

    double GetMotorPower() const override;

    void Initialize() override;

//        void StepFinalize() override {};

   protected:

    void DefineLogMessages() override;

    std::shared_ptr<chrono::ChLink> GetChronoLink() override;

    chrono::ChLinkMotorLinear *GetChronoItem_ptr() const override;


  };

} //end namespace frydom


#endif //FRYDOM_FRLINACTUATOR_H
