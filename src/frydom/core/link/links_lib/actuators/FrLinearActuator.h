//
// Created by lletourn on 07/05/19.
//

#ifndef FRYDOM_FRLINACTUATOR_H
#define FRYDOM_FRLINACTUATOR_H

#include "FrActuator.h"

#include "chrono/physics/ChLinkMotorLinear.h"

namespace frydom {

    // Forward declaration
    template <typename OffshoreSystemType>
    class FrLink;

    template <typename OffshoreSystemType>
    class FrLinearActuator : public FrActuator<OffshoreSystemType> {
    private:
        std::shared_ptr<chrono::ChLinkMotorLinear> m_chronoActuator;

    public:
        explicit FrLinearActuator(FrLink<OffshoreSystemType> *actuatedLink, ACTUATOR_CONTROL control);

        void SetMotorFunction(const FrFunctionBase& function) override;

        Force GetMotorForceInNode(FRAME_CONVENTION fc) const override;

        Torque GetMotorTorqueInNode(FRAME_CONVENTION fc) const override;

        double GetMotorPower() const override;

        void Initialize() override;

//        void StepFinalize() override {};

    protected:

        std::shared_ptr<chrono::ChLink> GetChronoLink() override;
        chrono::ChLinkMotorLinear* GetChronoItem_ptr() const override;


    };

} //end namespace frydom


#endif //FRYDOM_FRLINACTUATOR_H
