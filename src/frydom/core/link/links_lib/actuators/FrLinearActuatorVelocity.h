// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================


#ifndef FRYDOM_FRLINEARACTUATORVELOCITY_H
#define FRYDOM_FRLINEARACTUATORVELOCITY_H

#include "FrLinearActuator.h"

#include "chrono/physics/ChLinkMotorLinearSpeed.h"

namespace frydom {

    // Forward declaration
    class FrLinearActuatorVelocity;

    namespace internal {

        struct FrLinkMotorLinearSpeed : public chrono::ChLinkMotorLinearSpeed {

            FrLinearActuatorVelocity* m_frydomActuator;

            explicit FrLinkMotorLinearSpeed(FrLinearActuatorVelocity* frydomActuator);

            void SetupInitial() override;

//            bool GetDisabled() override;
//
//            void MakeDisabled(bool disabled) override;


        };

    }  // end namespace frydom::internal


    // Forward declaration
    class FrFunctionBase;

    /**
     * \class FrLinearActuatorVelocity
     * \brief Class not used.
     */
    class FrLinearActuatorVelocity : public FrLinearActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorLinearSpeed> m_chronoActuator;


    public:
        explicit FrLinearActuatorVelocity(FrLink* actuatedLink);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearActuatorVelocity"; }

        void SetConstantLinearVelocity(double velocity);

        void SetLinearVelocityFunction(const FrFunctionBase& function);

        void Initialize() override;

        void Update(double time);

        void StepFinalize() override;


        friend void internal::FrLinkMotorLinearSpeed::SetupInitial();


    protected:

        void SetMotorFunction(const FrFunctionBase& function) override;

//        internal::FrLinkMotorLinearSpeed* GetChronoActuator() const override;
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;
        internal::FrLinkMotorLinearSpeed* GetChronoItem_ptr() const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARACTUATORVELOCITY_H
