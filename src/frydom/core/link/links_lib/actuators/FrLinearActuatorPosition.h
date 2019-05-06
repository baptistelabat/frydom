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


#ifndef FRYDOM_FRLINEARACTUATORPOSITION_H
#define FRYDOM_FRLINEARACTUATORPOSITION_H

#include "FrLinearActuator.h"

#include "chrono/physics/ChLinkMotorLinearPosition.h"

namespace frydom {

    // Forward declaration
    class FrLinearActuatorPosition;

    namespace internal {

        struct FrLinkMotorLinearPosition : public chrono::ChLinkMotorLinearPosition {

            FrLinearActuatorPosition* m_frydomActuator;

            explicit FrLinkMotorLinearPosition(FrLinearActuatorPosition* frydomActuator);

            void SetupInitial() override;

//            bool GetDisabled() override;
//
//            void MakeDisabled(bool disabled) override;


        };

    }  // end namespace frydom::internal


    // Forward declaration
    class FrFunctionBase;

    /**
     * \class FrLinearActuatorPosition
     * \brief Class not used.
     */
    class FrLinearActuatorPosition : public FrLinearActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorLinearPosition> m_chronoActuator;


    public:
        explicit FrLinearActuatorPosition(FrLink* actuatedLink);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "LinearActuatorPosition"; }

        void SetConstantLinearPosition(double position);

        void SetLinearPositionFunction(const FrFunctionBase& function);

        void Initialize() override;

        void Update(double time);

        void StepFinalize() override;


        friend void internal::FrLinkMotorLinearPosition::SetupInitial();


    protected:

        void SetMotorFunction(const FrFunctionBase& function) override;

//        internal::FrLinkMotorLinearPosition* GetChronoActuator() const override;
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;
        internal::FrLinkMotorLinearPosition* GetChronoItem_ptr() const override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRLINEARACTUATORPOSITION_H
