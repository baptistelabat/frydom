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


#ifndef FRYDOM_FRANGULARACTUATORANGLE_H
#define FRYDOM_FRANGULARACTUATORANGLE_H

#include "FrAngularActuator.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"

namespace frydom {

    // Forward declaration
    class FrAngularActuatorAngle;

    namespace internal {

        struct FrLinkMotorRotationAngle : public chrono::ChLinkMotorRotationAngle {

            FrAngularActuatorAngle* m_frydomActuator;

            explicit FrLinkMotorRotationAngle(FrAngularActuatorAngle* frydomActuator);

            void SetupInitial() override;

//            bool GetDisabled() override;
//
//            void MakeDisabled(bool disabled) override;


        };

    }  // end namespace frydom::internal


    // Forward declaration
    class FrFunctionBase;

    /**
     * \class FrAngularActuatorAngle
     * \brief Class not used.
     */
    class FrAngularActuatorAngle : public FrAngularActuator {

    private:
        std::shared_ptr<internal::FrLinkMotorRotationAngle> m_chronoActuator;


    public:
        explicit FrAngularActuatorAngle(FrLink* actuatedLink);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "AngularActuatorAngle"; }

        void SetConstantAngularAngle(double velocity);

        void SetAngularAngleFunction(const FrFunctionBase &function);

        void Initialize() override;

        void Update(double time);


        friend void internal::FrLinkMotorRotationAngle::SetupInitial();


    protected:

        void SetMotorFunction(const FrFunctionBase& function) override;

//        internal::FrLinkMotorRotationAngle* GetChronoActuator() const override;
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;
        internal::FrLinkMotorRotationAngle* GetChronoItem_ptr() const override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRANGULARACTUATORANGLE_H
