//
// Created by frongere on 06/02/19.
//

#ifndef FRYDOM_FRACTUATOR_H
#define FRYDOM_FRACTUATOR_H

#include <memory>

#include "frydom/core/link/FrLinkBase.h"

namespace frydom {


//    namespace internal {
//
//        struct FrMotorBase {
//
//            virtual bool GetDisabled() = 0;
//            virtual void MakeDisabled(bool disabled) = 0;
//
//        };
//
//    }  // end namespace frydom::internal


    // Forward declaration
    class FrLink;
    class FrFunctionBase;

    class FrActuator : public FrLinkBase {

    protected:
//        std::shared_ptr<internal::FrMotorBase> m_chronoMotor;

        FrLink* m_actuatedLink;


    public:
        FrActuator(FrLink* actuatedLink);


        // TODO : ajouter des methodes communes a tous les actuateurs tel que GetPower() ...

        /// Tells if all constraints of this link are currently turned on or off by the user.
        bool IsDisabled() const override;

        /// User can use this to enable/disable all the constraint of the link as desired.
        void SetDisabled(bool disabled) override;

//        /// Tells if the link is broken, for excess of pulling/pushing.
//        virtual bool IsBroken() const override;
//
//        /// Set the 'broken' status vof this link.
//        virtual void SetBroken(bool broken) override;

        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        bool IsActive() const override;

        std::string GetTypeName() const override { return "Actuator"; }

        virtual void SetMotorFunction(const FrFunctionBase& function) = 0;

    protected:

//        virtual internal::FrMotorBase* GetChronoActuator() const = 0;

        chrono::ChLinkBase* GetChronoItem_ptr() const override = 0;

    };


}  // end namespace frydom


#endif //FRYDOM_FRACTUATOR_H
