//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H

#include <chrono/physics/ChLinkMotorLinearSpeed.h>
#include "chrono/physics/ChLinkMate.h"

#include "FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {

    // Forward declaration
    class FrLink_;

    struct _FrLinkBase : public chrono::ChLink {

        FrLink_* m_frydomLink;

        explicit _FrLinkBase(FrLink_* link);

        void SetupInitial() override;

        void Update(bool update_assets) override;

    };

    // Forward declaration
    class FrOffshoreSystem_;
    class FrNode_;

    class FrLink_ : public FrObject {

    protected:

        std::shared_ptr<_FrLinkBase> m_chronoLink;

        FrOffshoreSystem_* m_system;


    public:

        FrLink_();

        FrOffshoreSystem_* GetSystem();

        void SetName(const char name[]);

        std::string GetName() const;

        virtual void Update() = 0;

    protected:

        virtual std::shared_ptr<chrono::ChLink> GetChronoLink() const { return m_chronoLink;}

        friend void FrOffshoreSystem_::AddLink(std::shared_ptr<FrLink_>);

    };
//
//
//
//    struct _FrLinearActuatorBase : public chrono::ChLinkMotorLinearSpeed {
//
//        FrLink_* m_frydomLink;
//
//        explicit _FrLinearActuatorBase(FrLink_* link);
//
//        void SetupInitial() override;
//
//        void Update(bool update_assets) override;
//
//    };












}  // end namesapce frydom


#endif //FRYDOM_FRLINK_H
