//
// Created by camille on 20/11/18.
//

#ifndef FRYDOM_FRPHYSICSITEM_H
#define FRYDOM_FRPHYSICSITEM_H

#include "FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include <chrono/physics/ChPhysicsItem.h>

namespace frydom {

    class FrPhysicsItem_;


    namespace internal {

        struct _FrPhysicsItemBase : public chrono::ChPhysicsItem {

            FrPhysicsItem_ *m_frydomPhysicsItem;

            explicit _FrPhysicsItemBase(FrPhysicsItem_ *item);

            void SetupInitial() override;

            void Update(bool update_assets) override;

            friend class FrPhysicsItem_;

        };

    }



    class FrOffshoreSystem_;

    class FrPhysicsItem_: public FrObject {

    protected:
        std::shared_ptr<internal::_FrPhysicsItemBase> m_chronoPhysicsItem;

        FrOffshoreSystem_* m_system;

    public:

        FrPhysicsItem_();

        FrOffshoreSystem_* GetSystem();

        void SetName(const char name[]);

        std::string GetName() const;

        virtual void Update(double time) = 0;

    protected:

        virtual std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() const ;

        friend void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrPhysicsItem_>);




    };

}


#endif //FRYDOM_FRPHYSICSITEM_H
