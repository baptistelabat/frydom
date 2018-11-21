//
// Created by camille on 20/11/18.
//

#ifndef FRYDOM_FRPHYSICSITEM_H
#define FRYDOM_FRPHYSICSITEM_H

#include <chrono/physics/ChPhysicsItem.h>
#include "FrObject.h"

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


    };

}


#endif //FRYDOM_FRPHYSICSITEM_H
