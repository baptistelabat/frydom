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


#ifndef FRYDOM_FRPHYSICSITEM_H
#define FRYDOM_FRPHYSICSITEM_H

#include "FrObject.h"
#include "frydom/core/misc/FrColors.h"
#include "frydom/core/FrOffshoreSystem.h"
#include <chrono/physics/ChPhysicsItem.h>

namespace frydom {

    class FrPhysicsItem_;


    namespace internal {

        struct _FrPhysicsItemBase : public chrono::ChPhysicsItem {

            FrPhysicsItem_ *m_frydomPhysicsItem;

            /// Constructor.
            explicit _FrPhysicsItemBase(FrPhysicsItem_ *item);

            void SetupInitial() override;

            void Update(bool update_assets) override;           // TODO : non nécessaire

            void Update(double time, bool update_assets) override;

            friend class FrPhysicsItem_;

        };

    }  // end namespace frydom::internal


    class FrOffshoreSystem_;
    class FrTriangleMeshConnected;

    /**
     * \class FrPhysicsItem_
     * \brief Class for defining objects which are neither bodies nor links, for instance caterany lines.
     */
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

        void AddMeshAsset(std::shared_ptr<frydom::FrTriangleMeshConnected> mesh);

        void SetColor(NAMED_COLOR colorName);

        void SetColor(const FrColor& color);

        virtual void SetupInitial();

        void Initialize() override {};
    protected:

        virtual std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() const ;

    };

    /**
     * \class FrPrePhysicsItem_
     * \brief Class for defining physics items updated before bodies.
     */
    class FrPrePhysicsItem_ : public FrPhysicsItem_{
    protected:
        friend void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem_>);
    };

    /**
     * \class FrMidPhysicsItem_
     * \brief Class for defining physics items updated after bodies but before links.
     */
    class FrMidPhysicsItem_ : public FrPhysicsItem_{
    protected:
        friend void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem_>);
    };

    /**
     * \class FrPostPhysicsItem_
     * \brief Class for defining physics items updated after links.
     */
    class FrPostPhysicsItem_ : public FrPhysicsItem_{
    protected:
        friend void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem_>);
    };

}  // end namespace frydom


#endif //FRYDOM_FRPHYSICSITEM_H
