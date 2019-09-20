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

#include "chrono/physics/ChPhysicsItem.h"

#include "frydom/asset/FrAssetOwner.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/misc/FrColors.h"


namespace frydom {

    template <typename OffshoreSystemType>
    class FrPhysicsItem;


    namespace internal {

        template <typename OffshoreSystemType>
        struct FrPhysicsItemBase : public chrono::ChPhysicsItem {

            FrPhysicsItem<OffshoreSystemType> *m_frydomPhysicsItem;

            /// Constructor.
            explicit FrPhysicsItemBase(FrPhysicsItem<OffshoreSystemType> *item);

            void SetupInitial() override;

            void Update(double time, bool update_assets) override;

            friend class FrPhysicsItem_;

        };

    }  // end namespace frydom::internal


    template <typename OffshoreSystemType>
    class FrOffshoreSystem;
    class FrTriangleMeshConnected;
    class FrAsset;

    /**
     * \class FrPhysicsItem
     * \brief Class for defining objects which are neither bodies nor links, for instance caterany lines.
     */
     template <typename OffshoreSystemType>
    class FrPhysicsItem: public FrObject<OffshoreSystemType> {

    protected:

        std::shared_ptr<internal::FrPhysicsItemBase<OffshoreSystemType>>
                m_chronoPhysicsItem;     ///> pointer to the related chrono physics item

        FrOffshoreSystem<OffshoreSystemType>* m_system;     ///< pointer to the system containing this physics item

        bool m_isActive = true;         ///< boolean to check if the physics item is active
                                        ///< if it's not the case, it is not updated during the simulation

        /// Get the shared pointer to the chrono related physics item
        /// \return Chrono related physics item
        virtual std::shared_ptr<internal::FrPhysicsItemBase<OffshoreSystemType>> GetChronoPhysicsItem() const ;

        friend void FrOffshoreSystem<OffshoreSystemType>::RemovePhysicsItem(std::shared_ptr<FrPhysicsItem<OffshoreSystemType>>);

    public:

        FrPhysicsItem();

        FrOffshoreSystem<OffshoreSystemType>* GetSystem();

        /// Check if the force is active
        bool IsActive() const;

        /// Activate or deactivate the force
        void SetActive(bool active);

        /// Return true if the physics item is included in the static analysis
        virtual bool IncludedInStaticAnalysis() const {return true;}

        void Update(double time);

        virtual void SetupInitial();

        void Initialize() override {};

    private:

        /// Virtual function to allow updating the child object from the solver
        /// \param time Current time of the simulation from beginning, in seconds
        virtual void Compute(double time) = 0;

    };

    /**
     * \class FrPrePhysicsItem
     * \brief Class for defining physics items updated before bodies.
     */
     template <typename OffshoreSystemType>
    class FrPrePhysicsItem : public FrPhysicsItem<OffshoreSystemType> {
    protected:
        friend void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem<OffshoreSystemType>>);
    };

    /**
     * \class FrMidPhysicsItem
     * \brief Class for defining physics items updated after bodies but before links.
     */
     template <typename OffshoreSystemType>
    class FrMidPhysicsItem : public FrPhysicsItem<OffshoreSystemType> {
    protected:
        friend void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem<OffshoreSystemType>>);
    };

    /**
     * \class FrPostPhysicsItem
     * \brief Class for defining physics items updated after links.
     */
     template <typename OffshoreSystemType>
    class FrPostPhysicsItem : public FrPhysicsItem<OffshoreSystemType> {
    protected:
        friend void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem<OffshoreSystemType>>);
    };

}  // end namespace frydom


#endif //FRYDOM_FRPHYSICSITEM_H
