//
// Created by lletourn on 06/03/19.
//

#ifndef FRYDOM_FRFEAMESH_H
#define FRYDOM_FRFEAMESH_H

#include "frydom/asset/FrAssetOwner.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"

namespace chrono{
    namespace fea {
        class ChMesh;
    }
}

namespace frydom {

    // Forward declarations
    template <typename OffshoreSystemType>
    class FrOffshoreSystem;

    template <typename OffshoreSystemType>
    class FrFEAMesh : public FrObject<OffshoreSystemType> {

    protected:

        FrOffshoreSystem<OffshoreSystemType> * m_system;

        virtual std::shared_ptr<chrono::fea::ChMesh> GetChronoMesh() = 0;

    public:

        FrOffshoreSystem<OffshoreSystemType>* GetSystem() { return m_system; };

        FrFEAMesh() = default;

        virtual void Update(double time) = 0;

//        virtual void SetupInitial();

        void Initialize() override;

        virtual double GetStaticResidual() = 0;

        virtual void Relax() = 0;

        friend void FrOffshoreSystem<OffshoreSystemType>::AddFEAMesh(std::shared_ptr<FrFEAMesh<OffshoreSystemType>>);
        friend void FrOffshoreSystem<OffshoreSystemType>::RemoveFEAMesh(std::shared_ptr<FrFEAMesh<OffshoreSystemType>>);

    };

} //end namespace frydom
#endif //FRYDOM_FRFEAMESH_H
