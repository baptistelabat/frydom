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
    class FrOffshoreSystem;

    class FrFEAMesh : public FrObject {

    protected:

        FrOffshoreSystem * m_system;

        virtual std::shared_ptr<chrono::fea::ChMesh> GetChronoMesh() = 0;

    public:

        FrOffshoreSystem* GetSystem() { return m_system; };

        FrFEAMesh() = default;

        virtual void Update(double time) = 0;

//        virtual void SetupInitial();

        void Initialize() override;

        virtual double GetStaticResidual() = 0;

        virtual void Relax() = 0;

    protected:

        FrPathManager* GetPathManager() const override { return m_system->GetPathManager(); }

        friend void FrOffshoreSystem::AddFEAMesh(std::shared_ptr<FrFEAMesh>);
        friend void FrOffshoreSystem::RemoveFEAMesh(std::shared_ptr<FrFEAMesh>);

    };

} //end namespace frydom
#endif //FRYDOM_FRFEAMESH_H
