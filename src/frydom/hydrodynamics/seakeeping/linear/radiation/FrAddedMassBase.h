//
// Created by camille on 29/01/19.
//

#ifndef FRYDOM_FRADDEDMASSBASE_H
#define FRYDOM_FRADDEDMASSBASE_H

#include "chrono/physics/ChPhysicsItem.h"
#include "FrVariablesAddedMassBase.h"

namespace frydom {

    namespace internal {

        class FrHydroDB_;

        class FrAddedMassBase : public chrono::ChPhysicsItem {

        private:

            FrHydroDB_* m_HDB;
            std::shared_ptr<FrVariablesAddedMassBase> m_variables;

        public:

            FrAddedMassBase(FrHydroDB_* HDB);

            void SetupInitial() override;

            void Update(bool update_assets) override;

            void Update(double time, bool update_assets) override;

            void IntLoadResidual_Mv(const unsigned int off,
                                    chrono::ChVectorDynamic<>& R,
                                    const chrono::ChVectorDynamic<>& w,
                                    const double c
            ) override;

            void InjectVariables(chrono::ChSystemDescriptor& mdescriptor) override;

        };
    }
}


#endif //FRYDOM_FRADDEDMASSBASE_H
