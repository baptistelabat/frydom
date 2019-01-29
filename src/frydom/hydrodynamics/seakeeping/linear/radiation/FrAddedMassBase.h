//
// Created by camille on 29/01/19.
//

#ifndef FRYDOM_FRADDEDMASSBASE_H
#define FRYDOM_FRADDEDMASSBASE_H

#include "chrono/physics/ChPhysicsItem.h"
#include "FrVariablesAddedMassBase.h"

namespace frydom {

    namespace internal {

        class FrAddedMassBase : public chrono::ChPhysicsItem {

        private:

            std::shared_ptr<FrVariablesAddedMassBase> m_variables;

        public:

            void SetupInitial() override;

            void Update(bool update_assets) override;

            void Update(double time, bool update_assets) override;

        };
    }
}


#endif //FRYDOM_FRADDEDMASSBASE_H
