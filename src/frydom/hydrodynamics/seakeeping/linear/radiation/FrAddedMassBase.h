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
        class FrBEMBody_;

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

            void IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta& v,
                                 const chrono::ChVectorDynamic<>& R, const unsigned int off_L,
                                 const chrono::ChVectorDynamic<>& L, const chrono::ChVectorDynamic<>& Qc) override;

            void IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta& v,
                                   const unsigned int off_L, chrono::ChVectorDynamic<>& L) override;

            void InjectVariables(chrono::ChSystemDescriptor& mdescriptor) override;

            void VariablesFbReset() override;

            void VariablesFbIncrementMq() override;

            // IntStateGatherAcceleration TODO : ??//
            // IntStateScatterAcceleration TODO : ??//
            // IntStateIncrement TODO : ??//

            FrHydroDB_* GetHDB() const { return m_HDB; }

            int GetBodyOffset(FrBEMBody_* BEMBody) const;

        };
    }
}


#endif //FRYDOM_FRADDEDMASSBASE_H
