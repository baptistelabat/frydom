//
// Created by camille on 29/01/19.
//

#ifndef FRYDOM_FRADDEDMASSBASE_H
#define FRYDOM_FRADDEDMASSBASE_H

#include "frydom/core/common/FrPhysicsItem.h"

namespace frydom {

    class FrRadiationModel_;
    class FrBody_;

    namespace internal {

        // forward declaration
        //class FrBEMBody_;
        class FrVariablesAddedMassBase;

        class FrAddedMassBase : public _FrPhysicsItemBase {

        private:

            FrRadiationModel_* m_frydomRadiationModel;
            std::shared_ptr<FrVariablesAddedMassBase> m_variables;

        public:

            explicit FrAddedMassBase(FrRadiationModel_* radiationModel);

            //
            // Update
            //

            void SetupInitial() override;

            void Update(bool update_assets) override;

            void Update(double time, bool update_assets) override;

            //
            // Descriptor
            //

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

            int GetBodyOffset(FrBody_* body) const;

            FrRadiationModel_* GetRadiationModel() const { return m_frydomRadiationModel; }

            //void SetSystem(FrOffshoreSystem_* system);
            //void SetSystem(chrono::ChSystem* system);

        };
    }
}


#endif //FRYDOM_FRADDEDMASSBASE_H
