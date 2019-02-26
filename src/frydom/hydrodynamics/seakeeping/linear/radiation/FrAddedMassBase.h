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


#ifndef FRYDOM_FRADDEDMASSBASE_H
#define FRYDOM_FRADDEDMASSBASE_H

//#include "frydom/core/common/FrPhysicsItem.h"

namespace frydom {


    // Forward declarations
    class FrRadiationModel_;
    class FrBody_;


    namespace internal {


        class FrVariablesAddedMassBase;

        class FrAddedMassBase : public _FrPhysicsItemBase {

        private:

            FrRadiationModel_* m_frydomRadiationModel;
            std::shared_ptr<FrVariablesAddedMassBase> m_variables;

        public:

            /// Constructor of the class.
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

            int GetBodyOffset(FrBody_* body) const;

            void SetVariables(FrBody_* body, chrono::ChMatrix<double>& qb, int offset) const;

            FrRadiationModel_* GetRadiationModel() const { return m_frydomRadiationModel; }
        };
    }


}  // end namespace frydom::internal


#endif //FRYDOM_FRADDEDMASSBASE_H
