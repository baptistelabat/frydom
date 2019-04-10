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

#include <memory>

#include "frydom/core/common/FrPhysicsItem.h"



namespace frydom {


    // Forward declarations
    class FrRadiationModel;
    class FrBody;


    namespace internal {


        class FrVariablesAddedMassBase;

        class FrAddedMassBase : public FrPhysicsItemBase {

        private:

            FrRadiationModel* m_frydomRadiationModel;
            std::shared_ptr<FrVariablesAddedMassBase> m_variables;

        public:

            /// Constructor of the class.
            explicit FrAddedMassBase(FrRadiationModel* radiationModel);

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

            int GetBodyOffset(FrBody* body) const;

            void SetVariables(FrBody* body, chrono::ChMatrix<double>& qb, int offset) const;

            FrRadiationModel* GetRadiationModel() const { return m_frydomRadiationModel; }
        };


    } // end namespace internal

}  // end namespace frydom


#endif //FRYDOM_FRADDEDMASSBASE_H
