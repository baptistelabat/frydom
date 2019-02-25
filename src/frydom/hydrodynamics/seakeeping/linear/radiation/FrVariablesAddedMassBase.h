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


#ifndef FRYDOM_FRVARIABLESADDEDMASSBASE_H
#define FRYDOM_FRVARIABLESADDEDMASSBASE_H

#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "frydom/core/math/FrMatrix.h"

namespace frydom {

    class FrBEMBody_;

    namespace internal {

        class FrAddedMassBase;

        class FrVariablesAddedMassBase : public chrono::ChVariables {

        private:

            FrAddedMassBase* m_addedMassBase;
            std::unordered_map<FrBEMBody_*, mathutils::Matrix66<double>> m_invAddedMassCorrection;

        public:

            FrVariablesAddedMassBase(FrAddedMassBase* addedMassBase, int ndof);

            void Initialize();

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                                                                        const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

            int GetBodyOffset(FrBody_* body) const;

            void SetVariables(FrBody_* body, chrono::ChMatrix<double>& result, int offset) const;

            chrono::ChMatrix<double> GetVariablesFb(FrBody_* body) const;

            chrono::ChMatrix<double> GetVariablesQb(FrBody_* body) const;

        };

    }

}


#endif //FRYDOM_FRADDEDMASSVARIABLE_H
