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

#include "MathUtils/Matrix66.h"

#include "chrono/solver/ChVariables.h"


namespace frydom {

    // Forward declaration
    class FrBody;
    class FrBEMBody;

    namespace internal {

        /*struct pair_hash {
            template <class T1, class T2>
            std::size_t operator () (const std::pair<T1,T2> &p) const {
                auto h1 = std::hash<T1>{}(p.first);
                auto h2 = std::hash<T2>{}(p.second);

                // Mainly for demonstration purposes, i.e. works but is overly simple
                // In the real world, use sth. like boost.hash_combine
                return h1 ^ h2;
            }
        };*/


        // Forward declaration
        class FrRadiationModelBase;

        class FrVariablesAddedMassBase : public chrono::ChVariables {

        private:

            FrRadiationModelBase* m_addedMassBase;
            std::unordered_map<std::pair<FrBEMBody*, FrBEMBody*>, mathutils::Matrix66<double>, pair_hash> m_invAddedMassCorrection;

        public:

            FrVariablesAddedMassBase(FrRadiationModelBase* addedMassBase, int ndof);

            void Initialize();

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                                                                        const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

            int GetBodyOffset(FrBody* body) const;

            void SetVariables(FrBody* body, chrono::ChMatrix<double>& result, int offset) const;

            chrono::ChMatrix<double> GetVariablesFb(FrBody* body) const;

            chrono::ChMatrix<double> GetVariablesQb(FrBody* body) const;

        };

    }  // end namespace frydom::internal

}  // end namespace frydom


#endif //FRYDOM_FRADDEDMASSVARIABLE_H
