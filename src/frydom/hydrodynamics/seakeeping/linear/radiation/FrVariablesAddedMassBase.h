//
// Created by camille on 29/01/19.
//

#ifndef FRYDOM_FRVARIABLESADDEDMASSBASE_H
#define FRYDOM_FRVARIABLESADDEDMASSBASE_H

#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "frydom/core/math/FrMatrix.h"

namespace frydom {

    class FrBEMBody_;

    namespace internal {

        class FrAddedMassBase;

        class FrVariablesAddedMassBase : public chrono::ChVariablesBodyOwnMass {

        private:

            FrAddedMassBase* m_addedMassBase;
            std::unordered_map<FrBEMBody_*, Eigen::MatrixXd> m_invAddedMassCorrection;

        public:

            FrVariablesAddedMassBase(FrAddedMassBase* addedMassBase);

            void Initialize();

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                                                                        const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

            int GetBodyOffset(FrBody_* body) const;

        };

    }

}


#endif //FRYDOM_FRADDEDMASSVARIABLE_H
