//
// Created by camille on 29/01/19.
//

#ifndef FRYDOM_FRVARIABLESADDEDMASSBASE_H
#define FRYDOM_FRVARIABLESADDEDMASSBASE_H

#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

    namespace internal {

        class FrVariablesAddedMassBase : public chrono::ChVariablesBodyOwnMass {

        private:

            _FrBodyBase* m_body;
            Eigen::MatrixXd m_infiniteAddedMass;

        public:

            FrVariablesAddedMassBase();

            void Initialize();

            void SetInfiniteAddedMass(const Eigen::MatrixXd InfiniteAddedMass);

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                                                                        const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

        };

    }

}


#endif //FRYDOM_FRADDEDMASSVARIABLE_H
