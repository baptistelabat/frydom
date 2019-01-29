//
// Created by camille on 29/01/19.
//

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"

namespace frydom {

    namespace internal {

        FrVariablesAddedMassBase::FrVariablesAddedMassBase() {

        }

        void FrVariablesAddedMassBase::SetInfiniteAddedMass(const Eigen::MatrixXd infiniteAddedMass) {
            m_infiniteAddedMass = infiniteAddedMass;
        }

        void FrVariablesAddedMassBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                        const chrono::ChMatrix<double>& vect) const {

        }

        void FrVariablesAddedMassBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                            const chrono::ChMatrix<double>& vect) const {

        }

        void FrVariablesAddedMassBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vect) const {

        }



    }
}