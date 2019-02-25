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


#include "frydom/utils/FrEigen.h"
#include "FrVariablesBEMBodyMass.h"

namespace frydom {

    FrVariablesBEMBodyMass::FrVariablesBEMBodyMass() {
        m_InfiniteAddedMass = Eigen::MatrixXd::Zero(6,6);
        m_GeneralizedMass = Eigen::MatrixXd::Zero(6,6);
        m_GeneralizedMass(0,0) = 1.;
        m_GeneralizedMass(1,1) = 1.;
        m_GeneralizedMass(2,2) = 1.;
        m_GeneralizedMass(3,3) = 1.;
        m_GeneralizedMass(4,4) = 1.;
        m_GeneralizedMass(5,5) = 1.;
        m_invGeneralizedMass = m_GeneralizedMass.inverse();
    }

    void FrVariablesBEMBodyMass::Initialize(const ChVariablesBodyOwnMass& variables) {

        ChVariablesBodyOwnMass::operator=(variables);

        m_InfiniteAddedMass = Eigen::MatrixXd::Zero(6,6);
        SetGeneralizedMass();
    }

    void FrVariablesBEMBodyMass::SetInfiniteAddedMass(const Eigen::MatrixXd InfiniteAddedMass) {
        m_InfiniteAddedMass = InfiniteAddedMass;
        SetGeneralizedMass();
    }

    void FrVariablesBEMBodyMass::SetGeneralizedMass() {
        m_GeneralizedMass = m_InfiniteAddedMass;
        m_GeneralizedMass(0,0) += GetBodyMass();
        m_GeneralizedMass(1,1) += GetBodyMass();
        m_GeneralizedMass(2,2) += GetBodyMass();
        m_GeneralizedMass.block<3,3>(3,3) += ChEig(GetBodyInertia());
        m_invGeneralizedMass = m_GeneralizedMass.inverse();
    }

    void FrVariablesBEMBodyMass::Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            result(i) = 0;
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_invGeneralizedMass(i, j) * vect(j);
            }
        }
    }

    void FrVariablesBEMBodyMass::Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_invGeneralizedMass(i, j) * vect(j);
            }
        }
    }

    void FrVariablesBEMBodyMass::Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_GeneralizedMass(i, j) * vect(j);
            }
        }

    }

// Computes the product of the corresponding block in the
// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
// NOTE: the 'vect' and 'result' vectors must already have
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offsets (that must be already updated) to know the
// indexes in result and vect.
    void FrVariablesBEMBodyMass::MultiplyAndAdd(chrono::ChMatrix<double>& result,
                                                const chrono::ChMatrix<double>& vect,
                                                const double c_a) const {
        assert(result.GetColumns() == 1 && vect.GetColumns() == 1);
        //
        for (int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
                result(this->offset + i) += c_a * m_GeneralizedMass(i, j) * vect(this->offset + j);
            }
        }
    }

// Add the diagonal of the mass matrix scaled by c_a to 'result'.
// NOTE: the 'result' vector must already have the size of system unknowns, ie
// the size of the total variables&constraints in the system; the procedure
// will use the ChVariable offset (that must be already updated) as index.
    void FrVariablesBEMBodyMass::DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const {
        assert(result.GetColumns() == 1);
        result(this->offset + 0) += c_a * m_GeneralizedMass(0, 0);
        result(this->offset + 1) += c_a * m_GeneralizedMass(1, 1);
        result(this->offset + 2) += c_a * m_GeneralizedMass(2, 2);
        result(this->offset + 3) += c_a * m_GeneralizedMass(3, 3);
        result(this->offset + 4) += c_a * m_GeneralizedMass(4, 4);
        result(this->offset + 5) += c_a * m_GeneralizedMass(5, 5);
    }

// Build the mass matrix (for these variables) scaled by c_a, storing
// it in 'storage' sparse matrix, at given column/row offset.
// Note, most iterative solvers don't need to know mass matrix explicitly.
// Optimized: doesn't fill unneeded elements except mass and 3x3 inertia.
    void FrVariablesBEMBodyMass::Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
        for (int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
                storage.SetElement(insrow + i, inscol + j, c_a * m_GeneralizedMass(i, j));
            }
        }
    }

}
