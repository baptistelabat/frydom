// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FREIGEN_H
#define FRYDOM_FREIGEN_H

#include <iostream>

#include <chrono/core/ChMatrix33.h>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "Eigen/Dense"


// Forward declarations
namespace chrono {
    template <class Real>
    class ChVector;

    template <class Real>
    class ChMatrix33;
}

namespace frydom {

    /// chrono::ChVector <--> Eigen::Matrix
    template <class Real=double>
    Eigen::Matrix<Real, 3, 1> ChEig(const chrono::ChVector<Real>& vect) {
        Eigen::Matrix<Real, 3, 1> out;
        out << vect.x(), vect.y(), vect.z();
        return out;
    }

    template <class Real=double>
    chrono::ChVector<Real> ChEig(const Eigen::Matrix<Real, 3, 1>& vect) {
        chrono::ChVector<Real> out(vect.x(), vect.y(), vect.z());
        return out;
    }

    /// chrono::ChMatrix33 <--> Eigen::Matrix
    template <class Real=double>
    Eigen::Matrix<Real, 3, 3> ChEig(const chrono::ChMatrix33<Real>& mat) {
        Eigen::Matrix<Real, 3, 3> out;
        out << mat.Get33Element(0, 0), mat.Get33Element(0, 1), mat.Get33Element(0, 2),
               mat.Get33Element(1, 0), mat.Get33Element(1, 1), mat.Get33Element(1, 2),
               mat.Get33Element(2, 0), mat.Get33Element(2, 1), mat.Get33Element(2, 2);
        return out;
    };

    template <class Real=double>
    chrono::ChMatrix33<Real> ChEig(const Eigen::Matrix<Real, 3, 3>& mat) {
        chrono::ChMatrix33<Real> out;
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        out.Set33Element(0, 0, mat(0, 0));
        return out;
    }

    /// chrono::ChMatrixNM <--> Eigen::Matrix
    template <class Real=double, int nbrows, int nbcols>
    Eigen::Matrix<Real, nbrows, nbcols> ChEig(const chrono::ChMatrixNM<Real, nbrows, nbcols>& mat) {
        Eigen::Matrix<Real, nbrows, nbcols> out;
        for (int j=0; j<nbcols; ++j) {
            for (int i=0; i<nbrows; ++i) {
                out(i, j) = mat.Element(i, j);
            }
        }
        return out;
    };

    template <class Real=double, int nbrows, int nbcols>
    chrono::ChMatrixNM<Real, nbrows, nbcols> ChEig(const Eigen::Matrix<Real, nbrows, nbcols>& mat) {
        chrono::ChMatrixNM<Real, nbrows, nbcols> out;
        for (int j=0; j<nbcols; ++j) {
            for (int i=0; i<nbrows; ++i) {
                out.SetElement(i, j, mat(i, j));
            }
        }
        return out;
    };

    /// chrono::ChMatrixDynamic <--> Eigen::Matrix
    template<class Real=double>
    Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> ChEig(const chrono::ChMatrixDynamic<Real>& mat) {

        int nbrows = mat.GetRows();
        int nbcols = mat.GetColumns();
        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> out(nbrows, nbcols);

        for (int j=0; j<nbcols; ++j) {
            for (int i=0; i<nbrows; ++i) {
                out(i, j) = mat.Element(i, j);
            }
        }
        return out;
    };

    template <class Real=double>
    chrono::ChMatrixDynamic<Real> ChEig(const Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic>& mat) {
        int nbrows = int(mat.rows());
        int nbcols = int(mat.cols());

        chrono::ChMatrixDynamic<Real> out(nbrows, nbcols);

        for (int j=0; j<nbcols; ++j) {
            for (int i=0; i<nbrows; ++i) {
                out.SetElement(i, j, mat(i, j));
            }
        }
        return out;
    };


    // TODO: faire le vecteur dynamique et le ChVector2d...



    // =================================================================================================================
    // Linear Algebra helpers
    // =================================================================================================================

    template <class Scalar>
    Scalar norm_inf(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& mat) {
        return mat.cwiseAbs().rowwise().sum().maxCoeff();
    }

    template <class Scalar=double>
    bool is_positiveSemidefinite(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& mat,
                                 const Scalar& epsilon=1e-7) {

        return (mat.rows() == mat.cols()) && ((mat.eigenvalues().real().array() > -epsilon).all());
    }

    // -----------------------------------------------------------------------------------------------------------------

    template <class Scalar=double>
    bool is_positiveSemidefinite(const chrono::ChMatrixNM<Scalar>& mat) {
        return is_positiveSemidefinite(ChEig(mat));
    }

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * \class QR_decomposition
     * \brief Class to perform a QR decomposition.
     */
    template <class Scalar>
    class QR_decomposition {

    private:
        Eigen::HouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> QR;

    public:
        explicit QR_decomposition(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
            assert(A.rows() >= A.cols());
            QR = A.householderQr();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetQ() const {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q;
            Q = QR.householderQ();
            auto thinQ = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(QR.rows(), QR.cols());
            return Q * thinQ;
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetR() const {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R;
            R = QR.matrixQR().template triangularView<Eigen::Upper>();
            return R.block(0, 0, QR.cols(), QR.cols());
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetA() const {
            return GetQ() * GetR();
        };

    };

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * \class LU_decomposition
     * \brief Class to perform a LU decomposition.
     */
    template <class Scalar>
    class LU_decomposition {

    private:
        Eigen::PartialPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> LU;
    public:
        explicit LU_decomposition(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
            assert(A.rows() == A.cols());
            LU = A.partialPivLu();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetL() const {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> L;
            L = LU.matrixLU().template triangularView<Eigen::StrictlyLower>();
            return L + Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(LU.rows(), LU.cols());
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetU() const {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> U;
            U = LU.matrixLU().template triangularView<Eigen::Upper>();
            auto p = LU.permutationP();
            return U;
        }

        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> GetPermutationP() const {
            return LU.permutationP();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetA() const {
            return LU.permutationP().inverse() * GetL() * GetU();
        }
    };

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * \class Cholesky_decomposition
     * \brief Class to perform a Cholesky decomposition.
     */
    template <class Scalar>
    class Cholesky_decomposition {

    private:
        Eigen::LLT<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> chol;

    public:
        explicit Cholesky_decomposition(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
            
            if (!is_positiveSemidefinite(A)) {
                throw std::runtime_error("In Cholesky decomposition, matrix must be positive demi-definite");
            }

            chol = A.llt();

//            if (chol.info() == Eigen::NumericalIssue) {
//                throw std::runtime_error("Possibly non semi-positive definite matrix for Cholesky decomposition !");
//            }
        }

        /// Get the L-matrix of the Cholesky decomposition.
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetL() const {
            return chol.matrixL();
        }

        /// Get the A-matrix of the Cholesky decomposition.
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetA() const {
            auto L = GetL();
            return L * L.transpose();
        }
    };

    // -----------------------------------------------------------------------------------------------------------------

    /**
     * \class SVD_decomposition
     * \brief Class to perform a SVD decomposition.
     */
    template <class Scalar>
    class SVD_decomposition {

    private:
        Eigen::JacobiSVD<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> SVD;

    public:
        explicit SVD_decomposition(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
            SVD = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> GetSingularValues() const {
            return SVD.singularValues();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetU() const {
            return SVD.matrixU();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetV() const {
            return SVD.matrixV();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetA() const {
            return GetU() * SVD.singularValues().asDiagonal() * GetV().adjoint();
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GetPinv(const Scalar tol=1e-6) const {
            auto sing_val_inv = GetSingularValues();

            for (long i=0; i < sing_val_inv.rows(); ++i) {
                if (sing_val_inv(i) > tol) {
                    sing_val_inv(i) = 1. / sing_val_inv(i);
                } else {
                    sing_val_inv(i) = 0.;
                }
            }
            return GetV() * sing_val_inv.asDiagonal() * GetU().adjoint();
        }
    };

    // -----------------------------------------------------------------------------------------------------------------

    /// Moore-Penrose pseudo-inverse
    template <class Scalar=double>
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
        pinv(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& mat, const Scalar tol=1e-6) {
        
        return SVD_decomposition<Scalar>(mat).GetPinv(tol);

    }


}  // end namespace frydom

#endif //FRYDOM_FREIGEN_H
