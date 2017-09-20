//
// Created by frongere on 18/09/17.
//

#ifndef FRYDOM_FREIGEN_H
#define FRYDOM_FREIGEN_H

#include <iostream>

#include <chrono/core/ChMatrix33.h>
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "Eigen/Dense"


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
    class QR_decomposition {

    private:
        Eigen::HouseholderQR<Eigen::Matrix<Scalar, -1, -1>> QR;

    public:
        explicit QR_decomposition(const Eigen::Matrix<Scalar, -1, -1>& A) {
            assert(A.rows() >= A.cols());
            QR = A.householderQr();
        }

        Eigen::Matrix<Scalar, -1, -1> GetQ() const {
            Eigen::Matrix<Scalar, -1, -1> Q;
            Q = QR.householderQ();
            auto thinQ = Eigen::Matrix<Scalar, -1, -1>::Identity(QR.rows(), QR.cols());
            return Q * thinQ;
        }

        Eigen::Matrix<Scalar, -1, -1> GetR() const {
            Eigen::Matrix<Scalar, -1, -1> R;
            R = QR.matrixQR().template triangularView<Eigen::Upper>();
            return R.block(0, 0, QR.cols(), QR.cols());
        }

        Eigen::Matrix<Scalar, -1, -1> GetA() const {
            return GetQ() * GetR();
        };

    };

    template <class Scalar>
    class LU_decomposition {

    private:
        Eigen::PartialPivLU<Eigen::Matrix<Scalar, -1, -1>> LU;
    public:
        explicit LU_decomposition(const Eigen::Matrix<Scalar, -1, -1>& A) {
            assert(A.rows() == A.cols());
            LU = A.partialPivLu();
        }

        Eigen::Matrix<Scalar, -1, -1> GetL() const {
            Eigen::Matrix<Scalar, -1, -1> L;
            L = LU.matrixLU().template triangularView<Eigen::StrictlyLower>();
            return L + Eigen::Matrix<Scalar, -1, -1>::Identity(LU.rows(), LU.cols());
        }

        Eigen::Matrix<Scalar, -1, -1> GetU() const {
            Eigen::Matrix<Scalar, -1, -1> U;
            U = LU.matrixLU().template triangularView<Eigen::Upper>();
            auto p = LU.permutationP();
            return U;
        }

        Eigen::PermutationMatrix<-1, -1, int> GetPermutationP() const {
            return LU.permutationP();
        }

        Eigen::Matrix<Scalar, -1, -1> GetA() const {
            return LU.permutationP().inverse() * GetL() * GetU();
        }
    };

    template <class Scalar>
    class Cholesky_decomposition {

    private:
        Eigen::LLT<Eigen::Matrix<Scalar, -1, -1>> chol;

    public:
        explicit Cholesky_decomposition(const Eigen::Matrix<Scalar, -1, -1>& A) {
            // FIXME: check also if the matrix is symmetric positive-definite !!
            assert(A.rows() == A.cols());
            chol = A.llt();
            if (chol.info() == Eigen::NumericalIssue) {
                throw std::runtime_error("Possibly non semi-positive definite matrix for Cholesky decomposition !");
            }
        }

        Eigen::Matrix<Scalar, -1, -1> GetL() const {
            return chol.matrixL();
        }

        Eigen::Matrix<Scalar, -1, -1> GetA() const {
            auto L = GetL();
            return L * L.transpose();
        }
    };

    template <class Scalar>
    class SVD_decomposition {

    private:
        Eigen::JacobiSVD<Eigen::Matrix<Scalar, -1, -1>> SVD;

    public:
        explicit SVD_decomposition(const Eigen::Matrix<Scalar, -1, -1>& A) {
//            assert(A.rows() == A.cols());
            SVD = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        }

        Eigen::Matrix<Scalar, -1, 1> GetSingularValues() const {
            return SVD.singularValues();
        }

        Eigen::Matrix<Scalar, -1, -1> GetU() const {
            return SVD.matrixU();
        }

        Eigen::Matrix<Scalar, -1, -1> GetV() const {
            return SVD.matrixV();
        }

        Eigen::Matrix<Scalar, -1, -1> GetA() const {
            Eigen::DiagonalMatrix<Scalar, -1, -1> S(SVD.singularValues());
            return GetU() * S * GetV().adjoint();
        }
    };

    /// Moore-Penrose pseudo-inverse
    template <class Scalar=double>
    Eigen::Matrix<Scalar, -1, -1> pinv(const Eigen::Matrix<double, -1, -1>& mat, const Scalar tol=1e-6) {

        auto svd = SVD_decomposition<Scalar>(mat);

        auto sing_val_inv = svd.GetSingularValues();

        for (long i=0; i < sing_val_inv.rows(); ++i) {
            if (sing_val_inv(i) > tol) {
                sing_val_inv(i) = 1. / sing_val_inv(i);
            } else {
                sing_val_inv(i) = 0.;
            }
        }

        Eigen::Matrix<Scalar, -1, -1> pinv_mat = (svd.GetV() * sing_val_inv.asDiagonal() * svd.GetU().transpose());
        return pinv_mat;
    }


    /// Print functions for chrono
    // TODO: placer ces fonctions dans un utilitaire autre que FrEigen !!
    // TODO : utiliser sprintf...
    template <class Real=double>
    std::ostream& operator<<(std::ostream& os, const chrono::ChMatrix<Real>& mat) {
        os << std::endl;
        for (int i=0; i<mat.GetRows(); ++i) {
            for (int j=0; j<mat.GetColumns(); j++) {
                os << mat.GetElement(i, j) << "\t";
            }
            os << std::endl;
        }
        os << std::endl;
        return os;
    }

    template <class Real=double>
    std::ostream& operator<<(std::ostream& os, const chrono::ChVector<Real>& vect) {
        os << std::endl << vect.x() << std::endl << vect.y() << std::endl << vect.z() << std::endl << std::endl;
        return os;
    }


}  // end namespace frydom

#endif //FRYDOM_FREIGEN_H
