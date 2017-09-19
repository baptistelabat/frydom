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

//    template <class T<class Real=double, int nbrows, int nbcols>>
//    class QR_decomp {
//
//    private:
//        Eigen::HouseholderQR<T> QR;
//        bool thin_decomposition = true;
//
//    public:
//        QR_decomp(const T& A) {
//
//            QR = Eigen::HouseholderQR<T>(A);
//
//        }
//
//    };


//    template <typename MatrixType>
//    class QR_decomp {
//
//    private:
//        Eigen::HouseholderQR<MatrixType> QR;
//
//    public:
////        typedef _MatrixType MatrixType;
//        template<typename InputType>
//        explicit QR_decomp(const MatrixType& matrix) {
//            std::cout << matrix;
//        }
//
//    };



    template <class Real=double, int nbrows, int nbcols>
    void QR_decomposition(const Eigen::Matrix<Real, nbrows, nbcols>& A,
                          Eigen::Matrix<Real, nbrows, nbcols>& Q,
                          Eigen::Matrix<Real, nbcols, nbcols>& R) {

        auto thinQ = Eigen::Matrix<Real, nbrows, nbcols>(Eigen::Matrix<Real, nbrows, nbcols>::Identity(nbrows, nbcols));

        Eigen::HouseholderQR<Eigen::Matrix<Real, nbrows, nbcols>> QR(A);

        Q = QR.householderQ() * thinQ;

//        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> R_tmp = QR.matrixQR();

//        Eigen::Matrix<Real> essai;
//
//        essai = R_tmp.triangularView<Eigen::Upper>();



        std::cout << "coucou";
//        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> tv;
//        tv = R_tmp.triangularView<Eigen::Upper>();
//        R = tv.block(0, 0, nbcols, nbcols);

    }




    /// Print functions for chrono
//    template <class Real=double>
//    void print(chrono::ChVector<Real>& vect) {
//        std::cout << "\n" << vect.x() << "\n" << vect.y() << "\n" << vect.z() << std::endl;
//    }
//
//    template <class Real=double>
//    void print(chrono::ChMatrix<Real>& mat) {
//        std::cout << "\n";
//        for (int i=0; i<mat.GetRows(); ++i) {
//            for (int j=0; j<mat.GetColumns(); j++) {
//                std::cout << mat.GetElement(i, j) << "\t";
//            }
//            std::cout << std::endl;
//        }
//        std::cout << std::endl;
//    }

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
