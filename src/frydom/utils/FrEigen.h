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


#ifndef FRYDOM_FREIGEN_H
#define FRYDOM_FREIGEN_H


//#include <chrono/core/ChMatrix33.h>
//#include "chrono/core/ChVector.h"
//#include "chrono/core/ChMatrix.h"
//#include "chrono/core/ChMatrixDynamic.h"
//
//#include "MathUtils/MathUtils.h"


namespace frydom {

    namespace internal {
        /// chrono::ChVector <--> Eigen::Matrix
        template<class Real=double>
        Eigen::Matrix<Real, 3, 1> ChEig(const chrono::ChVector<Real> &vect) {
            Eigen::Matrix<Real, 3, 1> out;
            out << vect.x(), vect.y(), vect.z();
            return out;
        }

        template<class Real=double>
        chrono::ChVector<Real> ChEig(const Eigen::Matrix<Real, 3, 1> &vect) {
            chrono::ChVector<Real> out(vect.x(), vect.y(), vect.z());
            return out;
        }

        /// chrono::ChMatrix33 <--> Eigen::Matrix
        template<class Real=double>
        Eigen::Matrix<Real, 3, 3> ChEig(const chrono::ChMatrix33<Real> &mat) {
            Eigen::Matrix<Real, 3, 3> out;
            out << mat.Get33Element(0, 0), mat.Get33Element(0, 1), mat.Get33Element(0, 2),
                    mat.Get33Element(1, 0), mat.Get33Element(1, 1), mat.Get33Element(1, 2),
                    mat.Get33Element(2, 0), mat.Get33Element(2, 1), mat.Get33Element(2, 2);
            return out;
        };

        template<class Real=double>
        chrono::ChMatrix33<Real> ChEig(const Eigen::Matrix<Real, 3, 3> &mat) {
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
        template<class Real=double, int nbrows, int nbcols>
        Eigen::Matrix<Real, nbrows, nbcols> ChEig(const chrono::ChMatrixNM<Real, nbrows, nbcols> &mat) {
            Eigen::Matrix<Real, nbrows, nbcols> out;
            for (int j = 0; j < nbcols; ++j) {
                for (int i = 0; i < nbrows; ++i) {
                    out(i, j) = mat.Element(i, j);
                }
            }
            return out;
        };

        template<class Real=double, int nbrows, int nbcols>
        chrono::ChMatrixNM<Real, nbrows, nbcols> ChEig(const Eigen::Matrix<Real, nbrows, nbcols> &mat) {
            chrono::ChMatrixNM<Real, nbrows, nbcols> out;
            for (int j = 0; j < nbcols; ++j) {
                for (int i = 0; i < nbrows; ++i) {
                    out.SetElement(i, j, mat(i, j));
                }
            }
            return out;
        };

        /// chrono::ChMatrixDynamic <--> Eigen::Matrix
        template<class Real=double>
        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> ChEig(const chrono::ChMatrixDynamic<Real> &mat) {

            int nbrows = mat.GetRows();
            int nbcols = mat.GetColumns();
            Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> out(nbrows, nbcols);

            for (int j = 0; j < nbcols; ++j) {
                for (int i = 0; i < nbrows; ++i) {
                    out(i, j) = mat.Element(i, j);
                }
            }
            return out;
        };

        template<class Real=double>
        chrono::ChMatrixDynamic<Real> ChEig(const Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> &mat) {
            int nbrows = int(mat.rows());
            int nbcols = int(mat.cols());

            chrono::ChMatrixDynamic<Real> out(nbrows, nbcols);

            for (int j = 0; j < nbcols; ++j) {
                for (int i = 0; i < nbrows; ++i) {
                    out.SetElement(i, j, mat(i, j));
                }
            }
            return out;
        };

    }  // end namespace frydom::internal

}  // end namespace frydom


#endif //FRYDOM_FREIGEN_H
