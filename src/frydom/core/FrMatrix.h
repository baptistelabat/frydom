//
// Created by frongere on 06/11/18.
//

#ifndef FRYDOM_FRMATRIX_H
#define FRYDOM_FRMATRIX_H

#include "chrono/core/ChMatrix33.h"

#include "MathUtils/Matrix33.h"

namespace frydom {

    using Matrix33 = mathutils::Matrix33<double>;


    void SplitMatrix33IntoCoeffs(const Matrix33& matrix,
                                 double& m00, double& m01, double& m02,
                                 double& m10, double& m11, double& m12,
                                 double& m20, double& m21, double& m22);


    namespace internal {

        /// Converts a chrono::ChMatrix33 into a frydom::Matrix33
        inline Matrix33 ChMatrix33ToMatrix33(const chrono::ChMatrix33<double>& chronoMatrix33) {
            Matrix33 matrix;
            matrix << chronoMatrix33.Get33Element(0, 0),
                      chronoMatrix33.Get33Element(0, 1),
                      chronoMatrix33.Get33Element(0, 2),
                      chronoMatrix33.Get33Element(1, 0),
                      chronoMatrix33.Get33Element(1, 1),
                      chronoMatrix33.Get33Element(1, 2),
                      chronoMatrix33.Get33Element(2, 0),
                      chronoMatrix33.Get33Element(2, 1),
                      chronoMatrix33.Get33Element(2, 2);
            return matrix;
        }

    } // end namespace internal

}  // end namespace frydom

#endif //FRYDOM_FRMATRIX_H
