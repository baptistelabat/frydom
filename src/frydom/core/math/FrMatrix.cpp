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


#include "FrMatrix.h"


namespace frydom {


    void
    SplitMatrix33IntoCoeffs(const Matrix33 &matrix, double &m00, double &m01, double &m02, double &m10, double &m11,
                            double &m12, double &m20, double &m21, double &m22) {
        m00 = matrix(0, 0);
        m01 = matrix(0, 1);
        m02 = matrix(0, 2);
        m10 = matrix(1, 0);
        m11 = matrix(1, 1);
        m12 = matrix(1, 2);
        m20 = matrix(2, 0);
        m21 = matrix(2, 1);
        m22 = matrix(2, 2);
    }

} // end namespace frydom