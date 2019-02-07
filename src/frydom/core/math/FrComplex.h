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


#ifndef FRYDOM_FRCOMPLEX_H
#define FRYDOM_FRCOMPLEX_H

#include "complex"

namespace frydom {

    using Complex = std::complex<double>;
    #define JJ Complex(0, 1)

}

#endif //FRYDOM_FRCOMPLEX_H
