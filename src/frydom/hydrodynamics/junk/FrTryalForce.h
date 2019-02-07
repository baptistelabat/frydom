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


#ifndef FRYDOM_FRTRYALFORCE_H
#define FRYDOM_FRTRYALFORCE_H

#include <iostream>
#include <fstream>

#include "frydom/core/force/FrForce.h"

namespace frydom {

    /**
     * \class FrTryalForce
     * \brief Class not used.
     */
    class FrTryalForce : public FrForce {

      private:
        bool done;
        std::ofstream myfile;

      public:

        FrTryalForce();
        ~FrTryalForce() {
            myfile.close();
            std::cout << "fichier ferme" << std::endl;
        }

        void UpdateState() override;

    };

}  //end namespace frydom

#endif //FRYDOM_FRTRYALFORCE_H
