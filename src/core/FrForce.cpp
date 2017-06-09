//
// Created by frongere on 08/06/17.
//

#include "FrForce.h"

namespace frydom{

    void FrForce::UpdateTime(double mytime) {
        ChTime = mytime;
        std::cout << "coucou" << mytime << std::endl;
    }

}  // end namespace frydom