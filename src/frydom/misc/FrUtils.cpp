//
// Created by frongere on 26/07/17.
//

#include "FrUtils.h"

namespace frydom {


    void printvect(const chrono::ChVector<double> &vect) {
        std::cout << vect.x() << "\t" << vect.y() << "\t" << vect.z();
    }

} // end namespace frydom