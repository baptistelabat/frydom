//
// Created by frongere on 23/06/17.
//

#include <chrono/core/ChVector.h>
#include "FrConstants.h"

#include "MathUtils.h"

namespace frydom {

    // SYMBOLIC DIRECTIONS EXPRESSED IN THE NED FRAME
    const chrono::ChVector<double> NORTH(1, 0, 0);
    const chrono::ChVector<double> NORTH_EAST(MU_SQRT2_2, MU_SQRT2_2, 0);
    const chrono::ChVector<double> EAST(0, 1, 0);
    const chrono::ChVector<double> SOUTH_EAST(-MU_SQRT2_2, MU_SQRT2_2, 0);
    const chrono::ChVector<double> SOUTH(-1, 0, 0);
    const chrono::ChVector<double> SOUTH_WEST(-MU_SQRT2_2, -MU_SQRT2_2, 0);
    const chrono::ChVector<double> WEST(0, -1, 0);
    const chrono::ChVector<double> NORTH_WEST(MU_SQRT2_2, -MU_SQRT2_2, 0);

}  // end namespace frydom