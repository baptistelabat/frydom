//
// Created by frongere on 23/06/17.
//

#include <chrono/core/ChVector.h>
#include "FrGeographic.h"

#include "MathUtils/MathUtils.h"

namespace frydom {

    // SYMBOLIC DIRECTIONS EXPRESSED IN THE NED FRAME
    const Direction NORTH(1, 0, 0);
    const Direction NORTH_EAST(MU_SQRT2_2, MU_SQRT2_2, 0);
    const Direction EAST(0, 1, 0);
    const Direction SOUTH_EAST(-MU_SQRT2_2, MU_SQRT2_2, 0);
    const Direction SOUTH(-1, 0, 0);
    const Direction SOUTH_WEST(-MU_SQRT2_2, -MU_SQRT2_2, 0);
    const Direction WEST(0, -1, 0);
    const Direction NORTH_WEST(MU_SQRT2_2, -MU_SQRT2_2, 0);

}  // end namespace frydom