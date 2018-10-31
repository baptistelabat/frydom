//
// Created by frongere on 27/09/18.
//

#include "FrVector.h"


namespace frydom {

    namespace internal {

        // SYMBOLIC DIRECTIONS EXPRESSED IN THE NWU FRAME
        const Direction NORTH(1, 0, 0);
        const Direction NORTH_EAST(MU_SQRT2_2, -MU_SQRT2_2, 0);
        const Direction EAST(0, -1, 0);
        const Direction SOUTH_EAST(-MU_SQRT2_2, -MU_SQRT2_2, 0);
        const Direction SOUTH(-1, 0, 0);
        const Direction SOUTH_WEST(-MU_SQRT2_2, MU_SQRT2_2, 0);
        const Direction WEST(0, 1, 0);
        const Direction NORTH_WEST(MU_SQRT2_2, MU_SQRT2_2, 0);

    }  // end namespace internal

} // end namespace frydom