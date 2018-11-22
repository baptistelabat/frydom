//
// Created by frongere on 27/09/18.
//

#include "FrVector.h"


namespace frydom {

    const Velocity NORTH(FRAME_CONVENTION fc) {
        auto vect = Velocity(1, 0, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity NORTH_EAST(FRAME_CONVENTION fc) {
        auto vect = Velocity(MU_SQRT2_2, -MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity EAST(FRAME_CONVENTION fc) {
        auto vect = Velocity(0, -1, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity SOUTH_EAST(FRAME_CONVENTION fc) {
        auto vect = Velocity(-MU_SQRT2_2, -MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity SOUTH(FRAME_CONVENTION fc) {
        auto vect = Velocity(-1, 0, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity SOUTH_WEST(FRAME_CONVENTION fc) {
        auto vect = Velocity(-MU_SQRT2_2, MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity WEST(FRAME_CONVENTION fc) {
        auto vect = Velocity(0, 1, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const Velocity NORTH_WEST(FRAME_CONVENTION fc) {
        auto vect = Velocity(MU_SQRT2_2, MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

} // end namespace frydom