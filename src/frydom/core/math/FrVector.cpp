//
// Created by frongere on 27/09/18.
//

#include "FrVector.h"


namespace frydom {

    const mathutils::Vector3d<double> NORTH(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(1, 0, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> NORTH_EAST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(MU_SQRT2_2, -MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> EAST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(0, -1, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> SOUTH_EAST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(-MU_SQRT2_2, -MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> SOUTH(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(-1, 0, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> SOUTH_WEST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(-MU_SQRT2_2, MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> WEST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(0, 1, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

    const mathutils::Vector3d<double> NORTH_WEST(FRAME_CONVENTION fc) {
        auto vect = mathutils::Vector3d<double>(MU_SQRT2_2, MU_SQRT2_2, 0);
        if (IsNED(fc)) { internal::SwapFrameConvention(vect);}
        return vect;
    }

} // end namespace frydom