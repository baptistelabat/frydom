//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWIND_H
#define FRYDOM_FRWIND_H

#include "chrono/core/ChVector.h"

#include "frydom/core/FrObject.h"
#include "frydom/core/FrConstants.h"
#include "frydom/environment/FrConventions.h"

#include "MathUtils.h"


using namespace mathutils;


namespace frydom {

    class FrWind : public FrObject {

    private:
        chrono::ChVector<double> m_windvector = chrono::VNULL;

    public:
        void Update(double time) {
            // TODO
        }

        void Set(const chrono::ChVector<>& unitDirection, double magnitude,
                 FrFrame frame=NED, FrDirectionConvention directionConvention=GOTO, SPEED_UNIT speedUnit=KNOT) {
            auto uDirection = unitDirection;
            uDirection /= unitDirection.Length();

            if (frame == NED) {
                uDirection = NED2NWU(uDirection);
            }

            if (directionConvention == COMEFROM) {
                uDirection = - uDirection;
            }

            auto vel = magnitude;

            if (speedUnit != MS) {
                vel = convert_velocity_unit(vel, speedUnit, MS);
            }

            // Building the current vector
            m_windvector = uDirection * vel;
        }


        virtual void Initialize() override {}

        virtual void StepFinalize() override {}

    };

}  // end namespace frydom
#endif //FRYDOM_FRWIND_H
