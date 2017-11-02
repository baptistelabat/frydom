//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBody.h"

#include <frydom/core/FrConstants.h>
#include <frydom/core/FrHydroBody.h>
#include "FrCurrentForce.h"

#include "frydom/IO/FrLoader.h"



namespace frydom {

    FrCurrentForce::FrCurrentForce(std::string yaml_file) {
        coeffs_table = std::move(IO::MakeCurrentPolarCoeffTable(yaml_file));
    }


    void FrCurrentForce::UpdateState() {

//        std::cout << "Updating current force" << std::endl;
        auto mybody = dynamic_cast<FrHydroBody*>(GetBody());

        // Getting body's velocity with respect to water (including current effect)
        auto relative_velocity = mybody->GetCurrentRelativeVelocity(NWU);
        auto vx = relative_velocity.x();
        auto vy = relative_velocity.y();
        auto vel2 = vx*vx + vy*vy;

        // Getting the angle between boat axis and velocity vector
        auto alpha = mybody->GetCurrentRelativeAngle(NWU, DEG);

        // Retrieving hull underwater properties
        auto rho_water = mybody->GetSystem()->GetEnvironment()->GetWaterDensity();
        auto transverse_area = mybody->GetTransverseUnderWaterArea();
        auto lateral_area = mybody->GetLateralUnderWaterArea();
        auto lpp = mybody->GetLpp();

        // Getting current force coefficients
        auto cx = coeffs_table.CX(alpha, NWU);
        auto cy = coeffs_table.CY(alpha, NWU);
        auto cz = coeffs_table.CZ(alpha, NWU);

        // Computing force components from formula.
        // These expressions concern forces in the body reference frame...
        auto fx = 0.5 * rho_water * cx * transverse_area * vel2;
        auto fy = 0.5 * rho_water * cy * transverse_area * vel2;
        auto mz = 0.5 * rho_water * cz * lateral_area * vel2 * lpp;

        force.x() = fx;
        force.y() = fy;
        force.z() = 0.;
        force = mybody->Dir_Body2World(force);

        moment.x() = 0.;
        moment.y() = 0.;
        moment.z() = mz;
        moment = mybody->TransformDirectionParentToLocal(moment);

    }



}  // end namespace frydom
