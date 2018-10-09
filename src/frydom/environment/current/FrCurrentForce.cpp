//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBody.h"

#include "frydom/core/FrConstants.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/IO/FrLoader.h"


namespace frydom {

    FrCurrentForce::FrCurrentForce(std::string yaml_file) {
        coeffs_table = std::move(MakeCurrentPolarCoeffTable(yaml_file));
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

        // Getting current force coefficients
        auto cx = coeffs_table.CX(alpha, NWU);
        auto cy = coeffs_table.CY(alpha, NWU);
        auto cz = coeffs_table.CZ(alpha, NWU);

        // Compute force and moment
        auto fx = cx * vel2;
        auto fy = cy * vel2;
        auto mz = cz * vel2;

        force.x() = fx;
        force.y() = fy;
        force.z() = 0.;
        force = mybody->Dir_Body2World(force);

        moment.x() = 0.;
        moment.y() = 0.;
        moment.z() = mz;
        moment = mybody->TransformDirectionParentToLocal(moment);

    }

    void FrCurrentForce::SetLogPrefix(std::string prefix_name) {
        if (prefix_name == "") {
            m_logPrefix = "Fcurrent_" + FrForce::m_logPrefix;
        } else {
            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
        }
    }

}  // end namespace frydom
