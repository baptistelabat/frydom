//
// Created by frongere on 21/06/17.
//

#include "FrCurrentForce.h"

#include "chrono/physics/ChBody.h"

#include "frydom/core/FrGeographic.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/IO/FrLoader.h"

#include "frydom/environment/FrEnvironment.h"


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
        // TODO: verifier qu'on a pas un probleme de frame !!!!
        auto fx = -cx * vel2; // FIXME: on doit caler le sige suivant la convention prise dans les coeffs...
        auto fy = -cy * vel2;
        auto mz = -cz * vel2;

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



    /// >>>>>>>>>>>>>>>>>>>>> REFACTORING

    FrCurrentForce_::FrCurrentForce_(std::string yamlFile) {
        m_coeffsTable = std::move(MakeCurrentPolarCoeffTable(yamlFile));
    }

    void FrCurrentForce_::Update(double time) {

        auto cogRelVel = m_body->GetLocalRelVelocityInStreamAtCOG(WATER, NWU);
        auto velSquare = cogRelVel.squaredNorm();
        auto alpha = m_body->GetApparentAngle(WATER, NWU, DEG);

        auto cx = m_coeffsTable.CX(alpha, NWU);
        auto cy = m_coeffsTable.CY(alpha, NWU);
        auto cz = m_coeffsTable.CZ(alpha, NWU);

        auto fx = -cx * velSquare;
        auto fy = -cy * velSquare;
        auto mz = -cz * velSquare;

        SetLocalForceTorqueAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }


}  // end namespace frydom
