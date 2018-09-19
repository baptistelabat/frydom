//
// Created by frongere on 03/07/17.
//

#include "FrWindForce.h"

#include "frydom/core/FrConstants.h"
#include "frydom/core/FrHydroBody.h"
#include "frydom/IO/FrLoader.h"


#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/wind/FrWind.h"

namespace frydom {

    FrWindForce::FrWindForce(const std::string& yaml_file) {
        this->ReadTable(yaml_file);
    }


    void FrWindForce::ReadTable(const std::string& yaml_file) {

        std::vector<double> angle, Cx, Cy, Cm;
        ANGLE_UNIT unit;

        LoadWindTableFromYaml(yaml_file, angle, Cx, Cy, Cm, unit);

        auto n = angle.size();

        assert(Cx.size() == n);
        assert(Cy.size() == n);
        assert(Cm.size() == n);

        if (unit == DEG) {
            deg2rad(angle);
        }

        m_table.SetX(angle);
        m_table.AddY("Cx", Cx);
        m_table.AddY("Cy", Cy);
        m_table.AddY("Cm", Cm);
    }


    void FrWindForce::UpdateState() {

        auto mybody = dynamic_cast<FrHydroBody*>(GetBody());

        // Relative wind velocity in the body reference frame
        auto body_velocity = mybody->GetVelocity();
        auto wind_velocity = mybody->GetSystem()->GetEnvironment()->GetWind()->GetFluxVector(NWU);

        auto relative_velocity = body_velocity - wind_velocity;         // In world frame
        relative_velocity = mybody->Dir_World2Body(relative_velocity);  // In body frame

        auto vx = relative_velocity.x();
        auto vy = relative_velocity.y();
        auto vel2 = vx*vx + vy*vy;

        auto relative_velocity_angle = atan2(vy, vx);
        relative_velocity_angle = Normalize_0_2PI(relative_velocity_angle);

        // Extract wind drag coefficients
        auto Cx = m_table.Eval("Cx", relative_velocity_angle);
        auto Cy = m_table.Eval("Cy", relative_velocity_angle);
        auto Cm = m_table.Eval("Cm", relative_velocity_angle);

        // Update Force
        force.x() = Cx * vel2;
        force.y() = Cy * vel2;
        force.z() = 0.;
        force = mybody->Dir_Body2World(force);

        moment.x() = 0.;
        moment.y() = 0.;
        moment.z() = Cm * vel2;

    }




}  // end namespace frydom