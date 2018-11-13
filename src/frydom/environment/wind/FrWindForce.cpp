//
// Created by frongere on 03/07/17.
//

#include "FrWindForce.h"

#include "frydom/core/FrGeographic.h"
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

    void FrWindForce::SetLogPrefix(std::string prefix_name) {
       if (prefix_name == "" ) {
           m_logPrefix = "Fwind_" + FrForce::m_logPrefix;
       } else {
           m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
       }

    }






    /// >>>>>>>>>>>>>>>>>>>>> REFACTORING

    FrWindForce_::FrWindForce_(std::string yamlFile) {
        this->ReadTable(yamlFile);
    }

    void FrWindForce_::ReadTable(std::string yamlFile) {

        std::vector<double> angle, Cx, Cy, Cm;
        ANGLE_UNIT unit;

        LoadWindTableFromYaml(yamlFile, angle, Cx, Cy, Cm, unit);

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

    void FrWindForce_::Update(double time) {

        FrFrame_ cogFrame = m_body->GetFrameAtCOG(NWU);
        Velocity cogWorldVel = m_body->GetCOGVelocityInWorld(NWU);

        Velocity cogRelVel = m_body->GetSystem()->GetEnvironment()->GetWind()->GetRelativeVelocityInFrame(cogFrame, cogWorldVel, NWU);

        double alpha = cogRelVel.GetProjectedAngleAroundZ(RAD)+M_PI;
        alpha = Normalize_0_2PI(alpha);

        auto cx = m_table.Eval("Cx", alpha);
        auto cy = m_table.Eval("Cy", alpha);
        auto cz = m_table.Eval("Cm", alpha);

        double velSquare = cogRelVel.squaredNorm();

        auto fx = cx * velSquare;
        auto fy = cy * velSquare;
        auto mz = cz * velSquare;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }


}  // end namespace frydom
