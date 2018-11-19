//
// Created by camille on 15/11/18.
//

#include "FrFlowForce.h"

#include "frydom/core/FrBody.h"
#include "frydom/environment/FrFlowBase.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/IO/FrLoader.h"
#include "frydom/core/FrFrame.h"
#include "MathUtils/Vector3d.h"


namespace frydom {

    FrFlowForce::FrFlowForce(const std::string& yamlFile) {
        this->ReadTable(yamlFile);
    }

    void FrFlowForce::ReadTable(const std::string& yamlFile) {

        std::vector<std::pair<double, Vector3d<double>>> polar;
        std::pair<double, Vector3d<double>> new_element;
        ANGLE_UNIT angle_unit;
        FRAME_CONVENTION fc;
        DIRECTION_CONVENTION dc;

        LoadFlowPolarCoeffFromYaml(yamlFile, polar, angle_unit, fc, dc);

        if (angle_unit == DEG) {
            for (auto it=polar.begin(); it != polar.end(); ++it) { it->first *= DEG2RAD; }
        }

        // Complete if symmetry
        auto max_angle = polar.back().first;
        if (std::abs(max_angle - M_PI) < 10e-2) {

            for (unsigned int i=polar.size()-2; i>=1; i--) {
                new_element.first = 2.* M_PI - polar[i].first;
                new_element.second = { polar[i].second[0], -polar[i].second[1], -polar[i].second[2] };
                polar.push_back(new_element);
            }
        }

        // Delete double term
        if ( std::abs(polar.back().first - 2.* M_PI) < 10e-2) {
            polar.pop_back();
        }

        // Conversion to NWU if NED convention is used
        if (fc == NED) {
            for (auto it=polar.begin(); it != polar.end(); ++it) { it->first = -it->first; }
        }

        // Conversion to GOTO if COMEFROM convention is used
        if (dc == COMEFROM) {
            for (auto it=polar.begin(); it!= polar.end(); ++it) { it->first += M_PI; }
        }

        // Normalized angle in [0, 2pi]
        for (auto it=polar.begin(); it != polar.end(); ++it) { it->first = Normalize_0_2PI(it->first); }

        // Sort element according to increasing angles
        std::sort(polar.begin(), polar.end(), [](auto const &a, auto const &b) {
            return a.first < b.first;
        });

        // Adding last term for angle equal to 2pi
        new_element.first = 2* M_PI;
        new_element.second =  polar.begin()->second;
        polar.push_back( new_element );

        // Change tuple into std::vector
        // TODO : essayer d'utiliser directement du VectorN dans les méthodes d'interpolation

        std::vector<double> anglesL;
        std::vector<double> cxL, cyL, cnL;

        for (auto it=polar.begin(); it != polar.end(); ++it) {
            anglesL.push_back(it->first);
            cxL.push_back(it->second[0]);
            cyL.push_back(it->second[1]);
            cnL.push_back(it->second[2]);
        }

        m_table.SetX(anglesL);
        m_table.AddY("cx", cxL);
        m_table.AddY("cy", cyL);
        m_table.AddY("cn", cnL);
    }

    void FrFlowForce::Update(double time) {

        double alpha = m_fluxVelocityInBody.GetProjectedAngleAroundZ(RAD);
        alpha = Normalize_0_2PI(alpha);

        auto cx = m_table.Eval("cx", alpha);
        auto cy = m_table.Eval("cy", alpha);
        auto cn = m_table.Eval("cn", alpha);

        double SquaredVelocity = m_fluxVelocityInBody.squaredNorm();

        auto fx = cx * SquaredVelocity;
        auto fy = cy * SquaredVelocity;
        auto mz = cn * SquaredVelocity;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }


    void FrCurrentForce2_::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        m_fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetCurrent()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        FrFlowForce::Update(time);
    }



    void FrWindForce2_::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        m_fluxVelocityInBody = m_body->GetSystem()->GetEnvironment()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        FrFlowForce::Update(time);

    }

} // end of namespace frydom