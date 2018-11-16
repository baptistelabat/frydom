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

        std::vector<double> angles, cx, cy, cn;
        ANGLE_UNIT angle_unit;
        FRAME_CONVENTION fc;
        DIRECTION_CONVENTION dc;

        LoadFlowPolarCoeffFromYaml(yamlFile, angles, cx, cy, cn, angle_unit, fc, dc);
        //LoadFlowPolarCoeffFromYaml(yamlFile, polar, angle_unit, fc, dc);

        auto n = angles.size();
        assert(cx.size() == n);
        assert(cy.size() == n);
        assert(cn.size() == n);

        if (angle_unit == DEG) { deg2rad(angles); }

        // Convert double into tuple
        // TODO : utiliser directement un tuple en sortie de Load coefficient
        std::pair<double, Vector3d<double>> new_element;
        for (int i=0; i<angles.size(); i++) {
            //new_element.first = angles[i];
            //new_element.second = Vector3d<double>(cx[i], cy[i], cn[i]);
            //polar.push_back( new_element ); // TODO : a passer en argument du loader
            polar.push_back( std::pair<double, Vector3d<double>>(angles[i], Vector3d<double>(cx[i], cy[i], cn[i])));
        }

        // Complete if symmetry
        // TODO : essayer de ne plus avoir recours à cx, cy cn mais seulement à polar
        auto max_angle = polar.end()->first;
        if (std::abs(max_angle - M_PI) < 10e-2) {

            std::pair<double, Vector3d<double>> new_element;

            for (unsigned int i=polar.size()-1; i>=1; i--) {
                new_element.first = 2.* M_PI - angles[i];
                new_element.second = Vector3d<double>(cx[i], -cy[i], -cn[i]);
                polar.push_back(new_element);
            }
        }

        // Delete double term
        if ( (*angles.end() - 2.* M_PI) < 10e-2) {
            polar.pop_back();
        }

        // Conversion to NWU if NED convention is used
        if (fc == NED) {
            for (auto& element: polar) { element.first = -element.first; }
        }

        // Conversion to GOTO if COMEFROM convention is used
        if (dc == COMEFROM) {
            for (auto& element: polar) { element.first = element.first + M_PI; }
        }

        // Normalized angle in [0, 2pi]
        for (auto& element: polar) { Normalize_0_2PI(element.first); }

        // Sort element according to increasing angles
        std::sort(polar.begin(), polar.end(), [](auto const &a, auto const &b) {
            return a.first < b.first;
        });

        // Adding last term for angle equal to 2pi
        new_element.first = 2* M_PI;
        new_element.second =  Vector3d<double>(cx[0], cy[0], cn[0]);
        polar.push_back( new_element );

        // Change tuple into std::vector
        // TODO : essayer d'utiliser directement du VectorN dans les méthodes d'interpolation

        std::vector<double> anglesL;
        std::vector<double> cxL, cyL, cnL;

        for (auto element: polar) {
            anglesL.push_back(std::get<0>(element));
            cxL.push_back(element.second[0]);
            cyL.push_back(element.second[1]);
            cnL.push_back(element.second[2]);
        }

        m_table.SetX(anglesL);
        m_table.AddY("cx", cxL);
        m_table.AddY("cz", cyL);
        m_table.AddY("cn", cnL);
    }

    void FrFlowForce::Update(double time) {

        FrFrame_ FrameAtCOG = m_body->GetFrameAtCOG(NWU);
        Velocity VelocityInWorldAtCOG =  m_body->GetCOGVelocityInWorld(NWU);

        Velocity FluxVelocityInFrame = m_body->GetSystem()->GetEnvironment()->GetWind()
                ->GetRelativeVelocityInFrame(FrameAtCOG, VelocityInWorldAtCOG, NWU);

        double alpha = FluxVelocityInFrame.GetProjectedAngleAroundZ(RAD)+M_PI;
        alpha = Normalize_0_2PI(alpha);

        auto cx = m_table.Eval("cx", alpha);
        auto cy = m_table.Eval("cy", alpha);
        auto cn = m_table.Eval("cn", alpha);

        double SquaredVelocity = FluxVelocityInFrame.squaredNorm();

        auto fx = cx * SquaredVelocity;
        auto fy = cy * SquaredVelocity;
        auto mz = cn * SquaredVelocity;

        SetForceTorqueInBodyAtCOG(Force(fx, fy, 0.), Torque(0., 0., mz), NWU);

    }

} // end of namespace frydom