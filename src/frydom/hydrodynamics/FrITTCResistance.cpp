//
// Created by frongere on 13/06/17.
//

#include <cmath>

#include "chrono/physics/ChBody.h"  // TODO : a retirer

#include "FrITTCResistance.h"
//#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"
//#include "frydom/environment/current/FrCurrent.h"
#include "frydom/environment/flow/FrFlowBase.h"
#include "frydom/core/FrBody.h"


namespace frydom{

    FrITTC57::FrITTC57()
            : rho(1025.),
              nu(1e-6),
              Lpp(0.),
              k(0.25),
              S(0.),
              Ax(0.) {}

    void FrITTC57::UpdateState() {

        // Getting velocity
        auto abs_vel = Body->GetPos_dt();
        auto rel_vel = Body->TransformDirectionParentToLocal(abs_vel);

        // Getting relative velocity with respect to water
        auto ux = rel_vel.x();

        // Computing Reynolds number
        auto Re = std::abs(ux) * Lpp / nu;

        // Computing ITTC57 flat plate friction coefficient
        auto CF = 0.075 / pow( log10(Re)-2., 2. );

        // Residual friction
        auto CR = 0.12 * CF; // TODO: changer !! juste pour avoir qqch (mettre en attribut)

        // Total coefficient
        auto Ct = CF + CR;

        //
        relforce.x() = -0.5 * rho * S * (1+k) * Ct * ux * std::abs(ux);


        force = Body->TransformDirectionLocalToParent(relforce);

//        std::cout << force.x() << "\t" << force.y() << "\t" << force.z() << std::endl;

        auto force_verif = Body->TransformDirectionParentToLocal(force);

    }

    void FrITTC57::SetWaterDensity(double myrho) { rho = myrho; }

    double FrITTC57::GetWaterDensity() { return rho; }

    void FrITTC57::SetWaterKinematicViscosity(double mynu) { nu = mynu; }

    double FrITTC57::GetWaterKinematicViscosity() { return nu; }

    void FrITTC57::SetCharacteristicLength(double myLpp) { Lpp = myLpp; }

    double FrITTC57::GetCharacteristicLength() { return Lpp; }

    void FrITTC57::SetHullFormFactor(double myk) { k = myk; }

    double FrITTC57::GetHullFormFactor() { return k; }

    void FrITTC57::SetHullWettedSurface(double myS) { S = myS; }

    double FrITTC57::GetHullWettedSurface() { return S; }

    void FrITTC57::SetHullFrontalProjectedArea(double myAx) { Ax = myAx; }

    double FrITTC57::GetHullFrontalProjectedArea() { return Ax; }

    void FrITTC57::SetLogPrefix(std::string prefix_name) {
        if (prefix_name=="") {
            m_logPrefix = "Fittc57_" + FrForce::m_logPrefix;
        } else {
            m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
        }
    }













    //////// REFACTOR --------------->>>>>>>>>>>>>>>>

    void FrITTCResistance_::Update(double time) {

        Velocity cogBodyVel = m_body->GetCOGVelocityInBody(NWU);
        double  ux = cogBodyVel.GetVx();

        // Computing Reynolds number
        double Re = GetSystem()->GetEnvironment()->GetOcean()->GetReynoldsNumberInWater(m_Lpp, ux);

        // Computing ITTC57 flat plate friction coefficient
        auto Cf = 0.075 / pow( log10(Re)-2., 2. );

        // Total coefficient
        auto Ct = (1. + m_k)*Cf + m_cr + m_ca + m_caa + m_capp;

        // Resistance along the body X Axis
        double Rt = - 0.5 * m_environment->GetOcean()->GetDensity() * m_hullWetSurface * Ct * ux * std::abs(ux);

        SetForceInBody(Force(Rt, 0., 0.), NWU);
    }

    void FrITTCResistance_::SetRoughnessFromLength(double Lwl, double surfaceRoughness) {
        m_ca = (105. * std::pow(surfaceRoughness / Lwl, 1./3.) - 0.64) * 0.001;
    }

    void FrITTCResistance_::SetAirResistanceFromArea(double area) {
        m_caa = area / (1000. * m_hullWetSurface);
    }

    void FrITTCResistance_::Initialize() {
        FrForce_::Initialize();
        m_environment = GetSystem()->GetEnvironment(); // To reduce the number of indirections during update
    }

    void FrITTCResistance_::StepFinalize() {

    }

}  // end namespace frydom