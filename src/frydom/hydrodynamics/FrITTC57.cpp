//
// Created by frongere on 13/06/17.
//

#include <cmath>

#include "chrono/physics/ChBody.h"  // TODO : a retirer

#include "FrITTC57.h"
//#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/environment/FrEnvironmentInc.h"
//#include "frydom/environment/current/FrCurrent.h"
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




    FrITTC57_::FrITTC57_(double characteristicLength, double hullFormFactor, double hullWetSurface,
                         double hullFrontalArea) :
                         FrForce_(),
                         m_Lpp(characteristicLength),
                         m_k(hullFormFactor),
                         m_S(hullWetSurface),
                         m_Ax(hullWetSurface) {}

    void FrITTC57_::Update(double time) {

        // TODO : simplifier la procedure pour avoir les vitesses relatives fonction du courant ou du vent. Repercuter
        // aussi sur FrLinearDamping avant d'aller plus loin !!

        // On va ecrire une methode de FrBody_ permettant d'avoir cela dans l'eau et l'air et qui appelle la methode qui
        // va bien depuis current et wind


        // Getting the relative velocity with respect to water along body X axis
        Position cogWorldPos = m_body->GetCOGPositionInWorld(NWU);
        Velocity cogWorldVel = m_body->GetCOGVelocityInWorld(NWU);

        Velocity relVel = m_environment->GetCurrent()->GetAbsRelativeVelocity(cogWorldPos, cogWorldVel, NWU);

        m_body->ProjectVectorInBody<Velocity>(relVel, NWU);
        double  ux = relVel.GetVx();

        // Computing Reynolds number
        double Re = GetSystem()->GetEnvironment()->GetReynoldsNumberInWater(m_Lpp, ux);

        // Computing ITTC57 flat plate friction coefficient
        auto CF = 0.075 / pow( log10(Re)-2., 2. );

        // Residual friction
        auto CR = 0.12 * CF; // TODO: changer !! juste pour avoir qqch (mettre en attribut)

        // Total coefficient
        auto Ct = CF + CR;

        // Resistance along the body X Axis
        double Rt = - 0.5 * m_environment->GetWaterDensity() * m_S * (1+m_k) * Ct * ux * std::abs(ux);

        SetForceInBody(Force(Rt, 0., 0.), NWU);

    }

    void FrITTC57_::Initialize() {
        m_environment = GetSystem()->GetEnvironment(); // To reduce the number of indirections during update
    }

    void FrITTC57_::StepFinalize() {

    }

}  // end namespace frydom