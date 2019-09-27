// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrITTCResistance.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"


namespace frydom {

    void FrITTCResistance::Compute(double time) {

      auto body = GetBody();
      auto system = body->GetSystem();
      auto ocean = system->GetEnvironment()->GetOcean();

      Velocity cogBodyVel = body->GetCOGVelocityInBody(NWU);
      double ux = cogBodyVel.GetVx();

      // Computing Reynolds number
      double Re = ocean->GetReynoldsNumberInWater(m_Lpp, ux);

      // Computing ITTC57 flat plate friction coefficient
      auto Cf = 0.075 / pow(log10(Re) - 2., 2.);

      // Total coefficient
      auto Ct = (1. + m_k) * Cf + m_cr + m_ca + m_caa + m_capp;

      // Resistance along the body X Axis
      double Rt = -0.5 * ocean->GetDensity() * m_hullWetSurface * Ct * ux * std::abs(ux);

      SetForceInBody(Force(Rt, 0., 0.), NWU);
    }

    void FrITTCResistance::SetRoughnessFromLength(double Lwl, double surfaceRoughness) {
      m_ca = (105. * std::pow(surfaceRoughness / Lwl, 1. / 3.) - 0.64) * 0.001;
    }

    void FrITTCResistance::SetAirResistanceFromArea(double area) {
      m_caa = area / (1000. * m_hullWetSurface);
    }

    void FrITTCResistance::Initialize() {
      FrForce::Initialize();
    }

    FrITTCResistance::FrITTCResistance(const std::string &name,
                                       double Lpp,
                                       double hullWetSurface,
                                       double cr,
                                       double k,
                                       double ca,
                                       double caa,
                                       double capp) :
        FrForce(name),
        m_Lpp(Lpp),
        m_hullWetSurface(hullWetSurface),
        m_k(k),
        m_cr(cr),
        m_ca(ca),
        m_caa(caa),
        m_capp(capp) {}

}  // end namespace frydom
