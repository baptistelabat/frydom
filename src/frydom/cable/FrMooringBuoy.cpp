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

#include "FrMooringBuoy.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/hydrodynamics/damping/FrLinearDamping.h"


namespace frydom {


    void FrMooringBuoy::FrSphereNonLinearHydrostaticForce::Compute(double time) {
      auto m_buoy = dynamic_cast<FrMooringBuoy *>(GetBody());
      Force Gvector(0., 0., -m_buoy->GetSystem()->GetGravityAcceleration());
      auto rho_water = GetBody()->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
      GetParent();
      // FIXME : appliquer la force au centre de poussée et non au centre de gravité : théoriquement aucun effet, mais plus propre.
      SetForceInWorldAtCOG(-m_buoy->GetVolume() * rho_water * Gvector, NWU);
    }

    FrMooringBuoy::FrMooringBuoy(const std::string& name, double radius, double mass, bool visual_asset,
                                 double damping) : FrBody(name) {
      m_radius = radius;
      m_hydrostaticForce = std::make_shared<FrSphereNonLinearHydrostaticForce>("NonlinearHydrostaticsForceOn_" + name);
      AddExternalForce(m_hydrostaticForce);

      m_dampingForce = std::make_shared<FrLinearDamping>("LinearDampingForceOn_" + name, WATER, false);
      m_dampingForce->SetDiagonalDamping(damping, damping, damping, damping, damping, damping);
      AddExternalForce(m_dampingForce);

      // Properties of the sphere
      double inertia = (2.0 / 5.0) * mass * radius * radius;

      //FIXME: passer par makeItSphere pour la suite, si possible...
//            makeItSphere(this,radius,mass);

      // Building the Chrono body
      SetInertiaTensor(FrInertiaTensor(mass, inertia, inertia, inertia, 0., 0., 0., Position(), NWU));

      // Collision
      auto collisionModel = m_chronoBody->GetCollisionModel();
      collisionModel->ClearModel();
      collisionModel->AddSphere(radius,
                                chrono::ChVector<double>());  // TODO: permettre de specifier les coords relatives dans le modele !!
      collisionModel->BuildModel();
      AllowCollision(true);  // A retirer ?
      SetSmoothContact();  // Smooth contact by default

      // Asset
      AddSphereShape(radius);
    }

    void FrMooringBuoy::Update() {
      computeVolume();
    }

    double FrMooringBuoy::computeDraft() {

      auto eta = GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetPosition(GetPosition(NWU), NWU);
      auto zh = GetPosition(NWU).GetZ() - eta;

      if (zh < -m_radius) { return m_radius; }
      if (zh > m_radius) { return -m_radius; }
      else { return -zh; }

    }


    std::shared_ptr<FrMooringBuoy>
    make_mooring_buoy(const std::string& name, FrOffshoreSystem *system, double radius, double mass, bool visual_asset,
                      double damping) {
      auto buoy = std::make_shared<FrMooringBuoy>(name, radius, mass, visual_asset, damping);
      system->Add(buoy);
      buoy->SetColor(DarkRed);
      return buoy;
    }


}  // end namespace frydom
