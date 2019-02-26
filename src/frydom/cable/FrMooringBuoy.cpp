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
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/hydrodynamics/damping/FrLinearDamping.h"


namespace frydom {


    void FrMooringBuoy::FrSphereNonLinearHydrostaticForce::Update(double time) {
        auto m_buoy = dynamic_cast<FrMooringBuoy*>(m_body);
        Force Gvector(0.,0.,-m_buoy->GetSystem()->GetGravityAcceleration());
        auto rho_water = m_body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
        // FIXME : appliquer la force au centre de poussée et non au centre de gravité : théoriquement aucun effet, mais plus propre.
        SetForceInBody(-m_buoy->GetVolume()*rho_water*Gvector, NWU);
    }

    FrMooringBuoy::FrMooringBuoy(double radius, double mass, bool visual_asset, double damping) {
        m_radius = radius;
        m_hydrostaticForce = std::make_shared<FrSphereNonLinearHydrostaticForce>();
        AddExternalForce(m_hydrostaticForce);

        m_dampingForce = std::make_shared<FrLinearDamping_>(WATER,false);
        m_dampingForce->SetDiagonalDamping(damping,damping,damping,damping,damping,damping);
        AddExternalForce(m_dampingForce);

        // Properties of the sphere
        double inertia = (2.0 / 5.0) * mass * radius * radius;

        //FIXME: passer par makeItSphere pour la suite, si possible...
//            makeItSphere(this,radius,mass);

        // Building the Chrono body
        SetInertiaTensor(FrInertiaTensor_(mass, inertia, inertia, inertia, 0., 0., 0., FrFrame_(), NWU));

        // Collision
        auto collisionModel = m_chronoBody->GetCollisionModel();
        collisionModel->ClearModel();
        collisionModel->AddSphere(radius, chrono::ChVector<double>());  // TODO: permettre de specifier les coords relatives dans le modele !!
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
        if (GetPosition(NWU).GetZ()<-m_radius) { return m_radius;}
        if (GetPosition(NWU).GetZ()>m_radius) {return -m_radius;}
        else {return -GetPosition(NWU).GetZ();}
    }


    std::shared_ptr<FrMooringBuoy>
    make_mooring_buoy(FrOffshoreSystem_* system, double radius, double mass, bool visual_asset, double damping){
        auto buoy = std::make_shared<FrMooringBuoy>(radius, mass, visual_asset, damping);
        system->Add(buoy);
        buoy->SetColor(DarkRed);
        return buoy;
    }


}  // end namespace frydom
