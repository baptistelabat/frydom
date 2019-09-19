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


    template <typename OffshoreSystemType>
    void FrMooringBuoy<OffshoreSystemType>::FrSphereNonLinearHydrostaticForce::Compute(double time) {
        auto m_buoy = dynamic_cast<FrMooringBuoy*>(this->m_body);
        Force Gvector(0.,0.,-m_buoy->GetSystem()->GetGravityAcceleration());
        auto rho_water = this->m_body->GetSystem()->GetEnvironment()->GetOcean()->GetDensity();
        // FIXME : appliquer la force au centre de poussée et non au centre de gravité : théoriquement aucun effet, mais plus propre.
        SetForceInWorldAtCOG(-m_buoy->GetVolume()*rho_water*Gvector, NWU);
    }

    template <typename OffshoreSystemType>
    FrMooringBuoy<OffshoreSystemType>::FrMooringBuoy(double radius, double mass, bool visual_asset, double damping) {
        m_radius = radius;
        m_hydrostaticForce = std::make_shared<FrSphereNonLinearHydrostaticForce>();
        AddExternalForce(m_hydrostaticForce);

        m_dampingForce = std::make_shared<FrLinearDamping>(WATER,false);
        m_dampingForce->SetDiagonalDamping(damping,damping,damping,damping,damping,damping);
        AddExternalForce(m_dampingForce);

        // Properties of the sphere
        double inertia = (2.0 / 5.0) * mass * radius * radius;

        //FIXME: passer par makeItSphere pour la suite, si possible...
//            makeItSphere(this,radius,mass);

        // Building the Chrono body
        this->SetInertiaTensor(FrInertiaTensor(mass, inertia, inertia, inertia, 0., 0., 0., Position(), NWU));

        // Collision
        auto collisionModel = this->m_chronoBody->GetCollisionModel();
        collisionModel->ClearModel();
        collisionModel->AddSphere(radius, chrono::ChVector<double>());  // TODO: permettre de specifier les coords relatives dans le modele !!
        collisionModel->BuildModel();
        this->AllowCollision(true);  // A retirer ?
        this->SetSmoothContact();  // Smooth contact by default

        // Asset
      this->AddSphereShape(radius);
    }

    template <typename OffshoreSystemType>
    void FrMooringBuoy<OffshoreSystemType>::Update() {
        computeVolume();
    }

    template <typename OffshoreSystemType>
    double FrMooringBuoy<OffshoreSystemType>::computeDraft() {

        auto eta = this->GetSystem()->GetEnvironment()->GetOcean()->GetFreeSurface()->GetPosition(this->GetPosition(NWU), NWU);
        auto zh = this->GetPosition(NWU).GetZ() - eta;

        if (zh<-m_radius) { return m_radius;}
        if (zh>m_radius) {return -m_radius;}
        else {return -zh;}

    }

    template <typename OffshoreSystemType>
    std::shared_ptr<FrMooringBuoy<OffshoreSystemType>>
    make_mooring_buoy(FrOffshoreSystem<OffshoreSystemType>* system, double radius, double mass, bool visual_asset, double damping){
        auto buoy = std::make_shared<FrMooringBuoy<OffshoreSystemType>>(radius, mass, visual_asset, damping);
        system->Add(buoy);
        buoy->SetColor(DarkRed);
        return buoy;
    }


}  // end namespace frydom
