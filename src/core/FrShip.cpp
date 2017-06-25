//
// Created by frongere on 21/06/17.
//

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkMate.h"
#include "FrShip.h"
#include "../propeller/FrPropeller.h"
#include "FrOffshoreSystem.h"


namespace frydom {

    void FrShip::AddPropeller(std::shared_ptr<FrPropeller> propeller) {  // FIXME : pourquoi le propeller serait shared ????

        // Adding propeller to the propeller list
        propellerlist.push_back(propeller);

        // Adding the propeller as a force in the force list
        AddForce(propeller);

    }

    void FrShip::RemovePropeller(std::shared_ptr<FrPropeller> propeller) {
        // Trying to remove objects not previously added?
        assert(std::find<std::vector<std::shared_ptr<FrPropeller>>::iterator>(propellerlist.begin(),
                                                                              propellerlist.end(),
                                                                              propeller) != propellerlist.end());

        // warning! linear time search
        propellerlist.erase(
                std::find<std::vector<std::shared_ptr<FrPropeller>>::iterator>(propellerlist.begin(),
                                                                               propellerlist.end(),
                                                                               propeller));
        RemoveForce(propeller);
    }

    chrono::ChVector<double> FrShip::GetShipVelocity() const {
        return GetCoord_dt().pos;
    }

    void FrShip::Set3DOF(bool flag) {
        if (flag) {
            Set3DOF_ON();
        } else {
            Set3DOF_OFF();
        }
    }

    void FrShip::Set3DOF_ON() {
        if (is3DOF){ return; }
        // TODO: voir si on peut pas faire quelque chose avec l'attribut flipped de ChLinkMate...
        auto plane_constraint = std::make_shared<chrono::ChLinkMatePlane>();
        auto free_surface_body = dynamic_cast<FrOffshoreSystem*>(GetSystem())->getFreeSurface()->getBody();
        plane_constraint->Initialize(shared_from_this(), free_surface_body,
                                     true,
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(),
                                     chrono::ChVector<>(0, 0, 1),
                                     chrono::ChVector<>(0, 0, -1));
        system->AddLink(plane_constraint);
        constraint3DOF = plane_constraint;
        is3DOF = true;
    }

    void FrShip::Set3DOF_OFF() {
        if (!is3DOF) { return; }

        system->RemoveLink(constraint3DOF);
        constraint3DOF->SetSystem(0);
        is3DOF = false;
    }


}  // end namespace frydom