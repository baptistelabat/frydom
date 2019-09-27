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


#ifndef FRYDOM_FREASYBODIES_H
#define FRYDOM_FREASYBODIES_H

#include <memory>

namespace frydom {

    // Forward declaration
    class FrBody;


    void makeItBox(std::shared_ptr<FrBody> body, double xSize, double ySize, double zSize, double mass);

    void makeItCylinder(std::shared_ptr<FrBody> body, double radius, double height, double mass);

    void makeItSphere(std::shared_ptr<FrBody> body, double radius, double mass);


    std::shared_ptr<FrBody> make_BoxBody(const std::string& name, double xSize, double ySize, double zSize, double mass);

    std::shared_ptr<FrBody> make_CylinderBody(const std::string& name, double radius, double height, double mass);

    std::shared_ptr<FrBody> make_SphereBody(const std::string& name, double radius, double mass);


    // TODO : faire le make_ConeBody ---> mat d'eolienne...

    // TODO : porter le python FRyDoM pour les calculs d'inertie...

    // TODO : renommer ce fichier en FrStandardBodies


}  // end namespace frydom


#endif //FRYDOM_FREASYBODIES_H
