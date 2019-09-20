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
    template <typename OffshoreSystemType>
    class FrBody;


    template <typename OffshoreSystemType>
    void makeItBox(std::shared_ptr<FrBody<OffshoreSystemType>> body, double xSize, double ySize, double zSize, double mass);

    template <typename OffshoreSystemType>
    void makeItCylinder(std::shared_ptr<FrBody<OffshoreSystemType>> body, double radius, double height, double mass);

    template <typename OffshoreSystemType>
    void makeItSphere(std::shared_ptr<FrBody<OffshoreSystemType>> body, double radius, double mass);

    template <typename OffshoreSystemType>
    std::shared_ptr<FrBody<OffshoreSystemType>> make_BoxBody(double xSize, double ySize, double zSize, double mass);

    template <typename OffshoreSystemType>
    std::shared_ptr<FrBody<OffshoreSystemType>> make_CylinderBody(double radius, double height, double mass);

    template <typename OffshoreSystemType>
    std::shared_ptr<FrBody<OffshoreSystemType>> make_SphereBody(double radius, double mass);


    // TODO : faire le make_ConeBody ---> mat d'eolienne...

    // TODO : porter le python FRyDoM pour les calculs d'inertie...

    // TODO : renommer ce fichier en FrStandardBodies


}  // end namespace frydom


#endif //FRYDOM_FREASYBODIES_H
