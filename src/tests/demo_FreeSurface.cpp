// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// demo code for free surface definition
//
// =============================================================================

//#include <iostream>
//#include <unistd.h>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "../environment/waves/FrFlatFreeSurface.h"


int main(int argc, char* argv[]) {
    // Creating the system
    chrono::ChSystemNSC system;

    // Creating the free surface
    frydom::environment::FrFlatFreeSurface free_surface(2);

    free_surface.Initialize(0, 6, 1);


    chrono::geometry::ChTriangleMeshConnected mesh = free_surface.getMesh();

    chrono::ChVector<>* vertex;

    return 0;

}

