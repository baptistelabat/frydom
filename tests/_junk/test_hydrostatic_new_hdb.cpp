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

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ========================= Test hydrostatic =================== " << std::endl;

    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

    std::cout << "--> Create a system ..." << std::endl;
    FrOffshoreSystem system;

    std::cout << "--> Create a body ..." << std::endl;
    auto body = system.NewBody();

    std::cout << "--> Load HDB ... " << std::endl;
    auto hdb = make_hydrodynamic_database(resources_path.resolve("Platform_HDB.hdb5").path());

    std::cout << "--> Eq Frame ... " << std::endl;
    auto eqFrame = make_equilibrium_frame(body, &system);


    std::cout << "--> Set Map ..." << std::endl;
    hdb->Map(0, body.get(), eqFrame);

    std::cout << "--> Create hydrostatic force ... " << std::endl;
    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    std::cout << "--> Initialize force ..." << std::endl;
    forceHst->Initialize();

    std::cout << "K33 = " << forceHst->GetStiffnessMatrix().GetK33() << std::endl;
    std::cout << "K44 = " << forceHst->GetStiffnessMatrix().GetK44() << std::endl;
    std::cout << "K55 = " << forceHst->GetStiffnessMatrix().GetK55() << std::endl;
    std::cout << "K34 = " << forceHst->GetStiffnessMatrix().GetK34() << std::endl;
    std::cout << "K35 = " << forceHst->GetStiffnessMatrix().GetK35() << std::endl;
    std::cout << "K45 = " << forceHst->GetStiffnessMatrix().GetK45() << std::endl;

    std::cout << "====================== END =====================" << std::endl;
}
