//
// Created by camille on 23/01/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    std::cout << " ========================= Test hydrostatic =================== " << std::endl;

    std::cout << "--> Create a system ..." << std::endl;
    FrOffshoreSystem_ system;

    std::cout << "--> Create a body ..." << std::endl;
    auto body = system.NewBody();

    std::cout << "--> Load HDB ... " << std::endl;
    auto hdb = make_hydrodynamic_database("DeepSeaStavanger.hdb5");

    std::cout << "--> Eq Frame ... " << std::endl;
    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());
    system.AddPhysicsItem(eqFrame);

    std::cout << "--> Set Map ..." << std::endl;
    hdb->Map(0, body.get(), eqFrame);

    std::cout << "--> Create hydrostatic force ... " << std::endl;
    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    std::cout << "--> Initialize force ..." << std::endl;
    forceHst->Initialize();

    std::cout << "K33 = " << forceHst->GetStiffnessMatrix()->GetK33() << std::endl;
    std::cout << "K44 = " << forceHst->GetStiffnessMatrix()->GetK44() << std::endl;
    std::cout << "K55 = " << forceHst->GetStiffnessMatrix()->GetK55() << std::endl;
    std::cout << "K34 = " << forceHst->GetStiffnessMatrix()->GetK34() << std::endl;
    std::cout << "K35 = " << forceHst->GetStiffnessMatrix()->GetK35() << std::endl;
    std::cout << "K45 = " << forceHst->GetStiffnessMatrix()->GetK45() << std::endl;

    std::cout << "====================== END =====================" << std::endl;
}
