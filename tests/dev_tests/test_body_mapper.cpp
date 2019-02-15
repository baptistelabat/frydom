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

    std::cout << "========================= Test body mapper ===================== " << std::endl;

    std::cout << "--> Create system..." << std::endl;
    FrOffshoreSystem_ system;

    std::cout << "--> Create body..." << std::endl;
    auto body = system.NewBody();

    std::cout << "--> Read file : Platform_HDB.hdb5" << std::endl;
    auto hdb = std::make_shared<FrHydroDB_>("Platform_HDB.hdb5");

    std::cout << "--> Create eqFrame..." << std::endl;
    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(body.get());

    std::cout << "--> Link map..." << std::endl;
    hdb->Map(0, body.get(), eqFrame);

    std::cout << "--> Test Get BEMBody" << std::endl;
    auto bemBody = hdb->GetBody(body.get());
    std::cout << "BEMBody name : " << bemBody->GetName() << std::endl;

    auto eqFrame2 = hdb->GetMapper()->GetEquilibriumFrame(body.get());
    std::cout << "eqFrame x : " << eqFrame2->GetPosition(NWU).GetX() << std::endl;
    std::cout << "eqFrame y : " << eqFrame2->GetPosition(NWU).GetY() << std::endl;
    std::cout << "eqFrame z : " << eqFrame2->GetPosition(NWU).GetZ() << std::endl;


}