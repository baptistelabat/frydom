//
// Created by frongere on 12/10/17.
//

#include <frydom/utils/FrIrrApp.h>
#include <frydom/cable/FrDynamicCable.h>
#include <chrono/solver/ChSolverMINRES.h>
#include <chrono_fea/ChLinkPointFrame.h>
#include "frydom/core/FrCore.h"
#include "frydom/misc/FrUtils.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // System
    FrOffshoreSystem system;

    // Body
    double radius = 1.5;
    double density = 1000;
    auto sphere = std::make_shared<FrSphere>(radius, density);

    sphere->SetPos(chrono::ChVector<>(100, 0, 0));
    system.AddBody(sphere);

    // FEA cable
    auto cable = std::make_shared<FrDynamicCable>();
    cable->SetDiameter(0.05);
    cable->SetCableLength(10);
    cable->SetNumberOfElements(20);
    cable->SetYoungModulus(1e8);
    cable->SetLinearDensity(50);
    cable->SetRayleighDamping(0.1);
    cable->SetDrawRadius(0.1);
    cable->SetDrawNodeSize(0.3);

    auto anchor = system.GetWorldBody()->CreateNode();
    cable->SetStartingNode(anchor);

    auto node = sphere->CreateNode();
    cable->SetEndingNode(node);

    system.Add(cable);

    cable->Initialize();


    auto hinge_anchor = std::make_shared<ChLinkPointFrame>();  // TODO: avoir methode cable.attachToNode... --> cree la contrainte...
    hinge_anchor->Initialize(cable->GetStartingNodeFEA(), system.GetWorldBody());
    system.Add(hinge_anchor);

    auto hinge_sphere = std::make_shared<ChLinkPointFrame>();  // TODO: avoir methode cable.attachToNode... --> cree la contrainte...
    hinge_sphere->Initialize(cable->GetEndingNodeFEA(), sphere);
    system.Add(hinge_sphere);



    system.SetSolverType(chrono::ChSolver::Type::MINRES);  // TODO: voir si on peut regler ce solveur pour des simulations sans cable
    system.SetSolverWarmStarting(true);
    system.SetMaxItersSolverSpeed(200);
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
    msolver->SetVerbose(true);
//    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);


    auto app = FrIrrApp(system, 50);

    system.SetupInitial();

    app.SetTimestep(0.01);
    app.Run();




    
    return 0;
}