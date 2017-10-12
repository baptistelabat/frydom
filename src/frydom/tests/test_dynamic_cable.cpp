//
// Created by frongere on 11/10/17.
//

#include <frydom/core/FrOffshoreSystem.h>
#include <frydom/utils/FrIrrApp.h>
#include <chrono/solver/ChSolverMINRES.h>
#include <chrono_fea/ChLinkPointFrame.h>
#include "frydom/cable/FrDynamicCable.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // TODO: essayer de reproduire la simulation de demo_cable_ANCF

    // Creating the system
    FrOffshoreSystem system;

    // Creating a fixed body
//    auto body1 = system.GetWorldBody();
    auto body1 = std::make_shared<FrBody>();
    body1->SetBodyFixed(true);
    system.AddBody(body1);

    auto cable = std::make_shared<FrDynamicCable>();
    cable->SetCableLength(30);  // TODO: augmenter la taille et la discretisation une fois le cable attache...
    cable->SetLinearDensity(30);
    cable->SetDiameter(0.05);
    cable->SetNumberOfElements(40);
    cable->SetYoungModulus(1e9);
    cable->SetRayleighDamping(0.5);
//    cable->Set

    auto node1 = body1->CreateNode(chrono::ChVector<double>(0, 0, 0));  // TODO: rendre polymorphe pour specifier juste (x, y, z)... (position relative)
    auto node2 = body1->CreateNode(chrono::ChVector<double>(20, 0, 0));

    cable->SetStartingNode(node1);
    cable->SetEndingNode(node2);

    cable->Initialize();
    system.Add(cable);

    // Adding constraint to the cable
    auto constraint_hinge = std::make_shared<ChLinkPointFrame>();  // TODO: avoir methode cable.attachToNode... --> cree la contrainte...
    constraint_hinge->Initialize(cable->GetStartingNodeFEA(), body1);
    system.Add(constraint_hinge);  // FIXME : voir pourquoi cette contrainte ne semble pas active  ????...

    auto constraint_hinge2 = std::make_shared<ChLinkPointFrame>();  // TODO: avoir methode cable.attachToNode... --> cree la contrainte...
    constraint_hinge2->Initialize(cable->GetEndingNodeFEA(), body1);
    system.Add(constraint_hinge2);  // FIXME : voir pourquoi cette contrainte ne semble pas active  ????...



    // TODO: la visualisation doit pouvoir se regler directement dans l'objet...
    auto visbeam = std::make_shared<ChVisualizationFEAmesh>(*cable.get());
    visbeam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    visbeam->SetColorscaleMinMax(-0.4, 0.4);
    visbeam->SetSmoothFaces(true);
    visbeam->SetWireframe(false);
    cable->AddAsset(visbeam);


    // VISUALISATION
    frydom::FrIrrApp app(&system, L"Frydom vizualization based on Irrlicht");
    app.AddTypicalLights();
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 30), irr::core::vector3df(1, 0, -1));
    app.AddTypicalLogo("../src/frydom/tests/data/frydom_logo.png");

    app.AssetBindAll();
    app.AssetUpdateAll();

    system.SetupInitial();

    app.SetTimestep(0.01);
//    app.SetTryRealtime(false);

    system.SetSolverType(chrono::ChSolver::Type::MINRES);  // TODO: voir si on peut regler ce solveur pour des simulations sans cable
    system.SetSolverWarmStarting(true);
    system.SetMaxItersSolverSpeed(2000);
    system.SetMaxItersSolverStab(400);
    system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
//    msolver->SetVerbose(true);
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED); // FONCTIONNE
//    system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);

//    app.SetVideoframeSave(capture_video);

    while (app.GetDevice()->run()) {

        app.BeginScene();
        app.DrawAll();
        app.DoStep();
        app.EndScene();
        std::cout << "Time: " << system.GetChTime() << "\n";

    }


    return 0;
}