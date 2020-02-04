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

int main(int argc, char *argv[]) {

  // System
  FrOffshoreSystem system;

  // TODO: il serait bien de pouvoir generer des corps et des cables (objets physiques) directement en appelant des
  // methodes de FrOffshoreSystem afin que ces derniers soient directement enregistres aupres du systeme...
  // Le probleme de cela est que du coup, il faut avoir une methode (enfin plutot une factory) pour chaque type d'objet...

  // TODO: Une fonction a appeler toujours en debut de script FRyDoM serait frydomInit --> cree le systeme ???

  // Body
  double radius = 1.5;
  double mass = 100;
  auto sphere = std::make_shared<FrSphere>(radius, mass); // TODO: permettre de donner une masse

  sphere->SetPos(chrono::ChVector<>(100, 0, 0));
  sphere->SetName("sphere");
  system.AddBody(sphere);

  // FEA cable
  auto cable = std::make_shared<FrDynamicCable>();  // TODO : avoir un constructeur permettant d'obliger a specifier les pptes du cable sans en oublier
  cable->SetDiameter(0.05);
  cable->SetCableLength(10);
  cable->SetNumberOfElements(20);
  cable->SetYoungModulus(1e10);
  cable->SetLinearDensity(50);
  cable->SetRayleighDamping(0.1);
  cable->SetDrawRadius(0.1);
  cable->SetDrawNodeSize(0.3);

  // Creating nodes to fix on boundary of the cable on the world and the other on the sphere
  // TODO: Ne pas creer les noeuds explicitement mais utiliser des methodes de FrCable type AttachToBody(body, relpos)
  // pour fixer le cable a un corps avec creation automatique du FrNode et du FEANode
  // TODO: plutot que de faire appel explicitement a WorlBody, ajouter une methode FixToWorld
  // TODO: permettre de specifier une position relative du noeud par rapport au repere de reference du corps

  auto anchor = system.GetWorldBody()->CreateNode();  // Ici, preciser la coordonnee relative
  cable->SetStartingNode(anchor);


  auto node = sphere->CreateNode();
  cable->SetEndingNode(node);

  system.Add(cable);

  cable->Initialize();

  // TODO : les lignes suivantes jusqu'a app devraient etre gerees par defaut suivant qu'on a un cable ou pas ...
  // TODO: voir si on peut specifier ces reglages pour un modele dans cable
  // Si NON, Peut-on avoir un reglage auto du solveur MINRES
  system.SetSolverType(
      chrono::ChSolver::Type::MINRES);  // TODO: voir si on peut regler ce solveur pour des simulations sans cable
  system.SetSolverWarmStarting(true);
  system.SetMaxItersSolverSpeed(
      1000);  // TODO: mettre en place une adaptation lorsqu'on a un residu du solveur trop important
  system.SetMaxItersSolverStab(200);
  system.SetTolForce(1e-13);
  auto msolver = std::static_pointer_cast<chrono::ChSolverMINRES>(system.GetSolver());
  msolver->SetVerbose(true);
//    msolver->SetVerbose(false);
  msolver->SetDiagonalPreconditioning(true);

//    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
  system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
  system.SetupInitial();

  auto app = FrIrrApp(system, 50);

  app.SetTimestep(0.01); // TODO: doit etre le pas de temps par defaut...
  app.AddTypicalCamera(irr::core::vector3df(0, 150, -50), irr::core::vector3df(0, 0, -50));
  app.Run();

  return 0;
}