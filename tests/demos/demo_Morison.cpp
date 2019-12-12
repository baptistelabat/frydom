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

  /**
   * This demo features two cases of Morison loads applied on a structure. The first one is a basic horizontal cylinder.
   * Only one Morison element is needed to represent the Morison loads. The second structure is a semi-submersible platform
   * which requires the introduction of one element per underwater braces. All coefficients (added mass, drag and friction)
   * are given for the cylinder, but only drag coefficients are specified for the platform.
   *
   */

  // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
  FRAME_CONVENTION
      fc = NWU;
  // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
  DIRECTION_CONVENTION dc = GOTO;

  // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
  FrOffshoreSystem system("demo_Morison");

  system.GetEnvironment()->ShowSeabed(false);

  // ------------------ Wave Field ------------------ //

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

  // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
  auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

  // The free surface grid is defined here as a squared one ranging from -100m to 100m (in north and west
  // directions) with a 2m steps.
  FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);

  // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
  // update is not activated.
  FSAsset->SetUpdateStep(10);

  // The Airy regular wave parameters are its height, period and direction.
  double waveHeight = 2.;
  double wavePeriod = 2. * M_PI;
  Direction waveDirection = Direction(SOUTH(fc));

  auto waveField = FreeSurface->SetAiryRegularOptimWaveField(waveHeight, wavePeriod, waveDirection, fc, dc);

  if (true) {
    // ------------------ Cylinder ------------------ //

    // Create the cylinder
    auto cylinder = system.NewBody("Cylinder");
    double radius = 3, height = 30, mass = 1000;
    makeItCylinder(cylinder, radius, height, mass);
    cylinder->SetColor(LightGoldenRodYellow);
    cylinder->SetPosition(Position(0., 0., -0.5 * height), fc);

    cylinder->SetFixedInWorld(true);


    // ------------------ Morison Model ------------------ //

    // Several ways exist to add a Morison model to a body. Remember that a Morison model is a composition of Morison
    // elements (using a composition pattern). Morison elements can be added simply to a Morison model. Note that you
    // can also provide a Morison Force with a Morison element, if you got only one element.
    auto MorisonModel = make_morison_model(cylinder.get());

    // Define the added mass and drag Morison Coefficients.
    MorisonCoeff AddedMassCoeff(0.5, 0.7);
    MorisonCoeff DragCoeff(1.5, 1.7);
    double frictionCoeff = 0.1;

    // Add an element, with parameters corresponding to the cylinder.
    MorisonModel->AddElement(FrFrame(), height, 2. * radius, AddedMassCoeff, DragCoeff, frictionCoeff);

    // Instantiate a Morison Force, using a Morison model, and add it to the cylinder
    auto MorisonForce = make_morison_force("MorisonForce", cylinder, MorisonModel);

    // Make the asset (a vector) for the Morison force visible
    MorisonForce->ShowAsset(true);
  } else {
    // ------------------ platform ------------------ //
    auto Platform = system.NewBody("platform");

    auto platformMesh = FrFileSystem::join({system.config_file().GetDataFolder(), "ce/platform/Platform_GVA7500.obj"});
    Platform->AddMeshAsset(platformMesh);

    Platform->SetFixedInWorld(true);

    // Morison Model
    auto MorisonModel = make_morison_model(Platform.get());

    // Define the added mass and friction coefficients.
    MorisonCoeff AddedMassCoeff(0., 0.); // The added mass is not taken into account
    double frictionCoeff = 0.; // No friction either

    MorisonModel->SetExtendedModel(false); // The added mass is not taken into account

    auto cog = Platform->GetCOG(NWU);
    double diameter = 0.01;
    // Morison elements
    MorisonModel->AddElement(Position(-54.4, 31.04, -17.10) - cog, Position(54.4, 31.04, -17.10) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1551., 3456.), frictionCoeff);
    MorisonModel->AddElement(Position(-54.4, -31.04, -17.10) - cog, Position(54.4, -31.04, -17.10) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1551., 3456.), frictionCoeff);

    MorisonModel->AddElement(Position(-32.3, 31.04, -12) - cog, Position(-32.3, 31.04, 0.) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1352., 2501.), frictionCoeff);
    MorisonModel->AddElement(Position(32.3, 31.04, -12) - cog, Position(32.3, 31.04, 0.) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1352., 2501.), frictionCoeff);
    MorisonModel->AddElement(Position(-32.3, -31.04, -12) - cog, Position(-32.3, -31.04, 0.) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1352., 2501.), frictionCoeff);
    MorisonModel->AddElement(Position(32.3, -31.04, -12) - cog, Position(32.3, -31.04, 0.) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(1352., 2501.), frictionCoeff);

    MorisonModel->AddElement(Position(-32.3, 23.04, -17.10) - cog, Position(-32.3, -23.04, -17.10) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(123., 2928.), frictionCoeff);
    MorisonModel->AddElement(Position(32.3, 23.04, -17.10) - cog, Position(32.3, -23.04, -17.10) - cog, diameter,
                             AddedMassCoeff, MorisonCoeff(123., 2928.), frictionCoeff);

    // Instantiate a Morison Force, using a Morison model, and add it to the cylinder
    auto MorisonForce = make_morison_force("MorisonForce", Platform, MorisonModel);

    // Make the asset (a vector) for the Morison force visible
    MorisonForce->ShowAsset(true);
  }

  // ------------------ Run ------------------ //

  // You can change the dynamical simulation time step using.
  system.SetTimeStep(0.04);

  // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
  // the time length of the simulation (here 150) and the distance from the camera to the objectif (155m).
  // For saving snapshots of the simulation, just turn the boolean to true.
  system.RunInViewer(150, 155, false);


}
