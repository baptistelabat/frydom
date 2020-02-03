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

  /** The main components and methods of FrFreeSurface are introduced in this demo:
   *      - free surface asset definition and manipulation
   *      - the Twelfth rule tidal model
   *      - the Airy linear wavefield models : regular and irregular ones
   *
   */

  // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
  FRAME_CONVENTION fc = NWU;
  // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
  DIRECTION_CONVENTION dc = GOTO;

  // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
  FrOffshoreSystem system("demo_FreeSurface");

  // Turn this variable to false if you want to visualize an irregular wave field, rather than a regular one.
  bool Regular = false;

  //--------------------------------------------------------------------------------------------------------------
  // The free surface object contains the tidal and the wave field models, along with a visualization asset of the
  // free surface. By default, a null wave field and a twelfth rule tidal model is set.

  auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

  {
    //
    // EXAMPLE 1: Free surface asset definition and manipulation.
    //

    // It is also possible to hide the free surface asset, by using the following method. The free surface however
    // still exists, and you can still access its components.
    // system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);

    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset();

    // The free surface grid is defined here as a squared one ranging from -100m to 100m (in north and west
    // directions) with a 2m steps.
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(5);

  }

  {
    //
    // EXAMPLE 2: Tidal model
    //

    // The Twelfth rule tidal model is set by default.

    // A NoTidal model is also available with the following method. This model always return a zero tidal height.
    FreeSurface->GetTidal()->SetNoTidal();

    // The tidal model return mainly the tidal height varying in time, for long simulations. Be careful that the
    // tidal height is given accordingly to the frame convention specified.
    FreeSurface->GetTidal()->GetHeight(fc);

  }

  if (Regular) {
    //
    // EXAMPLE 3: Linear Airy Regular WaveField
    //

    // The setter give you access to the wavefield created in the free surface, in order to set the wave parameters.
    // You can choose between an optimized and a non optimized class. The optimized class is a derived class
    // of the non optimized one.

    // auto waveField = freeSurface->SetAiryRegularWaveField();
    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 2.;
    double wavePeriod = 2. * M_PI;
    Direction waveDirection = Direction(SOUTH(fc));

    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);

    // Note that you can also choose to set these parameters directly in the Setter of the Airy regular wave field
    // auto waveField = FreeSurface->SetAiryRegularOptimWaveField(waveHeight, wavePeriod, waveDirection, fc, dc);

  } else {
    //
    // EXAMPLE 4: Linear Airy Irregular WaveField
    //

    // The setter give you access to the wavefield created in the free surface, in order to set the wave parameters.
    // You can choose between an optimized and a non optimized class. The optimized class is a derived class
    // of the non optimized one.

    //    auto waveField = freeSurface->SetAiryIrregularWaveField();
    auto waveField = FreeSurface->SetAiryIrregularOptimWaveField();

    // The Airy irregular wave parameters are based on the wave spectrum chosen : Jonswap or Pierson-Moskowitz.
    // Set the JONSWAP wave spectrum : Significant height (Hs) and Peak period (Tp). A default Gamma for the Jonswap
    // spectrum is set to 3.3.
    double Hs = 3;
    double Tp = 9;
    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

    // Define the wave frequency discretization. It is based on a linear discretization within the extrema given and
    // using the number of frequency specified. With more frequency, it will be more realist but will take longer to
    // simulate.
    double w1 = 0.5;
    double w2 = 2;
    unsigned int nbFreq = 20;
    waveField->SetWaveFrequencies(w1, w2, nbFreq);

    // For a uni-directional wave, you just need to set the mean wave direction. You can also choose to set a
    // direction angle from North direction (see SetMeanWaveDirectionAngle()).
    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);

    // For a directional wave, you also have to specify the spreading factor and the refinement wanted on the
    // direction discretization. With more directions, it will be more realist but will take longer to simulate.
    // The direction discretization is based on a linear discretization. The extrema are computed automatically
    // using the spreading factor and the mean direction.
    double spreadingFactor = 10.;
    unsigned int nbDir = 10;
    waveField->SetDirectionalParameters(nbDir, spreadingFactor);

  }

  // ------------------ Run ------------------ //

  // You can change the dynamical simulation time step using.
  system.SetTimeStep(0.04);

  // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
  // the time length of the simulation (here 30s) and the distance from the camera to the objectif (100m).
  // For saving snapshots of the simulation, just turn the boolean to true.
  system.RunInViewer(30, 100, false);

}

