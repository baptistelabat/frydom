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
#include <ctime>

using namespace frydom;


int main(int argc, char* argv[]) {

    // Define the frame convention (North-West-Up or North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM)
    DIRECTION_CONVENTION dc = GOTO;

    // create an offshore system
    FrOffshoreSystem_ system;

    // Activate the time ramp, embedded in the environment
    auto timeRamp = system.GetEnvironment()->GetTimeRamp();
    timeRamp->Activate();

    // Set the water depth to infinite. Note that the water depth is set with a frame convention. In NWU, it is then set
    // negatively.
    system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

    auto seabed = system.GetEnvironment()->GetOcean()->GetSeabed();
    seabed->GetBathymetry(fc);

//    system.GetEnvironment()->GetOcean()->ShowSeabed(false);
//    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);


// Free surface grid definition
    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
    auto FSAsset = freeSurface->GetFreeSurfaceGridAsset();
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);
    FSAsset->SetUpdateStep(10);


// ------------------ Regular ------------------ //
////    auto waveField = freeSurface->SetAiryRegularWaveField();
//    auto waveField = freeSurface->SetAiryRegularOptimWaveField();
//    // Airy regular wave parameters
//    double waveHeight = 2.;    double wavePeriod = 2.*M_PI;
//    Direction waveDirection = Direction(SOUTH(fc));
//    waveField->SetWaveHeight(waveHeight);
//    waveField->SetWavePeriod(wavePeriod);
//    waveField->SetDirection(waveDirection, fc, dc);

// ------------------ Irregular ------------------ //
//    auto waveField = freeSurface->SetAiryIrregularWaveField();
    auto waveField = freeSurface->SetAiryIrregularOptimWaveField();

// Set the JONSWAP wave spectrum
    double Hs = 3;    double Tp = 9;
    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

    double w1 = 0.5; double w2 = 2; unsigned int nbFreq = 20;
    waveField->SetWaveFrequencies(w1,w2,nbFreq);

// Set wave direction
    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);
    double spreadingFactor = 10.;    unsigned int nbDir = 10;
    waveField->SetDirectionalParameters(nbDir, spreadingFactor);


// ------------------ Run ------------------ //
    system.SetTimeStep(0.01);

    system.Initialize();

    time_t startTime = time(nullptr);

    system.RunInViewer(30, 20, false);
//    system.AdvanceTo(10);

    time_t endTime = time(nullptr) - startTime;
    char* dt = ctime(&endTime);

    std::cout<<"time spent :"<<dt;

}

