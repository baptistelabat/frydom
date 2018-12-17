//
// Created by Lucas Letournel on 04/12/18.
//

#include "frydom/frydom.h"
#include <ctime>

using namespace frydom;


int main(int argc, char* argv[]) {
    // Frame convention
    FRAME_CONVENTION fc = NWU;
    // Wave direction convention
    DIRECTION_CONVENTION dc = GOTO;

// create a system
    FrOffshoreSystem_ system;

    auto timeRamp = system.GetEnvironment()->GetTimeRamp();
    timeRamp->Activate();

// Set depth to infinite
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-100);

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

