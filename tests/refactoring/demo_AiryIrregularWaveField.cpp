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

// Set depth to infinite
//    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(-100);

//    system.GetEnvironment()->GetOcean()->ShowSeabed(false);
//    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);


// Set the waveField to AiryRegular
    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
    auto FSAsset = freeSurface->GetFreeSurfaceGridAsset();
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);
//    FSAsset->UpdateAssetON();
    FSAsset->SetUpdateStep(5);
//    FSAsset->SetGridType(FrGridAsset::NOGRID);


// ------------------ Regular ------------------ //
////    auto waveField = freeSurface->SetAiryRegularWaveField();
//    auto waveField = freeSurface->SetAiryRegularOptimWaveField();
//    // Airy regular wave parameters
//    double waveHeight = 2.;
//    double wavePeriod = 2.*M_PI;
//    Direction waveDirection = Direction(SOUTH(fc));
//    waveField->SetWaveHeight(waveHeight);
//    waveField->SetWavePeriod(wavePeriod);
//    waveField->SetDirection(waveDirection, fc, dc);

// ------------------ Irregular ------------------ //
//    auto waveField = freeSurface->SetAiryIrregularWaveField();
    auto waveField = freeSurface->SetAiryIrregularOptimWaveField();
//    waveField->GetWaveRamp()->Deactivate();

// Set the JONSWAP wave spectrum
    double Hs = 3;
    double Tp = 9;
    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

    waveField->SetWaveFrequencies(0.5,2,20);

// Set wave direction
    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);
    waveField->SetDirectionalParameters(10, 10);



// ------------------ Run ------------------ //
    system.SetTimeStep(0.01);

    system.Initialize();

    time_t startTime = time(nullptr);

    system.RunInViewer(10, 20, false);
//    system.AdvanceTo(10);

    time_t endTime = time(nullptr) - startTime;
    char* dt = ctime(&endTime);

    std::cout<<"time spent :"<<dt;

}

