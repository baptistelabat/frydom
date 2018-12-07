//
// Created by Lucas Letournel on 04/12/18.
//

#include "frydom/frydom.h"

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
//    system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset()->SetGridType(FrGridAsset::NOGRID);
//    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGridType(FrGridAsset::NOGRID);

// Set the waveField to AiryRegular
    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
    auto FSAsset = freeSurface->GetFreeSurfaceGridAsset();
    FSAsset->SetGrid(-40., 40, 2, -20, 20, 1);
    FSAsset->UpdateAssetON();
//    FSAsset->SetGridType(FrGridAsset::NOGRID);


//    auto waveField = freeSurface->SetAiryRegularWaveField();
//    // Airy regular wave parameters
//    double waveHeight = 2.;
//    double wavePeriod = 2.*M_PI;
//    Direction waveDirection = Direction(SOUTH(fc));
//    waveField->SetWaveHeight(waveHeight);
//    waveField->SetWavePeriod(wavePeriod);
//    waveField->SetDirection(waveDirection, fc, dc);

    auto waveField = freeSurface->SetAiryIrregularWaveField();
//    waveField->GetWaveRamp()->Deactivate();

// Set the JONSWAP wave spectrum
    double Hs = 3;
    double Tp = 9;
    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

    waveField->SetWaveFrequencies(0.5,2,4);

// Set wave direction
    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);
    waveField->SetDirectionalParameters(5, 10);

    system.SetTimeStep(0.01);

    system.Initialize();
    system.RunInViewer(-10, 20, false);


}

