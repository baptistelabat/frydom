//
// Created by Lucas Letournel on 21/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrFreeSurface_,regularWaveField){
    // create a system
    FrOffshoreSystem_ system;

    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

//    auto waveField = FrLinearWaveField(LINEAR_REGULAR);
//    waveField.SetRegularWaveHeight(3);
//    waveField.SetRegularWavePeriod(9);
//    waveField.SetMeanWaveDirection(0., DEG);
//
//    waveField.GetSteadyElevation(0, 0);
//
//    freeSurface->SetLinearWaveField(waveField);

}

