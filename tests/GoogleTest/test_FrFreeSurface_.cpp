//
// Created by Lucas Letournel on 21/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;



TEST(FrFreeSurface_,regularWaveField){

    FRAME_CONVENTION fc = NWU;
    DIRECTION_CONVENTION dc = GOTO;

    double waveHeight = 0.1;
    double wavePeriod = 2.*M_PI;
    Direction waveDirection = Direction(NORTH(fc));


    // create a system
    FrOffshoreSystem_ system;

    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

    auto waveField = freeSurface->SetAiryRegularWaveField();
    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);

    std::cout<<"Direction Angle : "<<waveField->GetDirectionAngle(DEG,fc,dc)<<std::endl;
//    std::cout<<"Time : "<<waveField->GetTime()<<std::endl;

    auto waveLength = waveField->GetWaveLength();

    std::cout<<"Wave Length : "<<waveLength<<std::endl;

    std::cout<<"Elevation (0,0) : "<< waveField->GetElevation(0.,0.) << std::endl;

    std::cout<<"Elevation (Lambda,0) : "<< waveField->GetElevation(waveLength,0.) << std::endl;

    std::cout<<"Elevation (0.5Lambda,0) : "<< waveField->GetElevation(0.5*waveLength,0.) << std::endl;

    std::cout<<"Elevation (0,1.) : "<< waveField->GetElevation(0.,1.) << std::endl;



//    auto waveField = FrLinearWaveField(LINEAR_REGULAR);
//    waveField.SetRegularWaveHeight(3);
//    waveField.SetRegularWavePeriod(9);
//    waveField.SetMeanWaveDirection(0., DEG);
//
//    waveField.GetSteadyElevation(0, 0);
//
//    freeSurface->SetLinearWaveField(waveField);

}


//TEST(FrFreeSurfaceOld, regularWaveField){
//
//    FrOffshoreSystem system;
//
//    auto freeSurface = system.GetEnvironment()->GetFreeSurface();
//
//    freeSurface->SetLinearWaveField(LINEAR_REGULAR);
//
//    auto waveField = freeSurface->GetLinearWaveField();
//
//    waveField->SetRegularWaveHeight(3);
//    waveField->SetRegularWavePeriod(9);
//    waveField->SetMeanWaveDirection(0., DEG);
//
//    auto SteadyElevation = waveField->GetSteadyElevation(0, 0);
//
//
//    for (auto it=SteadyElevation.begin(); it!=SteadyElevation.end(); it++) {
//        std::cout << it << std::endl;
//    }
//
//    // Grid definition
//    auto xVect = linspace<double>(-1000., 1000., 100);
//    auto yVect = linspace<double>(-1000., 1000., 100);
//
//    auto eta = waveField->GetElevation(xVect, yVect);
//
//
//}

