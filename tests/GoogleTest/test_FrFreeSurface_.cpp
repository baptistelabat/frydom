//
// Created by Lucas Letournel on 21/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;



TEST(FrFreeSurface_,regularWaveField){

    // Frame convention
    FRAME_CONVENTION fc = NWU;
    // Wave direction convention
    DIRECTION_CONVENTION dc = GOTO;

    // create a system
    FrOffshoreSystem_ system;

    // Airy regular wave parameters
    double waveHeight = 1.;
    double wavePeriod = 2.*M_PI;
    Direction waveDirection = Direction(NORTH(fc));

    auto omega = 2.*M_PI/wavePeriod;
    auto k = omega * omega / system.GetGravityAcceleration();
    auto waveLength =  2.*M_PI/k;

    // Set depth to infinite (H>>3*waveLength)
    system.GetEnvironment()->GetOcean()->GetSeabed()->SetDepth(4*waveLength);

    // Set the waveField to AiryRegular
    auto freeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
    auto waveField = freeSurface->SetAiryRegularWaveField();
    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);

    // Test getters
    EXPECT_DOUBLE_EQ(waveHeight, waveField->GetWaveHeight());
    EXPECT_DOUBLE_EQ(wavePeriod, waveField->GetWavePeriod(S));
    EXPECT_DOUBLE_EQ(waveLength, waveField->GetWaveLength());
    EXPECT_DOUBLE_EQ(0.,waveField->GetDirectionAngle(DEG,fc,dc));
    Direction testDirection = Direction(NORTH(fc)) - waveField->GetDirection(fc,dc);
    EXPECT_TRUE(testDirection.isZero());

    // test at T = 0s
    //          test Elevation
    EXPECT_NEAR(0, waveField->GetElevation(0.,0.), 1e-8);
    EXPECT_NEAR(0, waveField->GetElevation(waveLength,0.), 1e-8);
    EXPECT_NEAR(0,-waveField->GetElevation(0.5*waveLength,0.), 1e-8);
    EXPECT_NEAR(0, waveField->GetElevation(0,1.), 1e-8);
    //          test Velocity
    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVx(), 1e-8);
    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVy(), 1e-8);
    EXPECT_NEAR(waveHeight*omega, -waveField->GetVelocity(0.,0.,0.).GetVz(), 1e-8);
    //          test Acceleration
    EXPECT_NEAR(waveHeight*omega*omega, -waveField->GetAcceleration(0.,0.,0.).GetAccX(), 1e-8);
    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccY(), 1e-8);
    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccZ(), 1e-8);

    // test at T = Tw/4
    system.AdvanceOneStep(0.25*wavePeriod);
    EXPECT_DOUBLE_EQ(0.25*wavePeriod,system.GetTime());
    //          test Elevation
    EXPECT_DOUBLE_EQ(waveHeight, -waveField->GetElevation(0.,0.));
    EXPECT_DOUBLE_EQ(waveHeight, -waveField->GetElevation(waveLength,0.));
    EXPECT_DOUBLE_EQ(waveHeight,  waveField->GetElevation(0.5*waveLength,0.));
    EXPECT_DOUBLE_EQ(waveHeight, -waveField->GetElevation(0,1.));
    //          test Velocity
    EXPECT_DOUBLE_EQ(waveHeight*omega, -waveField->GetVelocity(0.,0.,0.).GetVx());
    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVy(), 1e-8);
    EXPECT_NEAR(0., waveField->GetVelocity(0.,0.,0.).GetVz(), 1e-8);
    //          test Acceleration
    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccX(), 1e-8);
    EXPECT_NEAR(0., waveField->GetAcceleration(0.,0.,0.).GetAccY(), 1e-8);
    EXPECT_NEAR(waveHeight*omega*omega, waveField->GetAcceleration(0.,0.,0.).GetAccZ(), 1e-8);

//    // test that we have the correct wavePeriod
//    system.AdvanceOneStep(wavePeriod);
//    EXPECT_DOUBLE_EQ(waveHeight, -waveField->GetElevation(0.,0.));

    // test GOTO
    system.AdvanceOneStep(0.25*wavePeriod); // to be sure the wave is not symmetric around the origin
    auto testElevation = waveField->GetElevation(-0.1*waveLength,0.);
    system.AdvanceOneStep(0.1*wavePeriod);
    EXPECT_DOUBLE_EQ(testElevation, waveField->GetElevation(0.,0.));

    //



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

