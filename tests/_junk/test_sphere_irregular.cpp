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
#include "matplotlibcpp.h"

using namespace frydom;
using namespace mathutils;

int main(int argc, char* argv[]) {

    // System
    FrOffshoreSystem system;

    // Environment
    system.GetEnvironment()->SetWaterDensity(1000.);
    system.GetEnvironment()->GetFreeSurface()->SetGrid(-20., 20., 5.);

    // Set Free surface & wave field

    auto freeSurface = system.GetEnvironment()->GetFreeSurface();
    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = freeSurface->GetLinearWaveField();

    waveField->SetWaveSpectrum(JONSWAP);

    auto waveSpectrum = dynamic_cast<FrJonswapWaveSpectrum*>(waveField->GetWaveSpectrum());
    waveSpectrum->SetHs(1.);
    waveSpectrum->SetTp(6.2);
    waveSpectrum->SetGamma(1.);

    double wmin = 0.1;
    double wmax = 3.0;
    unsigned int nbFreq = 80;
    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);
    waveField->SetMeanWaveDirection(0., DEG);


    //freeSurface->SetWaveField(waveField);

    // Body
    auto sphere = std::make_shared<FrShip>();
    sphere->SetName("sphere");
    sphere->SetHydroMesh("sphere.obj", true);

    sphere->SetInertiaXX(chrono::ChVector<double>(1.690e6, 1.690e6, 2.606e6));
    sphere->SetMass(2.618e5);     
    sphere->SetCOG(chrono::ChVector<double>(0., 0., -2.));
    sphere->SetEquilibriumFrame(WorldFixed, chrono::ChVector<double>(0., 0., -2.));

    system.AddBody(sphere);

    sphere->Set3DOF_ON(chrono::ChVector<>(1, 0, 0)); // FIXME : need to be adapted to select only one dof

    sphere->Log().SetNameAndDescription("SphereLog", "Message of the sphere");
    sphere->SetLogDefault();

    // Hydrostatic
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 7.6947e5;
    double K44 = 5.1263e6;
    hstStiffness->SetDiagonal(K33, K44, K44);
    sphere->AddForce(hstForce);

    // Hydrodynamic load
    FrHydroDB HDB = LoadHDB5("sphere_hdb.h5");

    auto hydroMapper = HDB.GetMapper();
    system.SetHydroMapper(hydroMapper);
    hydroMapper->Map(sphere, 0);

    auto radModel = std::make_shared<FrRadiationConvolutionModel>(&HDB, &system);
    radModel->AddRadiationForceToHydroBody(sphere);

    // Excitation force
    auto waveProbe = waveField->NewWaveProbe(0, 0);
    waveProbe->Initialize();

    auto excForce = std::make_shared<FrLinearExcitationForce>();
    sphere->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);

    // PTO / Linear Damping
    auto hydroDampingForce = std::make_shared<FrLinearDamping>();
    hydroDampingForce->SetDiagonalTranslationDamping(0, 0, 9.0080857e4);
    sphere->AddForce(hydroDampingForce);

    // Numerical scheme
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);

    double time;
    double dt = 0.02;

    std::vector<double> heave;
    std::vector<double> vtime;
    chrono::ChVector<double> position;

    system.SetStep(dt);
    system.Initialize();

    time = -dt;

    while (time < 200.) {

        time += dt;

        system.DoStepDynamics(dt);


    }

}