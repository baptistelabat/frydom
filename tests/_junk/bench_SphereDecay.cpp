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


void ValidationResults(const std::vector<double> vtime, const std::vector<double> heave, const std::string path) {

    FrHDF5Reader db("bench_sphere_decay_interp.h5");


    auto time_bench = db.ReadDoubleArraySTD(path + "/time");
    auto heave_bench = db.ReadDoubleArraySTD(path + "/motion");

    auto x = std::make_shared<std::vector<double>>();
    auto y = std::make_shared<std::vector<double>>();

    x = std::make_shared<std::vector<double>>(vtime);
    y = std::make_shared<std::vector<double>>(heave);

    Interp1dLinear<double, double> interpolator;
    interpolator.Initialize(x, y);
    auto heave_interp = interpolator.Eval(time_bench[0]);

    double verr;
    double err_max = 0.;
    double err_mean = 0.;

    for (unsigned int i=0; i < time_bench[0].size(); i++) {
        verr = std::fabs(heave_bench[0][i]- heave_interp[i]);
        err_max = std::max(err_max, verr);
        err_mean += verr;

        //std::cout << " time = " << time_bench[0][i] << " , i = " << i;
        //std::cout << ", heave = " << heave_interp[i] << ", heave_bench = " << heave_bench[0][i];
        //std::cout << ", err = " << verr << " , error max = " << err_max << std::endl;

    }

    err_mean = err_mean / float(time_bench[0].size());

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Validation results for test case " << path << std::endl;
    std::cout << "Error max : " << err_max << std::endl;
    std::cout << "Error mean : " << err_mean << std::endl;

    // Plot results
    matplotlibcpp::named_plot("results", vtime, heave);
    matplotlibcpp::named_plot("benchmark", time_bench[0], heave_bench[0], "--");
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel("heave motion (m)");
    matplotlibcpp::xlim(0., 40.);
    matplotlibcpp::grid;
    matplotlibcpp::legend();
    matplotlibcpp::show();
}

int main(int argc, char* argv[]) {

    std::cout << " ------------------------------------- \n"
                 " Run Sphere Decay Test Benchmark \n"
                 " ------------------------------------- " << std::endl;
    // System
    FrOffshoreSystem system;

    // Environment
    system.GetEnvironment()->SetWaterDensity(1000.);
    system.GetEnvironment()->GetFreeSurface()->SetGrid(-20, 20, 5);

    // Body
    auto sphere = std::make_shared<FrShip>();
    sphere->SetName("sphere");
    sphere->SetHydroMesh("sphere.obj", true);
    sphere->SetEquilibriumFrame(BodyFixed, chrono::ChVector<>());
    sphere->SetInertiaXX(chrono::ChVector<double>(1.690e6, 1.690e6, 2.606e6));
    sphere->SetMass(2.618e5);
    sphere->SetCOG(chrono::ChVector<double>(0., 0., -2.));

    system.AddBody(sphere);

    sphere->Set3DOF_ON(chrono::ChVector<>(1, 0, 0)); // FIXME : need to be adapted to select only one dof

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

    // Numerical scheme
    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);

    // Run simulation

    std::vector<std::string> list_case = {"decay1", "decay2", "decay3", "decay4"};
    std::vector<double> list_zi = {0., 1., 3., 5.};

    double time;
    double dt = 0.005;
    std::vector<double> heave;
    std::vector<double> vtime;

    system.SetStep(dt);
    system.Initialize();

    auto icase = 1;
    if (argv[1]) { icase = atoi(argv[1]); }

    auto zi = list_zi[icase];

    std::cout << " -------------------------------- " << std::endl;
    std::cout << " Set initial position : " << zi << " m" << std::endl;

    auto position = chrono::ChVector<>(0., 0., zi) + sphere->GetRelCOG();

    sphere->SetPos(position);

    heave.clear();
    vtime.clear();

    time = -dt;

    vtime.push_back(time);
    heave.push_back(position[2]);

    while (time < 40.) {

        time += dt;

        system.DoStepDynamics(dt);
        //system.DoFrameDynamics(time);

        position = sphere->GetPosition() - sphere->GetRelCOG();

        heave.push_back(position[2]);
        vtime.push_back(time);

    }

    ValidationResults(vtime, heave, list_case[icase]);


    return 0;
}