//
// Created by camille on 21/12/17.
//

#include <frydom/frydom.h>
#include <frydom/hydrodynamics/FrWaveDriftForce.h>

using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {

    // Offshore System
    FrOffshoreSystem system;

    // Wave Field Condition
    auto freeSurface = system.GetFreeSurface();
    freeSurface->Initialize(-200, 200, 2, -200, 200, 2);

    freeSurface->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = freeSurface->GetLinearWaveField();
    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    double wmin = 0.1;
    double wmax = 3.;
    unsigned int nbFreq = 80;
    waveField->SetWavePulsations(wmin, wmax, nbFreq, RADS);
    waveField->GetWaveSpectrum()->SetHs(5.);
    waveField->GetWaveSpectrum()->SetTp(15.);

    //waveField->GetWaveRamp()->SetDuration(20.);
    //waveField->GetWaveRamp()->SetIncrease();

    freeSurface->UpdateAssetON();

    // Get a waveProbe for wave drift force on body
    auto waveProbe = waveField->NewWaveProbe(0, 0);

    // New ship model
    auto platforme = std::make_shared<FrHydroBody>();
    platforme->SetName("Deepsea_Stanvenger");
    platforme->SetHydroMesh("GVA7500_geom_full2.obj", true);
    platforme->SetLpp(116.6);
    platforme->SetMass(3.22114e7);
    platforme->SetCOG(chrono::ChVector<double>(0., 0., 8.65));
    platforme->SetInertiaXX(chrono::ChVector<double>(5e7, 5e7, 1e8));

    auto p_ship = ChVector<>(0.,0.,0.);
    auto q_ship = euler_to_quat(90., 0., 0., CARDAN, DEG);

    platforme->SetPos(p_ship);
    platforme->SetRot(q_ship);

    system.AddBody(platforme);

    // Hydrostatics
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 1e10;
    double K44 = 0.;
    hstStiffness->SetDiagonal(K33, K44, K44);
    platforme->AddForce(hstForce);

    // Create a wave drift force model and add to the ship model
    std::string hdf5_file = "WaveDriftCoeff_100.h5";

    auto DriftForce = std::make_shared<FrWaveDriftForce>(hdf5_file);
    std::cout << "A new wave drift force model has been defined"<< std::endl;

    platforme->AddForce(DriftForce);
    std::cout << "Force has been add to the ship model" << std::endl;

    DriftForce->SetBody(platforme);

    DriftForce->SetWaveProbe(waveProbe);
    std::cout << "Define a new wave probe" << std::endl;

    DriftForce->SetCmplxElevation();
    std::cout << "Define the complex amplitude" << std::endl;

    // Running simulation

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto app = FrIrrApp(system, 30);
    app.AddTypicalLights();
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(0, 1, -1));

    //app.SetVideoframeSaveInterval(1);
    //app.SetVideoframeSave(true);

    app.Run();  // TODO: remettre
}