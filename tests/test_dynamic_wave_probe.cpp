//
// Created by camille on 01/06/18.
//

#include <matplotlibcpp.h>
#include "frydom/frydom.h"

#include "frydom/core/FrNode.h"
#include "frydom/environment/waves/FrWaveProbe.h"
#include "frydom/core/FrSpringDampingForce.h"
#include "frydom/core/FrNodeDynamic.h"

using namespace frydom;

void PlotResults(const std::vector<double>& vtime, const std::vector<double>& vx,
                 const std::vector<double>& vy, const std::vector<double>& vz,
                 const std::string label="velocity", const std::string line="-",
                 const bool show=false) {

    matplotlibcpp::named_plot(label+".x", vtime, vx, line);
    matplotlibcpp::named_plot(label+".y", vtime, vy, line);
    matplotlibcpp::named_plot(label+".z", vtime, vz, line);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel(label+" (SI)");
    matplotlibcpp::grid;
    matplotlibcpp::legend();
    if (show) { matplotlibcpp::show(); }
}

void PlotResults(const std::vector<double>& vtime, const std::vector<chrono::ChVector<>>& vdata,
                const std::string label="position", const std::string line="-", const bool show=false) {

    std::vector<double> vx, vy, vz;
    for (auto& data : vdata) {
        vx.push_back(data.x());
        vy.push_back(data.y());
        vz.push_back(data.z());
    }

    PlotResults(vtime, vx, vy, vz, label, line, show);

}

class FrTestForce : public FrForce {

public:

    void SetForce(const chrono::ChVector<> vect) { force = vect; }
    void SetMoment(const chrono::ChVector<> vect) { moment = vect; }

    void UpdateState() override {

        force.x() = 1e10;
        force.y() = 0.;
        force.z() = 0.;

        moment.x() = 0.;
        moment.y() = 0.;
        moment.z() = 0.;
    }

};

std::shared_ptr<FrShip> Platform(FrOffshoreSystem* system) {

    auto platform = std::make_shared<FrShip>();
    platform->SetName("Deepsea_Stavanger");
    platform->SetHydroMesh("GVA7500.obj", true);
    platform->SetLpp(108.8);
    platform->SetMass(3.22114e7);
    platform->SetCOG(chrono::ChVector<double>(0., 0., 8.65));
    platform->SetInertiaXX(chrono::ChVector<double>(5e7, 5.58e10, 1e8)); // TODO : verifier les valeurs inertielles
    platform->SetPos(
            chrono::ChVector<double>(0., 0., 8.65)); // FIXME : il s'agit ici de la position du centre de gravité

    system->AddBody(platform);

    // Hydrostatic load

    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 1e7;
    double K44 = 1e9;
    hstStiffness->SetDiagonal(K33, K44, K44);
    platform->AddForce(hstForce);

    // Hydrodynamic load
    system->SetHydroDB("DeepSeaStavanger.hdb5");
    auto HydroMapIndex = system->GetHydroMapNb() - 1;
    system->GetHydroMapper(HydroMapIndex)->Map(platform, 0);

    auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(HydroMapIndex), system);
    radModel->SetHydroMapIndex(HydroMapIndex); // TODO : patch hydro map multibody
    radModel->AddRadiationForceToHydroBody(platform);

    // Wind load
    auto wind_force = std::make_shared<FrWindForce>("PolarWindCoeffs_NC.yml");
    platform->AddForce(wind_force);

    // Current load
    auto current_force = std::make_shared<FrCurrentForce>("PolarCurrentCoeffs_NC.yml");
    platform->AddForce(current_force);

    // Additional damping

    //auto lin_damping_force = std::make_shared<FrLinearDamping>();
    //lin_damping_force->SetManeuveuringDampings(1e7, 1e7, 1e8);
    //platform->AddForce(lin_damping_force);

    //auto lin_damping_force = std::make_shared<FrLinearDamping>();
    //lin_damping_force->SetSeakeepingDampings(1e7, 1e7, 1e8);
    //platform->AddForce(lin_damping_force);

    // Wave Probe
    auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    auto waveProbe = waveField->NewWaveProbe(0., 0.);
    waveProbe->Initialize();

    // Wave Drift Force
    auto DriftForce = std::make_shared<FrWaveDriftForce>("DeepSea_WaveDriftCoeff.h5");
    platform->AddForce(DriftForce);
    DriftForce->SetBody(platform);
    DriftForce->SetWaveProbe(waveProbe);
    DriftForce->SetCmplxElevation();

    // Wave Excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    platform->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->SetHydroMapIndex(HydroMapIndex);


    return platform;
};


std::shared_ptr<FrShip> Ship(FrOffshoreSystem* system) {

    auto ship_pos = chrono::ChVector<double>(0., 0., 0.);

    auto ship = std::make_shared<FrShip>();
    ship->SetName("Ship");
    ship->SetHydroMesh("Ship.obj", true);
    ship->SetLpp(76.20);
    ship->SetMass(8.531e6);
    ship->SetCOG(chrono::ChVector<double>(0., 0., 0.));
    ship->SetInertiaXX(chrono::ChVector<double>(1.02e8, 2.8e9, 2.8e9));
    ship->SetPos(ship_pos);

    system->AddBody(ship);

    // Hydrostatic load
    auto hstForce = std::make_shared<FrLinearHydrostaticForce>();
    auto hstStiffness = hstForce->GetStiffnessMatrix();
    double K33 = 1.61e7;
    double K44 = 2.44e8;
    double K55 = 7.42e9;
    hstStiffness->SetDiagonal(K33, K44, K55);
    ship->AddForce(hstForce);

    // Hydrodynamic load
    system->SetHydroDB("Ship.h5");
    auto HydroMapIndex = system->GetHydroMapNb()-1;
    system->GetHydroMapper(HydroMapIndex)->Map(ship, 0);

    //auto radModel = std::make_shared<FrRadiationConvolutionModel>(system->GetHydroDB(HydroMapIndex), system);
    //radModel->SetHydroMapIndex(HydroMapIndex); // TODO : patch hydro map multibody
    //radModel->AddRadiationForceToHydroBody(ship);

    // Wind load
    //auto wind_force = std::make_shared<FrWindForce>("Ship_PolarWindCoeffs.yml");
    //ship->AddForce(wind_force);

    // Current load
    auto current_force = std::make_shared<FrCurrentForce>("Ship_PolarCurrentCoeffs.yml");
    ship->AddForce(current_force);

    // Wave Probe
    auto waveField = system->GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    auto waveProbe = waveField->NewWaveProbe(ship_pos.x(), ship_pos.y());
    waveProbe->Initialize();

    // Wave Drift Force
    //auto DriftForce = std::make_shared<FrWaveDriftForce>("Ship_WaveDriftCoeff.h5");
    //ship->AddForce(DriftForce);
    //DriftForce->SetBody(ship);
    //DriftForce->SetWaveProbe(waveProbe);
    //DriftForce->SetCmplxElevation();

    // Wave Excitation force
    auto excForce = std::make_shared<FrLinearExcitationForce>();
    ship->AddForce(excForce);
    excForce->SetWaveProbe(waveProbe);
    excForce->SetHydroMapIndex(HydroMapIndex);

    // Additional damping

    auto lin_damping_force = std::make_shared<FrLinearDamping>();
    lin_damping_force->SetManeuveuringDampings(1e7, 1e7, 1e8);
    ship->AddForce(lin_damping_force);

    //auto lin_damping_force = std::make_shared<FrLinearDamping>();
    //lin_damping_force->SetSeakeepingDampings(1e7, 1e7, 1e8);
    //ship->AddForce(lin_damping_force);

    //auto test_force = std::make_shared<FrTestForce>();
    //ship->AddForce(test_force);

    return ship;
}

int main(int argc, char* argv[]) {

    // --------------------------------------------------------
    // System
    // --------------------------------------------------------
    FrOffshoreSystem system;

    // --------------------------------------------------------
    // Environment
    // --------------------------------------------------------
    system.GetEnvironment()->SetWind(FrWind::UNIFORM);
    system.GetEnvironment()->GetWind()->Set(NORTH, 0., KNOT, NED, COMEFROM);

    system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
    system.GetEnvironment()->GetCurrent()->Set(EAST, 0., KNOT, NED, COMEFROM);

    system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_IRREGULAR);
    auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    waveField->SetWavePulsations(0.5, 2., 80, RADS);
    waveField->GetWaveSpectrum()->SetHs(5.);
    waveField->GetWaveSpectrum()->SetTp(10.);
    waveField->GetSteadyElevation(0, 0);

    // ----------------------------------------------------------
    // Body
    // ----------------------------------------------------------
    auto ship = Platform(&system);

    ship->SetPos_dt(chrono::ChVector<double>(5., 0., 0.));

    // ----------------------------------------------------------
    // Dynamic node
    // ----------------------------------------------------------

    auto body_sensor = std::make_shared<FrHydroBody>();
    system.Add(body_sensor);
    body_sensor->SetMass(1.);
    body_sensor->Set3DOF_ON();
    auto force = std::make_shared<FrSpringDampingForce>(ship.get(), 60, 0.5);
    body_sensor->SetPos_dt(ship->GetPos_dt());
    body_sensor->AddForce(force);


    // ----------------------------------------------------------
    // Wave probe
    // ----------------------------------------------------------

    auto waveProbe = std::make_shared<FrLinearWaveProbe>();
    waveProbe->AttachedNode(body_sensor);

    // ----------------------------------------------------------
    // Simulation
    // ----------------------------------------------------------

    std::vector<chrono::ChVector<double>> position_sensor, position_body;
    std::vector<chrono::ChVector<double>> velocity_sensor, velocity_body;
    std::vector<double> vtime;

    double time = 0.;
    double dt = 0.01;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    while (time < 80.) {

        system.DoStepDynamics(dt);
        time += dt;

        vtime.push_back(time);
        position_sensor.push_back(body_sensor->GetPosition());
        position_body.push_back(ship->GetPosition());
        velocity_sensor.push_back(body_sensor->GetVelocity());
        velocity_body.push_back(ship->GetVelocity());
    }

    matplotlibcpp::subplot(2,1,1);
    PlotResults(vtime, position_sensor, "sensor", "--");
    PlotResults(vtime, position_body, "body");

    matplotlibcpp::subplot(2,1,2);
    PlotResults(vtime, velocity_sensor, "sensor", "--");
    PlotResults(vtime, velocity_body, "body");

    matplotlibcpp::show();

}