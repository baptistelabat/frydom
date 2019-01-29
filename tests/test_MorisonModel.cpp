//
// Created by camille on 17/04/18.
//


#include "frydom/frydom.h"
#include "matplotlibcpp.h"
#include "frydom/environment/ocean/freeSurface/waves/FrFlowSensor.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/hydrodynamics/morison/FrMorisonModel.h"

using namespace frydom;
using namespace chrono;

void PlotResults(const std::vector<double>& vtime, const std::vector<double>& vx,
                 const std::vector<double>& vy, const std::vector<double>& vz,
                 const std::string label="velocity",
                 const bool show=true) {

    matplotlibcpp::named_plot(label+".x", vtime, vx);
    matplotlibcpp::named_plot(label+".y", vtime, vy);
    matplotlibcpp::named_plot(label+".z", vtime, vz);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel(label+" (SI)");
    matplotlibcpp::grid;
    matplotlibcpp::legend();
    if (show) { matplotlibcpp::show(); }

}


int main(int argc, char* argv[]) {

    // ---------------------------------------------------------
    // System
    // ---------------------------------------------------------

    FrOffshoreSystem system;

    // ---------------------------------------------------------
    // Environment
    // ---------------------------------------------------------

    //system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_IRREGULAR);
    //auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    //waveField->SetMeanWaveDirection(0., DEG);  // TODO: permettre de mettre une convention GOTO/COMEFROM
    //waveField->SetWavePulsations(0.5, 2., 80, RADS);
    //waveField->GetWaveSpectrum()->SetHs(1.);
    //waveField->GetWaveSpectrum()->SetTp(10.);

    system.GetEnvironment()->GetFreeSurface()->SetLinearWaveField(LINEAR_REGULAR);
    auto waveField = system.GetEnvironment()->GetFreeSurface()->GetLinearWaveField();
    waveField->SetRegularWaveHeight(1.);
    waveField->SetRegularWavePeriod(10.);
    waveField->SetMeanWaveDirection(0., DEG);

    waveField->GetSteadyElevation(0, 0);

    waveField->GetWaveRamp()->Deactivate();

    //waveField->SetStretching(VERTICAL);

    system.GetEnvironment()->SetCurrent(FrCurrent::UNIFORM);
    system.GetEnvironment()->GetCurrent()->Set(NORTH, 0., KNOT, NED, GOTO);

    // ----------------------------------------------------------
    // Morison structure
    // ----------------------------------------------------------

    auto structure = std::make_shared<FrHydroBody>();
    structure->SetPos(chrono::ChVector<double>(0., 0., -2));

    structure->SetMass(1400.);


    auto posA = chrono::ChVector<double>(0., -2., 0.);
    auto posB = chrono::ChVector<double>(0., +2., 0.);

    auto morison = std::make_shared<FrSingleElement>(posA, posB, 0.1, 0.1, 0.01, 0.);

    morison->SetBody(structure.get(), true);

    system.AddBody(structure);

    structure->SetBodyFixed(true);
    //structure->Set3DOF_ON(chrono::ChVector<>(0, 0, 1),
    //                      chrono::ChVector<>(0, 0, 2),
    //                      chrono::ChVector<>(0, 0, 0));

    // ----------------------------------------------------------
    // Simulation
    // ----------------------------------------------------------

    double time = 0.;
    double dt = 0.01;

    std::vector<double> vx, vy, vz, vtime;
    std::vector<double> ax, ay, az;
    std::vector<double> fx, fy, fz;
    std::vector<double> mx, my, mz;
    std::vector<double> x, y, z;
    std::vector<double> rx, ry, rz;

    ChVector<double> velocity, acceleration;
    ChVector<double> force, moment;
    ChVector<double> position, rotation;
    std::vector<std::shared_ptr<ChForce>> force_list;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);

    system.Initialize();

    std::cout << "Number of force : " << structure->GetForceList().size() << std::endl;

    while (time < 50.) {

        time += dt;

        system.DoStepDynamics(dt);

        structure->UpdateForces(time);

        velocity = morison->GetFlowSensor()->GetVelocity();

        vtime.push_back(time);
        vx.push_back(velocity.x());
        vy.push_back(velocity.y());
        vz.push_back(velocity.z());

        acceleration = morison->GetFlowSensor()->GetAcceleration();
        ax.push_back(acceleration.x());
        ay.push_back(acceleration.y());
        az.push_back(acceleration.z());

        force_list = structure->GetForceList();
        force_list[0]->GetBodyForceTorque(force, moment);
        fx.push_back(force.x());
        fy.push_back(force.y());
        fz.push_back(force.z());
        mx.push_back(moment.x());
        my.push_back(moment.y());
        mz.push_back(moment.z());

        position = structure->GetPos();
        rotation = quat_to_euler(structure->GetRot());
        x.push_back(position.x());
        y.push_back(position.y());
        z.push_back(position.z());
        rx.push_back(rotation.x());
        ry.push_back(rotation.y());
        rz.push_back(rotation.z());
    }

    matplotlibcpp::subplot(3,2,1);
    PlotResults(vtime, vx, vy, vz, "velocity", false);

    matplotlibcpp::subplot(3,2,2);
    PlotResults(vtime, ax, ay, az, "acceleration", false);

    matplotlibcpp::subplot(3,2,3);
    PlotResults(vtime, fx, fy, fz, "force", false);

    matplotlibcpp::subplot(3,2,5);
    PlotResults(vtime, mx, my, mz, "moment", false);

    matplotlibcpp::subplot(3,2,4);
    PlotResults(vtime, x, y, z, "position", false);

    matplotlibcpp::subplot(3,2,6);
    PlotResults(vtime, rx, ry, rz, "rotation");

    return 0;
}
