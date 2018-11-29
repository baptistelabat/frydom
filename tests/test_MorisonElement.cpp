//
// Created by camille on 28/11/18.
//

#include "frydom/frydom.h"
#include "matplotlibcpp.h"

using namespace frydom;

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

    // -----------------------------------------------------
    // System
    // -----------------------------------------------------

    FrOffshoreSystem_ system;

    // -----------------------------------------------------
    // Environment
    // -----------------------------------------------------

    auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField();
    //auto waveField = system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetLinearWaveField();

    waveField->SetWaveHeight(1.);
    waveField->SetWavePeriod(10.);
    waveField->SetDirection(NORTH(NWU), NWU, GOTO);
    waveField->GetWaveRamp()->Deactivate();

    system.GetEnvironment()->GetAtmosphere()->GetWind()->MakeFieldUniform();
    system.GetEnvironment()->GetAtmosphere()->GetWind()->GetFieldUniform()->Set(NORTH, 0., KNOT, GOTO);

    // ------------------------------------------------------
    // Morison elements
    // ------------------------------------------------------

    auto structure = system.NewBody();
    structure->SetPosition(Position(0., 0., -2), NWU);
    //structure->SetDOF(false, false, false, false, false, false);      // TODO : a reintégrer dans FrBody, pourquoi cela n'y est plus ???

    auto morisonModel = std::make_shared<FrMorisonSingleElement_>(structure.get());
    auto morisonForce = std::make_shared<FrMorisonForce_>(morisonModel);

    morisonModel->SetNodes(structure.get(), Position(0., -2, 0.), Position(0., 2., 0));
    morisonModel->SetDragCoeff(0.1);
    morisonModel->SetDiameter(0.1);

    system.AddPhysicsItem(morisonModel);
    structure->AddExternalForce(morisonForce);

    // ---------------------------------------------------------
    // Simulation
    // ---------------------------------------------------------

    double time = 0;
    double dt = 0.01;

    system.Initialize();
    system.SetTimeStep(dt);

    Force force;

    while (time < 50.) {
        time += dt;
        system.AdvanceTo(time);

        force = morisonForce->GetForceInWorld(NWU);

        std::cout << force.GetFx() << ";" << force.GetFy() << ";" << force.GetFz() << std::endl;
    }

}