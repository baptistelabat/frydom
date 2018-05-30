//
// Created by camille on 29/05/18.
//

#include <matplotlibcpp.h>

#include "frydom/frydom.h"

using namespace frydom;

void PlotResults(const std::vector<double>& vtime, const std::vector<double>& x,
                 const std::vector<double>& y, const std::vector<double>& z,
                 const std::string label)
{
    matplotlibcpp::named_plot(label+".x", vtime, x);
    matplotlibcpp::named_plot(label+".y", vtime, y);
    matplotlibcpp::named_plot(label+".z", vtime, z);
    matplotlibcpp::legend();
    matplotlibcpp::grid(true);
}


int main(int argc, char* argv[]) {

    // -------------------------------------------------
    // System
    // -------------------------------------------------

    FrOffshoreSystem system;

    // -------------------------------------------------
    // Body to link
    // -------------------------------------------------

    auto myBody = std::make_shared<FrBody>();
    myBody->SetPos(chrono::ChVector<>(0, 0, 100));
    myBody->SetBodyFixed(true);
    auto node1 = myBody->CreateNode();

    system.AddBody(myBody);

    auto myBox = std::make_shared<FrBody>();
    myBox->SetMass(1000);
    auto node2 = myBox->CreateNode();

    system.AddBody(myBox);

    // --------------------------------------------------
    // Catenary line
    // --------------------------------------------------
    double Lu = 100.;
    auto u = chrono::ChVector<double>(0, 0, -1);
    double q = 616.538;
    double EA = 1.5708e9;
    double A = 0.05;
    double E = EA/A;

    auto line = std::make_shared<FrCatenaryLine>(node1, node2, true, E, A, Lu, q, u);

    //system.Add(line);

    // -----------------------------------------------------
    // Simulation
    // -----------------------------------------------------

    double dt = 0.001;

    system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);
    system.SetStep(dt);
    system.Initialize();

    double time = 0.;
    std::vector<double> vtime;
    chrono::ChVector<double> f1, f2;
    std::vector<double> x1, y1, z1;
    std::vector<double> x2, y2, z2;
    std::vector<double> f1x, f1y, f1z;
    std::vector<double> f2x, f2y, f2z;

    while (time < 100.) {

        if (std::abs(time - 20.) < 0.5*dt) {
            line->SetUnrollingSpeed(0.1);
        }

        if (std::abs(time - 60.) < 0.5*dt) {
            line->SetUnrollingSpeed(0.);
        }

        if (std::abs(time - 80.) < 0.5*dt) {
            line->SetUnrollingSpeed(-0.1);
        }

        system.DoStepDynamics(dt);

        //line->Update(time);
        //line->solve();

        time += dt;

        vtime.push_back(time);

        x1.push_back(node1->GetAbsPos().x());
        y1.push_back(node1->GetAbsPos().y());
        z1.push_back(node1->GetAbsPos().z());

        x2.push_back(node2->GetAbsPos().x());
        y2.push_back(node2->GetAbsPos().y());
        z2.push_back(node2->GetAbsPos().z());

        f1 = line->GetTension(0.);
        f1x.push_back(f1.x());
        f1y.push_back(f1.y());
        f1z.push_back(f1.z());

        f2 = line->GetEndingNodeTension();
        f2x.push_back(f2.x());
        f2y.push_back(f2.y());
        f2z.push_back(f2.z());

    }

    //line.solve();
    //myBody.UpdateForces(false); // A quoi ca sert ??? (en plus ca semble pas etre un bool en param...)

    matplotlibcpp::subplot(2,2,1);
    PlotResults(vtime, x1, y1, z1, "Position_1");

    matplotlibcpp::subplot(2,2,2);
    PlotResults(vtime, x2, y2, z2, "Position_2");

    matplotlibcpp::subplot(2,2,3);
    PlotResults(vtime, f1x, f1y, f1z, "Force_1");

    matplotlibcpp::subplot(2,2,4);
    PlotResults(vtime, f2x, f2y, f2z, "Force_2");

    matplotlibcpp::show();

    // -----------------------------------------------------
    // End
    // -----------------------------------------------------

    return 0;
}