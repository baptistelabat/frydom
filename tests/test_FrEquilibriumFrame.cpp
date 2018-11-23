//
// Created by camille on 22/11/18.
//

#include "frydom/frydom.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"
#include "frydom/core/FrVector.h"

#include "matplotlibcpp.h"

using namespace frydom;

template <class Vector>
void GetVector(std::vector<double>& x, std::vector<double>& y, std::vector<double>& z,
               const std::vector<Vector>& fval) {
    for (auto val: fval) {
        x.push_back(val.at(0));
        y.push_back(val.at(1));
        z.push_back(val.at(2));
    }
}


inline Velocity HorSpeedFunction(const double& time) {
    Velocity result;
    result = Velocity(0., 2., 0.);
    result += Velocity(0.1 * sin(M_PI / 5. * time), 0.2 * sin(M_PI / 4. * time), 0);
    result += Velocity(0.5 * cos(M_PI / 1. * (time - 2.)), 0.8 * cos(M_PI / 5. * (time - 5.) ), 0);
    result += Velocity(1. * sin(M_PI / 10. * (time - 10.)), 1. * sin(M_PI / 10. * (time - 50.) ), 0);
    result += Velocity(0.8 * cos(M_PI / 20. * (time - 45)), 0.5 * cos(M_PI / 6. * (time - 2.) ), 0);
    return result;
}

int main(int argc, char* argv[]) {


    FrOffshoreSystem_ system;

    auto body = system.NewBody();

    body->RemoveGravity(true); // Permet de supprimer les mouvements verticaux

    //auto eqFrame = std::make_shared<FrEqFrameMeanMotion_>(body.get(), 60., 0.1);
    //eqFrame->SetPositionCorrection(60., 0.1, 0.01/60., 0.01/60.);
    //system.AddPhysicsItem(eqFrame);

    auto eqFrame = std::make_shared<FrEqFrameSpringDamping_>(body.get(), 120., 0.5);
    system.AddPhysicsItem(eqFrame);

    eqFrame->InitSpeedFromBody(true);

    double time = 0.;
    double dt = 0.01;
    double tmax = 400.;

    auto HorSpeed = Velocity(0., 2., 0.);
    auto RotSpeed = AngularVelocity(0., 0., M_PI_2 / 40.);
    auto NoRotSpeed = AngularVelocity(0., 0., 0.);

    int nt = int(tmax/dt);

    system.SetTimeStep(dt);

    std::vector<double> vtime;
    std::vector<Position> XBody, XFrame;
    std::vector<Velocity> VBody, VFrame;
    std::vector<double> WBody, WFrame;
    std::vector<double> RBody, RFrame;

    double temp1, temp2;
    double rb, rf;

    body->SetGeneralizedVelocityInBody(HorSpeed, NoRotSpeed, NWU);

    system.Initialize();

    for (int it=0; it<nt; ++it) {


        if (time > 80. and time < 120.) {
            body->SetGeneralizedVelocityInBody(HorSpeedFunction(time), RotSpeed, NWU);
        } else {
            body->SetGeneralizedVelocityInBody(HorSpeedFunction(time), NoRotSpeed, NWU);
        }

        system.AdvanceTo(time);
        time = it * dt;

        body->GetRotation().GetCardanAngles_DEGREES(temp1, temp2, rb, NWU);
        eqFrame->GetRotation().GetCardanAngles_DEGREES(temp1, temp2, rf, NWU);

        vtime.push_back(time);
        XBody.push_back(body->GetPosition(NWU));
        XFrame.push_back(eqFrame->GetPosition(NWU));
        VBody.push_back(body->GetVelocityInWorld(NWU));
        VFrame.push_back(eqFrame->GetVelocityInWorld(NWU));
        RBody.push_back(rb);
        RFrame.push_back(rf);
        WBody.push_back(body->GetAngularVelocityInWorld(NWU).GetWz());
        WFrame.push_back(eqFrame->GetAngularVelocityAroundZ(NWU));
    }


    // ------------------ OUTPUT PLOT -----------------------------

    std::vector<double> xb, yb, zb;
    std::vector<double> xf, yf, zf;
    std::vector<double> vxb, vyb, vzb;
    std::vector<double> vxf, vyf, vzf;

    GetVector<Position>(xb, yb, zb, XBody);
    GetVector<Position>(xf, yf, zf, XFrame);
    GetVector<Velocity>(vxb, vyb, vzb, VBody);
    GetVector<Velocity>(vxf, vyf, vzf, VFrame);

    matplotlibcpp::figure();
    matplotlibcpp::named_plot("body", xb, yb);
    matplotlibcpp::named_plot("frame", xf, yf);
    matplotlibcpp::xlabel("x-coordinates");
    matplotlibcpp::ylabel("y-coordinates");
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::named_plot("xb", vtime, xb);
    matplotlibcpp::named_plot("yb", vtime, yb);
    matplotlibcpp::named_plot("xf", vtime, xf);
    matplotlibcpp::named_plot("yf", vtime, yf);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel("Position (m)");
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::named_plot("vxb", vtime, vxb);
    matplotlibcpp::named_plot("vyb", vtime, vyb);
    matplotlibcpp::named_plot("vxf", vtime, vxf);
    matplotlibcpp::named_plot("vyf", vtime, vyf);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel("Velocity (m/s)");
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::named_plot("body", vtime, WBody);
    matplotlibcpp::named_plot("frame", vtime, WFrame);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel("Angular velocity (rad/s)");
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::named_plot("body", vtime, RBody);
    matplotlibcpp::named_plot("frame", vtime, RFrame);
    matplotlibcpp::xlabel("time (s)");
    matplotlibcpp::ylabel("Angular position (deg)");
    matplotlibcpp::legend();

    matplotlibcpp::show();


}