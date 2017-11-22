//
// Created by frongere on 29/06/17.
//


// TODO: faire un header qui permet d'importer tout core (FrCore.h)
//#include "frydom/core/FrShip.h"
//#include "frydom/core/FrOffshoreSystem.h"
//#include "frydom/environment/waves/FrFlatFreeSurface.h"
//#include "frydom/core/FrEulerAngles.h"
//#include "frydom/utils/FrIrrApp.h"

#include "frydom/frydom.h"


using namespace chrono;
using namespace frydom;

int main(int argc, char* argv[]) {

    bool both = true;

    double x = 0.;
    double y = 0.;
    double z = 0.;


    double phi = 0.;
    double theta = -90.;
    double psi = 0.;



    // SYSTEM AND FREE SURFACE
    auto system = frydom::FrOffshoreSystem();
//    auto fs = std::make_unique<FrFlatFreeSurface>(0.);
//    fs->Initialize(-50, 50, 50);
//    system.setFreeSurface(fs.release());
    system.GetEnvironment()->GetFreeSurface()->Initialize(-50, 50, 50);

    // SHIP WITH CHRONO TOOLS FOR ORIENTATION
    auto ship1 = std::make_shared<FrShip>();
    system.AddBody(ship1);
    ship1->SetBodyFixed(true);
    ship1->SetHydroMesh("../data/ship/MagneViking.obj", true);

    auto position = ChVector<>(x, y, z);
    auto quat1 = euler_to_quat(phi, theta, psi, CARDAN, DEG);

    // TESTS
    auto mat1 = quat_to_mat(quat1);
    auto vect = ChVector<>(1, 2, 3);

    auto quat_x_vect = quat1.Rotate(vect);
    auto mat_x_vect = mat1 * vect;
    auto quat_x_vect_back = quat1.RotateBack(vect);

    auto vect2 = quat1.Rotate(quat_x_vect_back);

    ship1->SetPos(position);
    ship1->SetRot(quat1);

//    double a = modulo360(370);  // test

    // SHIP WITH CHRONO TOOLS FOR ORIENTATION
    auto ship2 = std::make_shared<FrShip>();
    system.AddBody(ship2);
    ship2->SetBodyFixed(true);
    ship2->SetHydroMesh("../data/ship/MagneViking.obj", true);


    ChQuaternion<> quat2;
    if (both) {
        // Calcul de axis angle from quat1
        ChVector<> axis;
        double angle;
        quat_to_axis_angle(quat1, axis, angle, RAD);

        // Generating nexw quaternion from axis angle
        quat2 = Q_from_AngAxis(angle, axis);
    } else {
        quat2 = QUNIT;
    }

    ship2->SetPos(position);
    ship2->SetRot(quat2);

    // Verifs
    auto quat11 = ship1->GetRot();
    auto quat22 = ship2->GetRot();

    auto quat__ = euler_to_quat(quat_to_euler(quat22, CARDAN, DEG), CARDAN, DEG);

    // Essais axis_angle



    // VISU
    FrIrrApp app(&system, L"Viz");
    app.AddTypicalLights();
    app.AddTypicalCamera(irr::core::vector3df(0, 0, 300), irr::core::vector3df(1, 0, -1));


    app.AssetBindAll();
    app.AssetUpdateAll();

    while (app.GetDevice()->run()) {
        app.BeginScene();
        app.DrawAll();

        system.Update(true);

        app.EndScene();

    }

}