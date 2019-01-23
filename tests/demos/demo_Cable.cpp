//
// Created by lucas on 23/01/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    /** This demo presents basic features of FRyDoM, with EasyBody (box and spheres). The makeIt* functions set-up the
     * collision box, corresponding asset and inertia tensor. We simulate the free fall of several balls with
     * an initial velocity on a floor, with collisions between the balls.
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem_ system;

    // Hide the free surface and seabed visual assets.
    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);
    system.GetEnvironment()->GetOcean()->ShowSeabed(false);

    enum cableCase{ OneLine, TwoLines, Newton_Pendulum};

    cableCase Case = Newton_Pendulum;

    switch (Case) {
        case OneLine: {
            auto Node1 = system.GetWorldBody()->NewNode(Position(-10., 0., 0.), NWU);
            auto Node2 = system.GetWorldBody()->NewNode(Position(10., 0., 0.), NWU);

            // Line properties
            bool elastic = true;
            auto u = mathutils::Vector3d<double>(0., 0., -1.);
            double Lu = 40.;
            double q = 616.538;
            double EA = 1.5708e9;
            double A = 0.05;
            double E = EA / A;

            auto CatenaryLine = std::make_shared<FrCatenaryLine_>(Node1, Node2, elastic, E, A, Lu, q, u);

            system.Add(CatenaryLine);
            break;
        }

        case TwoLines: {
            auto sphere = system.NewBody();
            sphere->SetName("Sphere");
            makeItSphere(sphere, 1, 1000);

            sphere->SetPosition(Position(0., 0., 0.), NWU);
            sphere->SetVelocityInWorldNoRotation(Velocity(0., 20., 0.), NWU);
            //        sphere->SetBodyFixed(true);

            auto sphereNode = sphere->NewNode(Position(0., 0., 0.), NWU);
            auto worldNode = system.GetWorldBody()->NewNode(Position(0., 0., 50.), NWU);

            // Line properties
            bool elastic = true;
            auto u = mathutils::Vector3d<double>(0., 0., -1.);
            double unstretchedLength = 50;
            double linearDensity = 616.538;
            double EA = 1.5708e8;
            double sectionArea = 0.05;
            double YoungModulus = EA / sectionArea;

            auto CatenaryLine = std::make_shared<FrCatenaryLine_>(sphereNode, worldNode, elastic, YoungModulus,
                                                                  sectionArea, unstretchedLength, linearDensity, u);

            system.Add(CatenaryLine);
            break;
        }

        case Newton_Pendulum: {
            int n_sphere = 4;
            double radius = 2;
            double density = 1000;

            // Line properties
            bool elastic = true;
            auto u = mathutils::Vector3d<double>(0., 0., -1.);
            double unstretchedLength = 51;
            double linearDensity = 61.6538;
            double EA = 1.5708e8;
            double sectionArea = 0.05;
            double YoungModulus = EA / sectionArea;

            float steelYoungModulus = 1e12;
            float steelNormalDamping = 1e12;

            for (int ib = 0; ib < n_sphere; ++ib) {
                auto sphere = system.NewBody();
                // Give it a name
                std::stringstream concatenation;
                concatenation << "Sphere" << ib;
                sphere->SetName(concatenation.str().c_str());
                auto sphere_material_props = sphere->GetMaterialSurface();
                sphere_material_props->SetKn(steelYoungModulus);
                sphere_material_props->SetGn(steelNormalDamping);
                sphere_material_props->young_modulus = steelYoungModulus;
                sphere_material_props->restitution = 0;

                makeItSphere(sphere, radius, density);

                sphere->SetPosition(Position(0., 4. * ib, 0.), NWU);
                if (ib == 0) {
                    sphere->SetPosition(Position(0., -30., 10.), NWU);
//                    sphere->SetVelocityInWorldNoRotation(Velocity(0., 50., 0.), NWU);
                }

                auto sphereNode = sphere->NewNode(Position(0., 0., 0.), NWU);

                auto worldNode1 = system.GetWorldBody()->NewNode(Position(10., 4. * ib, 50.), NWU);
                auto worldNode2 = system.GetWorldBody()->NewNode(Position(-10., 4. * ib, 50.), NWU);


                auto CatenaryLine1 = std::make_shared<FrCatenaryLine_>(sphereNode, worldNode1, elastic, YoungModulus,
                                                                       sectionArea, unstretchedLength, linearDensity,
                                                                       u);
                auto CatenaryLine2 = std::make_shared<FrCatenaryLine_>(sphereNode, worldNode2, elastic, YoungModulus,
                                                                       sectionArea, unstretchedLength, linearDensity,
                                                                       u);

                system.Add(CatenaryLine1);
                system.Add(CatenaryLine2);
            }
            break;
        }
    }

    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

//    system.SetSolver(FrOffshoreSystem_::SOLVER::MINRES);
//    system.SetSolverWarmStarting(true);
//    system.SetSolverMaxIterSpeed(1000);
//    system.SetSolverMaxIterStab(200);
//    system.SetSolverForceTolerance(1e-13);

//    system.SetTimeStepper(FrOffshoreSystem_::TIME_STEPPER::EULER_IMPLICIT);
//    system.SetTimeStepper(FrOffshoreSystem_::TIME_STEPPER::EULER_IMPLICIT_LINEARIZED);

//    auto test = system.Get

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(30, 50, false);
}
