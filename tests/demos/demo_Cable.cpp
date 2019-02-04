//
// Created by lucas on 23/01/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    /** This demo presents cable features, either catenary line (quasi-static) alone, bound to the fixed world body, or
     * within a pendulum/Newton pendulum.
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

    // Definition of the three demo with cables
    enum cableCase{ FixedLine, Pendulum, Newton_Pendulum};

    // Chose the one you want to run
    cableCase Case = Newton_Pendulum;

    switch (Case) {
        // This case features one catenary line, with fixed ending and starting nodes. We can check the line profile and
        // stretched length.
        case FixedLine: {
            // Create the nodes from the world body (which is fixed)
            auto Node1 = system.GetWorldBody()->NewNode();
            Node1->SetPositionInBody(Position(-10., 0., 0.), NWU);
            auto Node2 = system.GetWorldBody()->NewNode();
            Node2->SetPositionInBody(Position(10., 0., 0.), NWU);

            // Line properties :
            bool elastic = true;                    // non elastic catenary lines are only available for non stretched lines
            auto u = Direction(0., 0., -1.);        // definition of the direction of the uniform load distribution
            double unstretchedLength = 40.;         // unstretched length
            double linearDensity = 616.538;         // linear density of the line
            double EA = 1.5708e9;                   //
            double sectionArea = 0.05;              // section area
            double YoungModulus = EA / sectionArea; // Young modulus of the line

            // Create the catenary line, using the nodes and line properties previously defined
            auto CatenaryLine = std::make_shared<FrCatenaryLine_>(Node1, Node2, elastic, YoungModulus, sectionArea, unstretchedLength, linearDensity, u, fc);

            // Don't forgot to add the line to the system !
            system.Add(CatenaryLine);
            break;
        }
        // This case features a pendulum : a sphere balancing at the end of a catenary line, with its other end fixed.
        case Pendulum: {
            // Create the moving sphere, from the system
            auto sphere = system.NewBody();
            // Give it a name
            sphere->SetName("Sphere");
            // make it a sphere : gives an asset, a collision box, and its inertia parameters
            makeItSphere(sphere, 1, 1000);

            // Set its initial position and velocity
            sphere->SetPosition(Position(0., 0., 0.), NWU);
            sphere->SetVelocityInWorldNoRotation(Velocity(0., 20., 0.), NWU);
            //        sphere->SetFixedInWorld(true);

            // create the nodes from the sphere and the world body.
            auto sphereNode = sphere->NewNode();
            auto worldNode = system.GetWorldBody()->NewNode();
            worldNode->SetPositionInBody(Position(0., 0., 50.), NWU);

            // Line properties :
            bool elastic = true;                    // non elastic catenary lines are only available for non stretched lines
            auto u = Direction(0., 0., -1.);        // definition of the direction of the uniform load distribution
            double unstretchedLength = 50.;         // unstretched length
            double linearDensity = 616.538;         // linear density of the line
            double EA = 1.5708e9;                   //
            double sectionArea = 0.05;              // section area
            double YoungModulus = EA / sectionArea; // Young modulus of the line

            // Create the catenary line, using the nodes and line properties previously defined
            auto CatenaryLine = std::make_shared<FrCatenaryLine_>(sphereNode, worldNode, elastic, YoungModulus,
                                                                  sectionArea, unstretchedLength, linearDensity, u, fc);

            // Don't forgot to add the line to the system !
            system.Add(CatenaryLine);
            break;
        }
        // This case features a Newton pendulum, consisting of a series of identically sized metal balls suspended in a
        // metal frame so that they are just touching each other at rest. Each ball is attached to the frame by two lines
        // of equal length angled away from each other. This restricts the pendulums' movements to the same plane.
        case Newton_Pendulum: {
            // spheres properties
            int n_sphere = 5;               // number of spheres wanted
            double diameter = 4;            // diameter of the spheres
            double density = 1000;          // density of the spheres
            float steelYoungModulus = 1e12; // Young modulus of the sphere (for contact)
            float steelNormalDamping = 1e12;// Normal damping of the sphere (for contact)

            // Line properties :
            bool elastic = true;                    // non elastic catenary lines are only available for non stretched lines
            auto u = Direction(0., 0., -1.);        // definition of the direction of the uniform load distribution
            double unstretchedLength = 51.;         // unstretched length
            double linearDensity = 61.6538;         // linear density of the line
            double EA = 1.5708e8;                   //
            double sectionArea = 0.05;              // section area
            double YoungModulus = EA / sectionArea; // Young modulus of the line

            // Loop to create the spheres
            for (int ib = 0; ib < n_sphere; ++ib) {
                // Create the sphere from the system
                auto sphere = system.NewBody();

                // Give it a name
                std::stringstream concatenation;
                concatenation << "Sphere" << ib;
                sphere->SetName(concatenation.str().c_str());

                // Set the material properties
                auto sphere_material_props = sphere->GetMaterialSurface();
                sphere_material_props->SetKn(steelYoungModulus);
                sphere_material_props->SetGn(steelNormalDamping);
                sphere_material_props->young_modulus = steelYoungModulus;
                sphere_material_props->restitution = 0;

                // Make it a sphere : gives an asset, a collision box, and its inertia parameters
                makeItSphere(sphere, 0.5*diameter, density);

                // Set the initial position
                sphere->SetPosition(Position(0., diameter * ib, 0.), NWU);
                if (ib == 0) {
                    sphere->SetPosition(Position(0., -30., 10.), NWU);
                }
//                sphere->SetVelocityInWorldNoRotation(Velocity(0., 0., 0.), NWU);

                // Create the nodes
                auto sphereNode = sphere->NewNode();

                auto worldNode1 = system.GetWorldBody()->NewNode();
                worldNode1->SetPositionInBody(Position(10., diameter * ib, 50.), NWU);
                auto worldNode2 = system.GetWorldBody()->NewNode();
                worldNode2->SetPositionInBody(Position(-10., diameter * ib, 50.), NWU);

                // Create the catenary lines, using the nodes and line properties previously defined
                auto CatenaryLine1 = std::make_shared<FrCatenaryLine_>(worldNode1, sphereNode, elastic, YoungModulus,
                                                                       sectionArea, unstretchedLength, linearDensity,
                                                                       u, fc);
                auto CatenaryLine2 = std::make_shared<FrCatenaryLine_>(worldNode2, sphereNode, elastic, YoungModulus,
                                                                       sectionArea, unstretchedLength, linearDensity,
                                                                       u, fc);
                // Set the number of drawn elements on the catenary lines (the more, the slower the simulation)
                CatenaryLine1->SetNbElements(10);
                CatenaryLine2->SetNbElements(10);
                // Don't forgot to add the lines to the system !
                system.Add(CatenaryLine1);
                system.Add(CatenaryLine2);
            }
            break;
        }
    }

    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // You can modify solver and time stepper parameters :
//    system.SetSolver(FrOffshoreSystem_::SOLVER::MINRES);
//    system.SetSolverWarmStarting(true);
//    system.SetSolverMaxIterSpeed(1000);
//    system.SetSolverMaxIterStab(200);
//    system.SetSolverForceTolerance(1e-13);

//    system.SetTimeStepper(FrOffshoreSystem_::TIME_STEPPER::EULER_IMPLICIT);
//    system.SetTimeStepper(FrOffshoreSystem_::TIME_STEPPER::EULER_IMPLICIT_LINEARIZED);

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(30, 50, false);
}
