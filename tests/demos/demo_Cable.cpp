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

using namespace frydom;

int main(int argc, char* argv[]) {

    /** This demo presents cable features : catenary line and dynamic cable.
     *
     * The catenary line is based on a quasi-static model, dedicated to simulate mooring line. This model is suitable for
     * lines with few or no dynamic, and not meant to work with too much traction. In this model, a uniform distributed
     * load is integrated on the line. Flexion and torsion cannot be modeled. The quasi-static approach leads to fast
     * computations, compared to the dynamic cable.
     *
     * The dynamic cable is based on Finite Element Analysis (FEA) solving, with an Euler Beam approximation. Flexion and
     * torsion are integrated. More precise than the catenary line, it is however more time consuming. It is particularly
     * dedicated for cable with large motions and strains, heavy loads, violent dynamic simulations, etc.
     *
     * Three demo are presented in this tutorial, with catenary lines and dynamic cables:
     *
     *  * a fixed line : both ends are constrained.
     *  * a pendulum : a sphere balancing at the end of a line, with its other end fixed.
     *  * a Newton pendulum : consists of a series of identically sized metal balls suspended in a metal frame so that
     *      they are just touching each other at rest. Each ball is attached to the frame by two lines of equal length
     *      angled away from each other. This restricts the pendulums' movements to the same plane.
     *
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;

    // Hide the free surface and seabed visual assets.
    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);
    system.GetEnvironment()->GetOcean()->ShowSeabed(false);

    // Definition of the three demo with cables
    enum cableCase{ FixedLine, Pendulum, Newton_Pendulum};

    // Chose the one you want to run
    cableCase Case = FixedLine;

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
            bool elastic = true;                                    //  non elastic catenary lines are only available for non stretched lines
            double unstretchedLength = 40.;                         //  unstretched length, in m
            double linearDensity = 616.538;                         //  linear density of the line, in kg/m
            double EA = 1.5708e6;                                   //  elasticty
            double diameter = 0.1;                                  //  diameter of the line, in m
            double sectionArea = MU_PI * pow((0.5 * diameter), 2);  //  section area, in m²
            double YoungModulus = EA / sectionArea;                 //  Young modulus of the line, in Pa
            unsigned int nbElement = 50;                            //  number of elements
            double RayleighDamping = 0.;                            //  Rayleigh damping

            // Create the catenary line, using the nodes and line properties previously defined
            auto CatenaryLine = make_catenary_line(Node1, Node2, &system, elastic, unstretchedLength, YoungModulus, sectionArea,
                                                   linearDensity, WATER);

            auto DynamicCable = make_dynamic_cable(Node1,Node2, &system, unstretchedLength, YoungModulus, sectionArea,
                    linearDensity, RayleighDamping, nbElement);

            break;
        }
        // This case features a pendulum : a sphere balancing at the end of a line, with its other end fixed.
        case Pendulum: {
            // Create the moving sphere, from the system
            auto sphere = system.NewBody();
            // Give it a name
            sphere->SetName("Sphere");
            // make it a sphere : gives an asset, a collision box, and its inertia parameters
            makeItSphere(sphere, 1, 1000);

            // Set its initial position and velocity
            double alpha = 30*DEG2RAD;
            double y = 50.*sin(alpha);
            double z = 50.*(1.-cos(alpha));
            sphere->SetPosition(Position(-2., y, z), NWU);

            // create the nodes from the sphere and the world body.
            auto sphereNode = sphere->NewNode();
//            sphereNode->SetPositionInBody({0.,0.,1.},NWU);
            auto worldNode = system.GetWorldBody()->NewNode();
            worldNode->SetPositionInBody(Position(-2., 0., 50.), NWU);

            // Line properties :
            bool elastic = true;                                    //  non elastic catenary lines are only available for non stretched lines
            double unstretchedLength = 50.;                         //  unstretched length, in m
            double linearDensity = 616.538;                         //  linear density of the line, in kg/m
            double EA = 1.5708e7;                                   //  elasticity
            double diameter = 0.5;                                  //  diameter of the line, in m
            double sectionArea = MU_PI * pow((0.5 * diameter), 2);  //  section area, in m²
            double YoungModulus = EA / sectionArea;                 //  Young modulus of the line, in Pa
            unsigned int nbElement = 50;                            //  number of elements
            double RayleighDamping = 0.;                            //  Rayleigh damping

            // Create the catenary line, using the nodes and line properties previously defined
            auto CatenaryLine = make_catenary_line(sphereNode, worldNode, &system, elastic, unstretchedLength,
                                                   YoungModulus, sectionArea, linearDensity, WATER);

            // Same with a Dynamic cable
            // Create the moving sphere, from the system
            auto sphere2 = system.NewBody();
            // Give it a name
            sphere2->SetName("Sphere");
            // make it a sphere : gives an asset, a collision box, and its inertia parameters
            makeItSphere(sphere2, 1, 1000);
            sphere2->SetPosition(Position(2., y, z), NWU);

            // create the nodes from the sphere and the world body.
            auto sphereNode2 = sphere2->NewNode();
            sphereNode2->SetPositionInBody({0.,0.,1},NWU);
            auto worldNode2 = system.GetWorldBody()->NewNode();
            worldNode2->SetPositionInBody(Position(2., 0., 50.), NWU);

            // Dynamic cable properties :
            unstretchedLength -= 1.;

            auto DynamicCable = make_dynamic_cable(sphereNode2, worldNode2, &system, unstretchedLength, YoungModulus,
                                                   sectionArea, linearDensity, RayleighDamping, nbElement);


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
            bool elastic = true;                            //  non elastic catenary lines are only available for non stretched lines
            double unstretchedLength = 51.;                 //  unstretched length, in m
            double linearDensity = 61.6538;                 //  linear density of the line, in kg/m
            double EA = 1.5708e7;                           //  elasticity
            double radius = 0.5;                            //  radius of the line, in m
            double sectionArea = MU_PI * pow((radius), 2);  //  section area, in m²
            double YoungModulus = EA / sectionArea;         //  Young modulus of the line, in Pa
            double RayleighDamping = 0.;                    //  Rayleigh damping
            unsigned int nbElements = 20;                   //  Number of elements of the dynamic cable

            double alpha = 15*DEG2RAD;
            double y = unstretchedLength*sin(alpha);
            double z = unstretchedLength*(1.-cos(alpha));

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
                sphere->SetPosition(Position(10., diameter * ib, 0.), NWU);
                if (ib == 0) {
                    sphere->SetPosition(Position(10., -y, z), NWU);
                }

                // Create the nodes
                auto sphereNode = sphere->NewNode();

                auto worldNode1 = system.GetWorldBody()->NewNode();
                worldNode1->SetPositionInBody(Position(20., diameter * ib, 50.), NWU);
                auto worldNode2 = system.GetWorldBody()->NewNode();
                worldNode2->SetPositionInBody(Position(0., diameter * ib, 50.), NWU);

                // Create the catenary lines, using the nodes and line properties previously defined
                auto CatenaryLine1 = make_catenary_line(worldNode1, sphereNode, &system, elastic, unstretchedLength, YoungModulus,
                                                        sectionArea, linearDensity, FLUID_TYPE::AIR);
                auto CatenaryLine2 = make_catenary_line(worldNode2, sphereNode, &system, elastic, unstretchedLength, YoungModulus,
                                                        sectionArea, linearDensity, FLUID_TYPE::AIR);
                // Set the number of drawn elements on the catenary lines (the more, the slower the simulation)
                CatenaryLine1->SetAssetElements(10);
                CatenaryLine2->SetAssetElements(10);
            }

            // Same with Dynamic cables
            unstretchedLength -= 0.5*diameter;

            // Loop to create the spheres
            for (int ib = 0; ib < n_sphere; ++ib) {
                // Create the sphere from the system
                auto sphere = system.NewBody();

                // Give it a name
                std::stringstream concatenation;
                concatenation << "SphereD" << ib;
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
                sphere->SetPosition(Position(-10., diameter * ib, 0.), NWU);
                if (ib == 0) {
                    sphere->SetPosition(Position(-10., -y, z), NWU);
                }

                // Create the nodes
                auto sphereNode = sphere->NewNode();
                sphereNode->SetPositionInBody({0.,0.,0.5*diameter}, NWU);

                auto worldNode1 = system.GetWorldBody()->NewNode();
                worldNode1->SetPositionInBody(Position(0., diameter * ib, 50.), NWU);
                auto worldNode2 = system.GetWorldBody()->NewNode();
                worldNode2->SetPositionInBody(Position(-20., diameter * ib, 50.), NWU);

                // Create the catenary lines, using the nodes and line properties previously defined
                auto DynamicCable1 = make_dynamic_cable(worldNode1, sphereNode, &system, unstretchedLength, YoungModulus,
                                                        sectionArea, linearDensity, RayleighDamping, nbElements);
                auto DynamicCable2 = make_dynamic_cable(worldNode2, sphereNode, &system, unstretchedLength, YoungModulus,
                                                        sectionArea, linearDensity, RayleighDamping, nbElements);
            }


            break;
        }
    }

    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // You can modify solver and time stepper parameters :
//    system.SetSolver(FrOffshoreSystem::SOLVER::MINRES);
    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(200);
    system.SetSolverMaxIterStab(200);
    system.SetSolverForceTolerance(1e-13);

//    system.SetTimeStepper(FrOffshoreSystem::TIME_STEPPER::EULER_IMPLICIT);
//    system.SetTimeStepper(FrOffshoreSystem::TIME_STEPPER::EULER_IMPLICIT_LINEARIZED);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here infinite) and the distance from the camera to the objectif (50).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(0., 50, false);

}
