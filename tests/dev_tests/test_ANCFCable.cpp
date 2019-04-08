//
// Created by lletourn on 08/03/19.
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
    FrOffshoreSystem system;

    // Hide the free surface and seabed visual assets.
    system.GetEnvironment()->GetOcean()->ShowFreeSurface(false);
    system.GetEnvironment()->GetOcean()->ShowSeabed(false);

    // Create the nodes from the world body (which is fixed)
    auto Node1 = system.GetWorldBody()->NewNode();
    Node1->SetPositionInBody(Position(-20., 0., 0.), NWU);
    auto Node2 = system.GetWorldBody()->NewNode();
    Node2->SetPositionInBody(Position(0., 0., 0.), NWU);

    // Line properties :
//    bool elastic = true;                    // non elastic catenary lines are only available for non stretched lines
    auto u = Direction(0., 0., -1.);        // definition of the direction of the uniform load distribution
    double unstretchedLength = 18.;         // unstretched length
    double linearDensity = 616.538;         // linear density of the line
    double EA = 1.5708e7;
    double diameter = 0.1;                 //
    double sectionArea = MU_PI * pow((0.5 * diameter), 2);              // section area
    double YoungModulus = EA / sectionArea; // Young modulus of the line
    double rayleighDamping = 0.;
    unsigned int nbElements = 50;

    // ANCF cable
    auto ANCFCable = std::make_shared<FrDynamicCable>(Node1, Node2, unstretchedLength, YoungModulus, sectionArea,
            linearDensity, rayleighDamping, nbElements);

    system.AddANCFCable(ANCFCable);

    // Catenary line
//    linearDensity += sectionArea * system.GetEnvironment()->GetFluidDensity(AIR);
    linearDensity *= 9.81;

    auto CatenaryLine = make_catenary_line(Node1,Node2,&system,true,YoungModulus,sectionArea,unstretchedLength,linearDensity,AIR);
    CatenaryLine->SetAssetElements(50);


//    system.Visualize(20.,false);

    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(200);
    system.SetSolverMaxIterStab(200);
    system.SetSolverForceTolerance(1e-13);

    system.SetTimeStep(0.01);

//    system.GetStaticAnalysis()->SetNbIteration(20);
//    system.GetStaticAnalysis()->SetNbSteps(200);
////    system.SolveStaticWithRelaxation();
//    system.VisualizeStaticAnalysis(20.,false);

    system.RunInViewer(0.,20);


}

