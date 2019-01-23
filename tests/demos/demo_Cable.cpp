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

    //==================================================================================================================












    //==================================================================================================================
    auto Node1 = system.GetWorldBody()->NewNode(Position(-10.,0.,0.), NWU);

    auto Node2 = system.GetWorldBody()->NewNode(Position(10.,0.,0.), NWU);

    bool elastic = true;
    double youngModulus, sectionArea,cableLength, q;
    mathutils::Vector3d<double> u;

    auto CatenaryLine = std::make_shared<FrCatenaryLine_>(Node1, Node2, elastic, youngModulus, sectionArea, cableLength, q, u);

    system.AddCable(CatenaryLine);
    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.04);

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(60, 75, false);
}
