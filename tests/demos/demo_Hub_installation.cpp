//
// Created by lucas on 01/02/19.
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

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------
    auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();
    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset();

    // The free surface grid is defined here as a squared one ranging from -100m to 100m (in north and west
    // directions) with a 2m steps.
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(10);

    // WaveField
    auto waveField = FreeSurface->SetAiryRegularOptimWaveField();

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 1.;    double wavePeriod = 2.*M_PI;
    Direction waveDirection = Direction(SOUTH(fc));

    waveField->SetWaveHeight(waveHeight);
    waveField->SetWavePeriod(wavePeriod);
    waveField->SetDirection(waveDirection, fc, dc);


    // --------------------------------------------------
    // Barge model
    // --------------------------------------------------

    auto barge = system.NewBody();
    barge->SetName("Barge");
    makeItBox(barge, 25., 15., 3., (1137.6-180.6)*1000);
//    barge->AddMeshAsset("Barge.obj");
    barge->SetColor(Yellow);
    barge->SetBodyFixed(true); //FIXME : delete this once HydroDB is added.

//    barge->SetInertiaTensor(FrInertiaTensor_((1137.6-180.6)*1000, 2.465e7,1.149e7,1.388e07, 0.,0.,0., FrFrame_(), NWU));

    float steelYoungModulus = 1e12; // Young modulus (for contact)
    float steelNormalDamping = 1e20;// Normal damping (for contact)
    auto steel = barge->GetMaterialSurface();
    steel->SetKn(steelYoungModulus);
    steel->SetGn(steelNormalDamping);
    steel->young_modulus = steelYoungModulus;
    steel->restitution = 0;

    // TODO: Add the HydroDB with the hydrodynamic forces

    // --------------------------------------------------
    // Crane model
    // --------------------------------------------------

    auto base_crane = system.NewBody();
    base_crane->SetName("Base_Crane");
    makeItBox(base_crane, 5., 5., 2., 120e3);
    base_crane->SetColor(Red);
    base_crane->SetPosition(Position(-7.5,0,2.5), fc);

    auto base_crane_1 = system.NewBody();
    makeItBox(base_crane_1,2.5,1.5,2, 10e3);
    base_crane_1->SetColor(Red);
    base_crane_1->SetPosition(Position(-7.5,1.75,4.5), fc);

    auto base_crane_2 = system.NewBody();
    makeItBox(base_crane_2,2.5,1.5,2, 10e3);
    base_crane_2->SetColor(Red);
    base_crane_2->SetPosition(Position(-7.5,-1.75,4.5), fc);


    auto arm_crane = system.NewBody();
    arm_crane->SetName("Arm_Crane");
    makeItBox(arm_crane,19.,1.5,1.5,20e3);
    arm_crane->SetColor(Green);
    arm_crane->SetPositionOfBodyPoint(Position(-9.0,0.,0.), Position(-7.5,0.,4.5), fc);

    FrRotation_ arm_Rotation;
    arm_Rotation.SetCardanAngles_DEGREES(0.,-45.,0., fc);
    arm_crane->RotateAroundPointInBody(arm_Rotation, Position(-9.0,0.,0.), fc);

    auto crane_node = arm_crane->NewNode();
    crane_node->SetPositionInBody(Position(9.,0.,-0.75), fc);

    // --------------------------------------------------
    // hub box model
    // --------------------------------------------------

    auto hub_box = system.NewBody();
    hub_box->SetName("Hub_Box");
    makeItBox(hub_box, 1.5,1.5,1.5, 20.7e3);
    hub_box->SetPosition(Position(crane_node->GetPositionInWorld(fc).GetX(),0.,2.25), fc);

    auto hub_node = hub_box->NewNode();
    hub_node->SetPositionInBody(Position(0.,0.,0.75), fc);

    // --------------------------------------------------
    // Hub Line
    // --------------------------------------------------
    // Line properties
    bool elastic = true;
    double unstretchedLength = crane_node->GetPositionInWorld(fc).GetZ() - hub_node->GetPositionInWorld(fc).GetZ();
    auto u = Direction(0, 0, -1);
    double linearDensity = 600;
    double EA = 5e8;
    double sectionArea = 0.05;
    double YoungModulus = EA / sectionArea;
    double breakTensionAsset = 500000;

    auto CatenaryLine = makeCatenaryLine(crane_node, hub_node, &system, elastic, YoungModulus,sectionArea,
                                         unstretchedLength, linearDensity, u, fc);


    // --------------------------------------------------
    // Mooring Lines
    // --------------------------------------------------




    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
//    system.Visualize(50.,false);
    system.RunInViewer(30, 50, false);
}