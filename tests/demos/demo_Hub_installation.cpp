//
// Created by lucas on 01/02/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {
    /** 
     * 
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

    auto SeabedGridAsset = system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset();
    SeabedGridAsset->SetGrid(-150., 150., 3., -150., 150., 3.);


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
    makeItBox(barge, 25., 15., 3., (1137.6-180.7)*1000);
//    barge->AddMeshAsset("Barge.obj");
    barge->SetColor(Yellow);
//    barge->SetFixedInWorld(true); //FIXME : delete this once HydroDB is added.

//    barge->SetInertiaTensor(FrInertiaTensor_((1137.6-180.6)*1000, 2.465e7,1.149e7,1.388e07, 0.,0.,0., FrFrame_(), NWU));

    float steelYoungModulus = 1e12; // Young modulus (for contact)
    float steelNormalDamping = 1e20;// Normal damping (for contact)
    auto steel = barge->GetMaterialSurface();
    steel->SetKn(steelYoungModulus);
    steel->SetGn(steelNormalDamping);
    steel->young_modulus = steelYoungModulus;
    steel->restitution = 0;

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database("Barge_frydom.h5");

    auto eqFrame = std::make_shared<FrEquilibriumFrame_>(barge.get());
    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, barge.get(), eqFrame);

    // -- Hydrostatic
    auto hydrostaticForce = make_linear_hydrostatic_force(hdb, barge);

    // -- Excitation force
    auto excitationForce = make_linear_excitation_force(hdb, barge);

    // -- Radiation
    auto radiationModel = make_radiation_convolution_model(hdb, &system);

    radiationModel->SetImpulseResponseSize(barge.get(), 6., 0.1);

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
    arm_crane->SetColor(DarkGreen);
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

    auto CatenaryLine = make_catenary_line(crane_node, hub_node, &system, elastic, YoungModulus, sectionArea,
                                           unstretchedLength, linearDensity, u, fc);


    // --------------------------------------------------
    // Mooring Lines
    // --------------------------------------------------
    double buoyRadius = 0.75;
    double buoyMass = 100;
    double buoyDamping = 1000;


    auto buoySE = make_mooring_buoy(&system, buoyRadius, buoyMass, true, buoyDamping);
    buoySE->SetPosition(Position(-50.,-25.,0.), NWU);
    auto buoyNodeSE = buoySE->NewNode();

    auto buoySW = make_mooring_buoy(&system, buoyRadius, buoyMass, true, buoyDamping);
    buoySW->SetPosition(Position(-50.,25.,0.), NWU);
    auto buoyNodeSW = buoySW->NewNode();

    auto buoyNE = make_mooring_buoy(&system, buoyRadius, buoyMass, true, buoyDamping);
    buoyNE->SetPosition(Position(50.,-25.,0.), NWU);
    auto buoyNodeNE = buoyNE->NewNode();

    auto buoyNW = make_mooring_buoy(&system, buoyRadius, buoyMass, true, buoyDamping);
    buoyNW->SetPosition(Position(50.,25.,0.), NWU);
    auto buoyNodeNW = buoyNW->NewNode();


    auto worldNodeSE = system.GetWorldBody()->NewNode();
    worldNodeSE->SetPositionInBody(Position(-125.,-75.,-system.GetEnvironment()->GetOcean()->GetDepth(fc)), fc);
    auto worldNodeNE = system.GetWorldBody()->NewNode();
    worldNodeNE->SetPositionInBody(Position(125.,-75.,-system.GetEnvironment()->GetOcean()->GetDepth(fc)), fc);
    auto worldNodeSW = system.GetWorldBody()->NewNode();
    worldNodeSW->SetPositionInBody(Position(-125.,75.,-system.GetEnvironment()->GetOcean()->GetDepth(fc)), fc);
    auto worldNodeNW = system.GetWorldBody()->NewNode();
    worldNodeNW->SetPositionInBody(Position(125.,75.,-system.GetEnvironment()->GetOcean()->GetDepth(fc)), fc);

    auto bargeNodeSE = barge->NewNode();
    bargeNodeSE->SetPositionInBody(Position(-12.5,-5.,0.),fc);
    auto bargeNodeSW = barge->NewNode();
    bargeNodeSW->SetPositionInBody(Position(-12.5,5.,0.),fc);
    auto bargeNodeNE = barge->NewNode();
    bargeNodeNE->SetPositionInBody(Position(12.5,-5.,0.),fc);
    auto bargeNodeNW = barge->NewNode();
    bargeNodeNW->SetPositionInBody(Position(12.5,5.,0.),fc);

    /// Mooring lines length

    unstretchedLength = 95;
    linearDensity = 20.;
    EA = 5e6;
    sectionArea = 0.025;
    YoungModulus = EA / sectionArea;
    double Lv = 42.5;
    breakTensionAsset = 50000;

    auto mooringLineSE = make_catenary_line(worldNodeSE, buoyNodeSE, &system, elastic, YoungModulus, sectionArea,
                                            unstretchedLength, linearDensity, u, fc);
    auto tetherLineSE = make_catenary_line(bargeNodeSE, buoyNodeSE, &system, elastic, YoungModulus, sectionArea,
                                           Lv, linearDensity, u, fc);

    auto mooringLineSW = make_catenary_line(worldNodeSW, buoyNodeSW, &system, elastic, YoungModulus, sectionArea,
                                            unstretchedLength, linearDensity, u, fc);
    auto tetherLineSW = make_catenary_line(bargeNodeSW, buoyNodeSW, &system, elastic, YoungModulus, sectionArea,
                                           Lv, linearDensity, u, fc);

    auto mooringLineNE = make_catenary_line(worldNodeNE, buoyNodeNE, &system, elastic, YoungModulus, sectionArea,
                                            unstretchedLength, linearDensity, u, fc);
    auto tetherLineNE = make_catenary_line(bargeNodeNE, buoyNodeNE, &system, elastic, YoungModulus, sectionArea,
                                           Lv, linearDensity, u, fc);

    auto mooringLineNW = make_catenary_line(worldNodeNW, buoyNodeNW, &system, elastic, YoungModulus, sectionArea,
                                            unstretchedLength, linearDensity, u, fc);
    auto tetherLineNW = make_catenary_line(bargeNodeNW, buoyNodeNW, &system, elastic, YoungModulus, sectionArea,
                                           Lv, linearDensity, u, fc);

    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.01);

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
//    system.Visualize(50.,false);
    system.RunInViewer(30, 50, false);
}