//
// Created by lletourn on 06/06/19.
//

//#include <frydom/core/statics/FrStability.h>
#include "frydom/frydom.h"

using namespace frydom;



/// Attach two bodies, with a fixed link
/// \param Pos1 position in body1 reference frame, of the fixed link marker
/// \param Pos2 position in body2 reference frame, of the fixed link marker
/// \param fc frame convention (NED/NWU)
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const Position& Pos1, const Position& Pos2, FRAME_CONVENTION fc) {

    auto thisNode = body1->NewNode();
    thisNode->SetPositionInBody(Pos1, fc);

    auto newNode = body2->NewNode();
    newNode->SetPositionInBody(Pos2, fc);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}

/// Attach two bodies, with a fixed link
/// \param frame1 frame in body1 reference frame, of the fixed link marker
/// \param frame2 frame in body2 reference frame, of the fixed link marker
/// \return new body created
void AttachBodies(const std::shared_ptr<FrBody>& body1, const std::shared_ptr<FrBody>& body2,
                  const FrFrame& frame1, const FrFrame& frame2) {

    auto thisNode = body1->NewNode();
    thisNode->SetFrameInBody(frame1);

    auto newNode = body2->NewNode();
    newNode->SetFrameInBody(frame2);

    auto fixedLink = make_fixed_link(thisNode, newNode, body1->GetSystem());

}

int main() {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    FrOffshoreSystem system;
    system.SetName("Seafastening");

//    auto steel = std::make_shared<chrono::ChMaterialSurfaceSMC>();
//    steel->SetYoungModulus(1e8);
//    steel->SetKn(1e8);
//    steel->SetGn(1e10);
//    steel->SetRestitution(0);

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------

    system.GetEnvironment()->GetTimeRamp()->SetActive(true);
    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 10., 1.);

    double Bathy = -30;
    system.GetEnvironment()->GetOcean()->GetSeabed()->SetBathymetry(Bathy,fc);

    auto SeabedGridAsset = system.GetEnvironment()->GetOcean()->GetSeabed()->GetSeabedGridAsset();
    SeabedGridAsset->SetGrid(-150., 150., 3., -150., 150., 3.);

//    auto seabedCollision = std::make_shared<FrCollisionModel>();
//    seabedCollision->AddBox(150,150,2,Position(0.,0.,Bathy-2),FrRotation());
//    seabedCollision->BuildModel();
//    system.GetWorldBody()->SetCollisionModel(seabedCollision);
//
//    system.GetWorldBody()->SetMaterialSurface(steel);

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
    auto waveField = FreeSurface->SetAiryIrregularOptimWaveField();

    // The Airy regular wave parameters are its height, period and direction.
//    double waveHeight = 0.25;    double wavePeriod = 2.*M_PI;
//    Direction waveDirection = Direction(SOUTH(fc));
//
//    waveField->SetWaveHeight(waveHeight);
//    waveField->SetWavePeriod(wavePeriod);
//    waveField->SetDirection(waveDirection, fc, dc);

    // The Airy irregular wave parameters are based on the wave spectrum chosen.
    double Hs = 3;    double Tp = 9;
    auto Jonswap = waveField->SetJonswapWaveSpectrum(Hs, Tp);

    double w1 = 0.5; double w2 = 2; unsigned int nbFreq = 20;
    waveField->SetWaveFrequencies(w1,w2,nbFreq);

    // For a uni-directional wave, you just need to set the mean wave direction. You can also choose to set a
    // direction angle from North direction (see SetMeanWaveDirectionAngle()).
    waveField->SetMeanWaveDirection(Direction(SOUTH(fc)), fc, dc);

//    double spreadingFactor = 10.;    unsigned int nbDir = 10;
//    waveField->SetDirectionalParameters(nbDir, spreadingFactor);

    //-------------------------------------
    // Barge
    //-------------------------------------

    auto CB28 = system.NewBody();
    CB28->SetName("CB28");
    CB28->AddMeshAsset("CB28_Full.obj");
    CB28->SetColor(Yellow);
    CB28->AllowCollision(false);

    // Inertia
//    double mass = 1E6;
//    Position COG = {12.,6.,13.44};
    double mass = 1.3E6;
    Position COG = {0.,0.,0.};
    FrFrame COGFrame(COG, FrRotation(), fc);

    double Ixx = 6.45E7, Iyy = 4.2E8, Izz = 4.8E8;

    CB28->SetInertiaTensor(FrInertiaTensor(mass,Ixx,Iyy,Izz,0.,0.,0.,COG,fc));

//    CB28->SetInertiaTensor(FrInertiaTensor(1.22e+06,
//            1.09522e+08,8.29757e+08,9.25909e+08,-2.97466e+07,-1.58085e+07,-9.51376e+06,
//            Position(9.41639, 4.00892, 12.6138),fc));



    auto eqFrame = std::make_shared<FrEquilibriumFrame>(CB28.get());
    system.AddPhysicsItem(eqFrame);

    auto hdb = make_hydrodynamic_database("CB28.h5");
    hdb->Map(0, CB28.get(), eqFrame);

    // hydrostatic

    auto CBMesh = make_hydro_mesh(CB28, "CB28_Full.obj", FrFrame(), FrHydroMesh::ClippingSupport::WAVESURFACE);

    auto hydrostaticForce = make_nonlinear_hydrostatic_force(CB28, CBMesh);

//    // -- Excitation force
//    auto excitationForce = make_linear_excitation_force(hdb, barge);
    auto FKForce = make_nonlinear_froude_krylov_force(CB28, CBMesh);

    // -- Radiation
    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(CB28.get(), 20., 0.025);



    // damping

//    auto dampingForce = make_linear_damping_force(CB28, WATER, true);
//    dampingForce->SetDiagonalDamping(1E7, 1E7, 1E7, 1E9, 1E9, 1E9);

//    // hydrostatic
//    double K33 = 1.57E7, K44 = 7.71E8, K55 = 5.35E9;
//    FrLinearHydrostaticStiffnessMatrix hydrostaticStiffness;
//    hydrostaticStiffness.SetDiagonal(K33, K44, K55);
//
//    auto hydrostaticForce = make_linear_hydrostatic_force(eqFrame, CB28);
//    hydrostaticForce->SetStiffnessMatrix(hydrostaticStiffness);


//    //-------------------------------------
//    // Pile
//    //-------------------------------------
//
//    auto pile = system.NewBody();
//    pile->SetName("Pile");
//    pile->SetColor(DarkRed);
//    makeItCylinder(pile, 3, 16, 80E3);
//    pile->AllowCollision(false);
//
//    FrFrame pileFrame(Position(0.,-8.,0.), FrRotation(), fc);
//    pileFrame.RotX_DEGREES(-90,fc,true);
//
//    FrFrame CB28Frame(Position(7.6, -4.064, 4.04), FrRotation(), fc);
//
//    AttachBodies(CB28, pile, CB28Frame, pileFrame);
//
////    auto pile = CB28->NewBody(CB28Frame, pileFrame);
//
//    //-------------------------------------
//    // Manifold
//    //-------------------------------------
//
//    auto manifold = system.NewBody(); // CB28->NewBody(Position(-8, -5.6, 4.04), Position(0.,0.,-3), fc);
//    manifold->SetName("Manifold");
//    manifold->SetColor(DarkGreen);
//    makeItBox(manifold, 16, 9.5, 6, 140E3);
//    manifold->AllowCollision(false);
//
//    AttachBodies(CB28, manifold, Position(-8, -5.6, 4.04), Position(0.,0.,-3), fc);

    //-------------------------------------
    // Assembly
    //-------------------------------------

//    FrAssembly assembly;
//    assembly.SetMasterBody(CB28);
//    assembly.AddToAssembly(pile);
//    assembly.AddToAssembly(manifold);

//    CB28->SetFixedInWorld(true);
//
//    system.Initialize();
//    system.DoAssembly();
//
//    CB28->SetFixedInWorld(false);

//    std::cout<<assembly.GetInertiaTensor()<<std::endl;

    system.SetTimeStep(0.01);


    //-------------------------------------
    // Stability
    //-------------------------------------
    system.GetStaticAnalysis()->SetNbSteps(35);
    system.GetStaticAnalysis()->SetNbIteration(20);
//    system.SolveStaticWithRelaxation();

//    std::cout<<"cog assembly"<<CB28->GetPointPositionInWorld(assembly.GetInertiaTensor().GetCOGPosition(NWU), NWU)<<std::endl;
//    std::cout<<"rotation"<<CB28->GetRotation()<<std::endl;


//    FrStability stability(CB28, hydrostaticForce);
//
//    FrInertiaTensor newInertia(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), fc);
//
//    for (int i = 1; i<90; i++)
//        stability.AddRotation(FrRotation(Direction(1,0,0), i*DEG2RAD, NWU));
//
//    stability.ComputeGZ(newInertia);
//    stability.WriteResults("GZ");


    system.RunInViewer(0., 80, false);
//    system.Visualize(50, false);
}
