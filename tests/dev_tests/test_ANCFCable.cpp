//
// Created by lletourn on 08/03/19.
//


#include "frydom/frydom.h"

#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "FEAcables.h"

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
    double unstretchedLength = 10.;         // unstretched length
    double linearDensity = 616.538;         // linear density of the line
    double EA = 1.5708e7;                   //
    double sectionArea = 0.05;              // section area
    double YoungModulus = EA / sectionArea; // Young modulus of the line
    double rayleighDamping = 0.;
    unsigned int nbElements = 50;

    auto ANCFCable = std::make_shared<FrANCFCable>(Node1, Node2, unstretchedLength, YoungModulus, sectionArea,
            linearDensity, rayleighDamping, nbElements);


    system.AddANCFCable(ANCFCable);


    //***********************************************************

//    auto my_mesh = std::make_shared<chrono::fea::ChMesh>();
//
//    auto msection_cable2 = std::make_shared<ChBeamSectionCable>();
//    msection_cable2->SetDiameter(0.015);
//    msection_cable2->SetYoungModulus(0.01e9);
//    msection_cable2->SetBeamRaleyghDamping(0.000);
//    msection_cable2->SetDrawCircularRadius(0.05);
//
//    // Shortcut!
//    // This ChBuilderBeamANCF helper object is very useful because it will
//    // subdivide 'beams' into sequences of finite elements of beam type, ex.
//    // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF class.
//    // If new nodes are needed, it will create them for you.
//    ChBuilderBeamANCF builder;
//
//    auto nodeAPos = ChVector<>(0, 0, 0);
//    auto nodeBPos = ChVector<>(10, 0, 0);
//
//    // Now, simply use BuildBeam to create a beam from a point to another:
//    builder.BuildBeam(my_mesh,                       // the mesh where to put the created nodes and elements
//                      msection_cable2,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
//                      50,                         // the number of ChElementBeamANCF to create
//                      nodeAPos,                   // the 'A' point in space (beginning of beam)
//                      nodeBPos);                  // the 'B' point in space (end of beam)
//
//    // After having used BuildBeam(), you can retrieve the nodes used for the beam,
//    // For example say you want to fix both pos and dir of A end and apply a force to the B end:
//    // builder.GetLastBeamNodes().back()->SetFixed(true);
////    builder.GetLastBeamNodes().front()->SetForce(ChVector<>(0, -0.2, 0));
//
//    // Node A
//    auto truc = std::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
//    truc->SetBodyFixed(true);
//    truc->SetPos(ChVector<>(0,0,0));
//
//    auto constraint_hinge1 = std::make_shared<ChLinkPointFrame>();
//    constraint_hinge1->Initialize(builder.GetLastBeamNodes().front(), truc, &truc->GetPos());
//    system.GetChronoSystem()->Add(constraint_hinge1);
//
//    // Node B
//    auto mtruss = std::make_shared<ChBody>();
//    mtruss->SetBodyFixed(true);
//    mtruss->SetPos(ChVector<>(20,0,0));
//
//    auto constraint_hinge2 = std::make_shared<ChLinkPointFrame>();
//    constraint_hinge2->Initialize(builder.GetLastBeamNodes().back(), mtruss, &mtruss->GetPos());
//    system.GetChronoSystem()->Add(constraint_hinge2);
//
//    // ==Asset== attach a visualization of the FEM mesh
//    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
//    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
//    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
//    mvisualizebeamA->SetSmoothFaces(true);
//    mvisualizebeamA->SetWireframe(false);
//    my_mesh->AddAsset(mvisualizebeamA);
//    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
//    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS); // E_GLYPH_NODE_CSYS
//    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
//    mvisualizebeamC->SetSymbolsThickness(0.05);
//    mvisualizebeamC->SetSymbolsScale(0.01);
//    mvisualizebeamC->SetZbufferHide(false);
//    my_mesh->AddAsset(mvisualizebeamC);
//
//
//    system.GetChronoSystem()->Add(my_mesh);

//    system.Initialize();

//    system.GetChronoSystem()->SetupInitial();


//    system.Visualize(20.,false);

//    my_system.SetSolverType(ChSolver::Type::MINRES);
//    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
//    my_system.SetMaxItersSolverSpeed(200);
//    my_system.SetMaxItersSolverStab(200);
//    my_system.SetTolForce(1e-13);
//    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
//    msolver->SetVerbose(false);
//    msolver->SetDiagonalPreconditioning(true);

    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(200);
    system.SetSolverMaxIterStab(200);
    system.SetSolverForceTolerance(1e-13);

    system.RunInViewer(0.,20);




//    // Create the catenary line, using the nodes and line properties previously defined
//    auto CatenaryLine = make_catenary_line(Node1, Node2, &system, elastic, YoungModulus, sectionArea,
//                                           unstretchedLength, linearDensity, WATER);




}

