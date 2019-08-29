//
// Created by camille on 11/07/19.
//

#include "frydom/frydom.h"

using namespace frydom;

void SetUpEnvironment(FrOffshoreSystem* system) {
    // Environment

    auto ocean = system->GetEnvironment()->GetOcean();
    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWavePeriod(3.);
    waveField->SetWaveHeight(0.1); //0.1
    waveField->SetDirection(0., DEG, NWU, GOTO);

    system->GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
    system->GetEnvironment()->GetTimeRamp()->SetActive(false);

    ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-5., 10., 0.5, -5., 5., 0.5);
    ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(5);

    ocean->GetSeabed()->SetBathymetry(-7.5, NWU);
    ocean->GetSeabed()->GetSeabedGridAsset()->SetGrid(-10., 15., 0.5, -12., 12., 0.5);
//    ocean->ShowSeabed(false);

    auto current = ocean->GetCurrent()->GetFieldUniform();
    current->SetNorth(0.5, KNOT, GOTO);
}



int main(int argc, char* argv[]) {

    // System

    FrOffshoreSystem system;

    system.GetPathManager()->SetLogOutputPath("../results");
    system.GetPathManager()->SetLogFrameConvention(NWU);
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));
    system.SetName("HexagonalArticulatedBuoy");

    // Environment
    SetUpEnvironment(&system);

    // Bodies

    auto cyl1 = system.NewBody();
    cyl1->SetName("cyl1");
    cyl1->AddMeshAsset(system.GetDataPath("cylinder_base_bar.obj"));
    cyl1->SetColor(Yellow);
    cyl1->SetPosition(Position(-2.498, 0., 0.), NWU);
    FrInertiaTensor inertiaTensor1(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl1->SetInertiaTensor(inertiaTensor1);

    auto cyl2 = system.NewBody();
    cyl2->SetName("cyl2");
    auto asset2 = std::make_shared<FrTriangleMeshConnected>();
    asset2->LoadWavefrontMesh(system.GetDataPath("cylinder_base_bar.obj"));
    asset2->Rotate(FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU));
    cyl2->AddMeshAsset(asset2);
    cyl2->SetColor(Yellow);
    cyl2->SetPosition(Position(-1.25, 2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor2(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl2->SetInertiaTensor(inertiaTensor2);

    auto cyl3 = system.NewBody();
    cyl3->SetName("cyl3");
    auto asset3 = std::make_shared<FrTriangleMeshConnected>();
    asset3->LoadWavefrontMesh(system.GetDataPath("cylinder_base_bar.obj"));
    asset3->Rotate(FrRotation(Direction(0., 0., 1.), M_PI/3., NWU));
    cyl3->AddMeshAsset(asset3);
    cyl3->SetColor(Yellow);
    cyl3->SetPosition(Position(1.25, 2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor3(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.) ,FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl3->SetInertiaTensor(inertiaTensor3);

    auto cyl4 = system.NewBody();
    cyl4->SetName("cyl4");
    cyl4->AddMeshAsset(system.GetDataPath("cylinder_base_bar.obj"));
    cyl4->SetColor(Yellow);
    cyl4->SetPosition(Position(2.498, 0., 0.), NWU);
    FrInertiaTensor inertiaTensor4(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl4->SetInertiaTensor(inertiaTensor4);

    auto cyl5 = system.NewBody();
    cyl5->SetName("cyl5");
    cyl5->AddMeshAsset(asset2);
    cyl5->SetColor(Yellow);
    cyl5->SetPosition(Position(1.25, -2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor5(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl5->SetInertiaTensor(inertiaTensor5);

    auto cyl6 = system.NewBody();
    cyl6->SetName("cyl6");
    cyl6->AddMeshAsset(asset3);
    cyl6->SetColor(Yellow);
    cyl6->SetPosition(Position(-1.25, -2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor6(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl6->SetInertiaTensor(inertiaTensor6);

    // Links

    auto node_1a = cyl1->NewNode();
    node_1a->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);
    auto node_1b = cyl2->NewNode();
    node_1b->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);
    auto link1 = make_spherical_link(node_1a, node_1b, &system);

    auto node_2a = cyl2->NewNode();
    node_2a->SetPositionInWorld(Position(0., 2.887, 0.), NWU);
    auto node_2b = cyl3->NewNode();
    node_2b->SetPositionInWorld(Position(0., 2.887, 0.), NWU);
    auto link2 = make_spherical_link(node_2a, node_2b, &system);

    auto node_3a = cyl3->NewNode();
    node_3a->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);
    auto node_3b = cyl4->NewNode();
    node_3b->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);
    auto link3 = make_spherical_link(node_3a, node_3b, &system);

    auto node_4a = cyl4->NewNode();
    node_4a->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);
    auto node_4b = cyl5->NewNode();
    node_4b->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);
    auto link4 = make_spherical_link(node_4a, node_4b, &system);

    auto node_5a = cyl5->NewNode();
    node_5a->SetPositionInWorld(Position(0., -2.887, 0.), NWU);
    auto node_5b = cyl6->NewNode();
    node_5b->SetPositionInWorld(Position(0., -2.887, 0.), NWU);
    auto link5 = make_spherical_link(node_5a, node_5b, &system);

    auto node_6a = cyl6->NewNode();
    node_6a->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);
    auto node_6b = cyl1->NewNode();
    node_6b->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);
    auto link6 = make_spherical_link(node_6a, node_6b, &system);

//    auto node_7a = cyl1->NewNode();
//    node_7a->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
//    auto node_7b = system.GetWorldBody()->NewNode();
//    node_7b->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
//    auto link7 = make_prismatic_link(node_7a, node_7b, &system);
//
//    auto node_8a = cyl4->NewNode();
//    node_8a->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
//    auto node_8b = system.GetWorldBody()->NewNode();
//    node_8b->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
//    auto link8 = make_prismatic_link(node_8a, node_8b, &system);

    // Hydrodynamics

    auto hdb = make_hydrodynamic_database(system.GetDataPath("hexagonal_articulated_buoy.hdb5"));

    auto eqFrame0 = std::make_shared<FrEquilibriumFrame>(Position(-2.498, 0., 0.), FrRotation(), NWU, cyl1.get());
    auto eqFrame1 = std::make_shared<FrEquilibriumFrame>(Position(-1.25, 2.165, 0.), FrRotation(), NWU, cyl2.get());
    auto eqFrame2 = std::make_shared<FrEquilibriumFrame>(Position(1.25, 2.165, 0.), FrRotation(), NWU, cyl3.get());
    auto eqFrame3 = std::make_shared<FrEquilibriumFrame>(Position(2.498, 0., 0.), FrRotation(), NWU, cyl4.get());
    auto eqFrame4 = std::make_shared<FrEquilibriumFrame>(Position(1.25, -2.165, 0.), FrRotation(), NWU, cyl5.get());
    auto eqFrame5 = std::make_shared<FrEquilibriumFrame>(Position(-1.25, -2.165, 0.), FrRotation(), NWU, cyl6.get());

    hdb->Map(0, cyl1.get(), eqFrame0);
    hdb->Map(1, cyl2.get(), eqFrame1);
    hdb->Map(2, cyl3.get(), eqFrame2);
    hdb->Map(3, cyl4.get(), eqFrame3);
    hdb->Map(4, cyl5.get(), eqFrame4);
    hdb->Map(5, cyl6.get(), eqFrame5);

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(cyl1.get(), 50., 0.02);
    radiationModel->SetImpulseResponseSize(cyl2.get(), 50., 0.02);
    radiationModel->SetImpulseResponseSize(cyl3.get(), 50., 0.02);
    radiationModel->SetImpulseResponseSize(cyl4.get(), 50., 0.02);
    radiationModel->SetImpulseResponseSize(cyl5.get(), 50., 0.02);
    radiationModel->SetImpulseResponseSize(cyl6.get(), 50., 0.02);

    // Hydrostatic

    auto cyl1Mesh = make_hydro_mesh(cyl1, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto cyl2Mesh = make_hydro_mesh(cyl2, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto cyl3Mesh = make_hydro_mesh(cyl3, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.) ,FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto cyl4Mesh = make_hydro_mesh(cyl4, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto cyl5Mesh = make_hydro_mesh(cyl5, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto cyl6Mesh = make_hydro_mesh(cyl6, system.GetDataPath("cylinder_base.obj"),
                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
                                    FrHydroMesh::ClippingSupport::PLANESURFACE);

    auto forceHst1 = make_nonlinear_hydrostatic_force(cyl1, cyl1Mesh);
    auto forceHst2 = make_nonlinear_hydrostatic_force(cyl2, cyl2Mesh);
    auto forceHst3 = make_nonlinear_hydrostatic_force(cyl3, cyl3Mesh);
    auto forceHst4 = make_nonlinear_hydrostatic_force(cyl4, cyl4Mesh);
    auto forceHst5 = make_nonlinear_hydrostatic_force(cyl5, cyl5Mesh);
    auto forceHst6 = make_nonlinear_hydrostatic_force(cyl6, cyl6Mesh);

    // Motor

    /*
    auto node_9a = cyl1->NewNode();
    node_9a->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
    auto node_9b = system.GetWorldBody()->NewNode();
    node_9b->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
    auto link9 = make_prismatic_link(node_9a, node_9b, &system);

    auto motor = link9->Motorize(POSITION);

    auto x = new_var("x");
    auto ramp = FrLinearRampFunction();
    ramp.SetByTwoPoints(0., 0., 10., 1.);
    auto ramp2 = FrLinearRampFunction();
    ramp.SetByTwoPoints(15., 1., 20., 0.);
    motor->SetMotorFunction(ramp * ramp2 * 0.3*sin(2.*M_PI/3.*x));
    */

        // Excitation

    auto excitationForce1 = make_linear_excitation_force(hdb, cyl1);
    auto excitationForce2 = make_linear_excitation_force(hdb, cyl2);
    auto excitationForce3 = make_linear_excitation_force(hdb, cyl3);
    auto excitationForce4 = make_linear_excitation_force(hdb, cyl4);
    auto excitationForce5 = make_linear_excitation_force(hdb, cyl5);
    auto excitationForce6 = make_linear_excitation_force(hdb, cyl6);

    // Morison

    auto cd = 3.2;

    auto morisonModel1 = make_morison_model(cyl1.get());
    //morisonModel1->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel1->AddElement(Position(0., -1, 0.), Position(0., 1., 0.), 1., 0., cd, 0., 5);
    auto morisonForce1 = make_morison_force(morisonModel1, cyl1);

    auto morisonModel2 = make_morison_model(cyl2.get());
    //morisonModel2->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel2->AddElement(Position(-0.866, -0.5, 0.), Position(0.866, 0.5, 0.), 1., 0., cd, 0., 5);
    auto morisonForce2 = make_morison_force(morisonModel2, cyl2);

    auto morisonModel3 = make_morison_model(cyl3.get());
    //morisonModel3->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel3->AddElement(Position(-0.866, +0.5, 0.), Position(0.866, -0.5, 0.), 1., 0., cd, 0., 5);
    auto morisonForce3 = make_morison_force(morisonModel3, cyl3);

    auto morisonModel4 = make_morison_model(cyl4.get());
    //morisonModel4->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel4->AddElement(Position(0., -1, 0.), Position(0., 1., 0.), 1., 0., cd, 0., 5);
    auto morisonForce4 = make_morison_force(morisonModel4, cyl4);

    auto morisonModel5 = make_morison_model(cyl5.get());
    //morisonModel5->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel5->AddElement(Position(-0.866, -0.5, 0.), Position(0.866, 0.5, 0.), 1., 0., cd, 0., 5);
    auto morisonForce5 = make_morison_force(morisonModel5, cyl5);

    auto morisonModel6 = make_morison_model(cyl6.get());
    //morisonModel6->AddElement(FrFrame(), 2., 1., 0., cd, 0. );
    morisonModel6->AddElement(Position(-0.866, +0.5, 0.), Position(0.866, -0.5, 0.), 1., 0., cd, 0., 5);
    auto morisonForce6 = make_morison_force(morisonModel6, cyl6);


    // Mooring Line

    bool elastic = false;
    double uLength = 12.5;

    auto cableProp = make_cable_properties();
    cableProp->SetSectionArea(0.00196);
    cableProp->SetEA(2.95E5);
    cableProp->SetLinearDensity(9.03);

    auto worldNodeNW = system.GetWorldBody()->NewNode();
    worldNodeNW->SetPositionInWorld(Position(-10, 5.76, -7.5), NWU);
    auto bodyNodeNW = cyl1->NewNode();
    bodyNodeNW->SetPositionInWorld(Position(-2.498, 1.444, 0.), NWU);

    auto worldNodeN = system.GetWorldBody()->NewNode();
    worldNodeN->SetPositionInWorld(Position(0., 11.548, -7.5), NWU);
    auto bodyNodeN = cyl2->NewNode();
    bodyNodeN->SetPositionInWorld(Position(0., 2.887, 0.), NWU);

    auto worldNodeNE = system.GetWorldBody()->NewNode();
    worldNodeNE->SetPositionInWorld(Position(10., 5.76, -7.5), NWU);
    auto bodyNodeNE = cyl3->NewNode();
    bodyNodeNE->SetPositionInWorld(Position(2.498, 1.444, 0.), NWU);

    auto worldNodeSE = system.GetWorldBody()->NewNode();
    worldNodeSE->SetPositionInWorld(Position(10., -5.76, -7.5), NWU);
    auto bodyNodeSE = cyl4->NewNode();
    bodyNodeSE->SetPositionInWorld(Position(2.498, -1.444, 0.), NWU);

    auto worldNodeS = system.GetWorldBody()->NewNode();
    worldNodeS->SetPositionInWorld(Position(0., -11.548, -7.5), NWU);
    auto bodyNodeS = cyl5->NewNode();
    bodyNodeS->SetPositionInWorld(Position(0., -2.887, 0.), NWU);

    auto worldNodeSW = system.GetWorldBody()->NewNode();
    worldNodeSW->SetPositionInWorld(Position(-10., -5.76, -7.5), NWU);
    auto bodyNodeSW = cyl6->NewNode();
    bodyNodeSW->SetPositionInWorld(Position(-2.498, -1.444, 0.), NWU);

    auto mooringLineNW = make_catenary_line(worldNodeNW, bodyNodeNW, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
    auto mooringLineN  = make_catenary_line(worldNodeN, bodyNodeN, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
    auto mooringLineNE = make_catenary_line(worldNodeNE, bodyNodeNE, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
    auto mooringLineSE = make_catenary_line(worldNodeSE, bodyNodeSE, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
    auto mooringLineS  = make_catenary_line(worldNodeS, bodyNodeS, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);
    auto mooringLineEW = make_catenary_line(worldNodeSW, bodyNodeSW, &system, cableProp, elastic, uLength, FLUID_TYPE::WATER);

    // Current force as quadratic damping

    double Su, Sv, Sw; Su = 2.; Sv = MU_PI ; Sw = 2.;

    auto currentForce1 = make_quadratic_damping_force(cyl1, WATER, true);
    currentForce1->SetProjectedSections(Su,Sv,Sw);
    currentForce1->SetDampingCoefficients(1.,1.,1.);

    auto currentForce2 = make_quadratic_damping_force(cyl2, WATER, true);
    currentForce2->SetProjectedSections(1.5,1.5,Sw);
    currentForce2->SetDampingCoefficients(1.,1.,1.);

    auto currentForce3 = make_quadratic_damping_force(cyl3, WATER, true);
    currentForce3->SetProjectedSections(1.5,1.5,Sw);
    currentForce3->SetDampingCoefficients(1.,1.,1.);

    auto currentForce4 = make_quadratic_damping_force(cyl4, WATER, true);
    currentForce4->SetProjectedSections(Su,Sv,Sw);
    currentForce4->SetDampingCoefficients(1.,1.,1.);

    auto currentForce5 = make_quadratic_damping_force(cyl5, WATER, true);
    currentForce5->SetProjectedSections(1.5,1.5,Sw);
    currentForce5->SetDampingCoefficients(1.,1.,1.);

    auto currentForce6 = make_quadratic_damping_force(cyl6, WATER, true);
    currentForce6->SetProjectedSections(1.5,1.5,Sw);
    currentForce6->SetDampingCoefficients(1.,1.,1.);

    // Simulation

    auto dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    bool is_irrlicht = true;

    if (is_irrlicht) {
        system.RunInViewer(50., 10., false, 5);
    } else {
        auto time = 0.;
        while(time < 50.) {
            time += dt;
            system.AdvanceTo(time);
            std::cout << "Time : " << time << "s " << std::endl;
        }
    }
    std::cout << " ============================================== End =============================== " << std::endl;
    return 0;
}
