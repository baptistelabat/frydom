//
// Created by camille on 11/07/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // System

    FrOffshoreSystem system;

    system.GetPathManager()->SetLogOutputPath("../results");
    system.GetPathManager()->SetLogFrameConvention(NWU);
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));
    system.SetName("HexagonalArticulatedBuoy");

    // Environment

    auto ocean = system.GetEnvironment()->GetOcean();
    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWavePeriod(3.);
    waveField->SetWaveHeight(0); //0.1
    waveField->SetDirection(0., DEG, NWU, GOTO);

    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0., 0., 15., 1.);
    system.GetEnvironment()->GetTimeRamp()->SetActive(false);

    ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-5., 5., 0.5, -5., 5., 0.5);
    ocean->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(5);
    ocean->ShowSeabed(false);

    // Bodies

    auto cyl1 = system.NewBody();
    cyl1->SetName("cyl1");
    cyl1->AddMeshAsset(system.GetDataPath("cylinder_base_bar.obj"));
    cyl1->SetColor(Green);
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
    cyl2->AddMeshAsset(system.GetDataPath("cylinder_base_bar_rz60m.obj"));
    cyl2->SetColor(Yellow);
    cyl2->SetPosition(Position(-1.25, 2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor2(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl2->SetInertiaTensor(inertiaTensor2);

    auto cyl3 = system.NewBody();
    cyl3->SetName("cyl3");
    cyl3->AddMeshAsset(system.GetDataPath("cylinder_base_bar_rz60.obj"));
    cyl3->SetColor(Yellow);
    cyl3->SetPosition(Position(1.25, 2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor3(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.) ,FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
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
    cyl5->AddMeshAsset(system.GetDataPath("cylinder_base_bar_rz60m.obj"));
    cyl5->SetColor(Yellow);
    cyl5->SetPosition(Position(1.25, -2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor5(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
            Position(0., 0., -0.1),
            NWU);
    cyl5->SetInertiaTensor(inertiaTensor5);

    auto cyl6 = system.NewBody();
    cyl6->SetName("cyl6");
    cyl6->AddMeshAsset(system.GetDataPath("cylinder_base_bar_rz60.obj"));
    cyl6->SetColor(Yellow);
    cyl6->SetPosition(Position(-1.25, -2.165, 0.), NWU);
    FrInertiaTensor inertiaTensor6(
            805.033,
            318.66, 100.63, 318.66, 0., 0., 0.,
            FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
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

    auto node_7a = cyl1->NewNode();
    node_7a->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
    auto node_7b = system.GetWorldBody()->NewNode();
    node_7b->SetPositionInWorld(Position(-2.498, 0., 0.), NWU);
    auto link7 = make_prismatic_link(node_7a, node_7b, &system);

    auto node_8a = cyl4->NewNode();
    node_8a->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
    auto node_8b = system.GetWorldBody()->NewNode();
    node_8b->SetPositionInWorld(Position(2.498, 0., 0.), NWU);
    auto link8 = make_prismatic_link(node_8a, node_8b, &system);

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

//    auto radiationModel = make_radiation_convolution_model(hdb, &system);
//    radiationModel->SetImpulseResponseSize(cyl1.get(), 50., 0.02);
//    radiationModel->SetImpulseResponseSize(cyl2.get(), 50., 0.02);
//    radiationModel->SetImpulseResponseSize(cyl3.get(), 50., 0.02);
//    radiationModel->SetImpulseResponseSize(cyl4.get(), 50., 0.02);
//    radiationModel->SetImpulseResponseSize(cyl5.get(), 50., 0.02);
//    radiationModel->SetImpulseResponseSize(cyl6.get(), 50., 0.02);

    // Hydrostatic

//    auto cyl1Mesh = make_hydro_mesh(cyl1, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto cyl2Mesh = make_hydro_mesh(cyl2, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto cyl3Mesh = make_hydro_mesh(cyl3, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.) ,FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto cyl4Mesh = make_hydro_mesh(cyl4, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.), FrRotation(), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto cyl5Mesh = make_hydro_mesh(cyl5, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), M_PI/3., NWU), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto cyl6Mesh = make_hydro_mesh(cyl6, system.GetDataPath("cylinder_base.obj"),
//                                    FrFrame(Position(0., 0., 0.), FrRotation(Direction(0., 0., 1.), -M_PI/3., NWU), NWU),
//                                    FrHydroMesh::ClippingSupport::PLANESURFACE);
//
//    auto forceHst1 = make_nonlinear_hydrostatic_force(cyl1, cyl1Mesh);
//    auto forceHst2 = make_nonlinear_hydrostatic_force(cyl2, cyl2Mesh);
//    auto forceHst3 = make_nonlinear_hydrostatic_force(cyl3, cyl3Mesh);
//    auto forceHst4 = make_nonlinear_hydrostatic_force(cyl4, cyl4Mesh);
//    auto forceHst5 = make_nonlinear_hydrostatic_force(cyl5, cyl5Mesh);
//    auto forceHst6 = make_nonlinear_hydrostatic_force(cyl6, cyl6Mesh);
    
    auto forceHst1 = make_linear_hydrostatic_force(hdb, cyl1);
    auto forceHst2 = make_linear_hydrostatic_force(hdb, cyl2);
    auto forceHst3 = make_linear_hydrostatic_force(hdb, cyl3);
    auto forceHst4 = make_linear_hydrostatic_force(hdb, cyl4);
    auto forceHst5 = make_linear_hydrostatic_force(hdb, cyl5);
    auto forceHst6 = make_linear_hydrostatic_force(hdb, cyl6);

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

    // Simulation

    auto dt = 0.01;

    system.SetTimeStep(dt);
    system.Initialize();

    bool is_irrlicht = true;

    if (is_irrlicht) {
        system.RunInViewer(50., 50., true, 5);
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
