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

FrLinearActuator* make_carriage(FrOffshoreSystem* system, const std::shared_ptr<FrNode>& shipNode,
        bool is_captive){

    FRAME_CONVENTION fc = NWU;

    auto mass = shipNode->GetBody()->GetInertiaTensor().GetMass();

    double tankLength = 140;
    double tankWidth = 5;
    double tankDepth = 3.048;

    // Resources path
    cppfs::FilePath resources_path(std::string(RESOURCES_PATH));

    // --------------------------------------------------
    // Seabed and Free-surface grid definitions
    // --------------------------------------------------
    auto Seabed = system->GetEnvironment()->GetOcean()->GetSeabed();
    Seabed->GetSeabedGridAsset()->SetGrid(
            -0.05*tankLength, 0.95*tankLength, 0.01*tankLength,
            -0.5*tankWidth, 0.5*tankWidth, 0.01*tankWidth);
    Seabed->SetBathymetry(-tankDepth,fc);

    auto FreeSurface = system->GetEnvironment()->GetOcean()->GetFreeSurface();
    FreeSurface->GetFreeSurfaceGridAsset()->SetGrid(
            -0.05*tankLength, 0.95*tankLength, 0.001*tankLength,
            -0.5*tankWidth, 0.5*tankWidth, 0.01*tankWidth);
    FreeSurface->GetFreeSurfaceGridAsset()->UpdateAssetON();
    FreeSurface->GetFreeSurfaceGridAsset()->SetUpdateStep(10);

    // --------------------------------------------------
    // Wall definition
    // --------------------------------------------------

    auto tankWall = system->NewBody();
    tankWall->SetName("Wall");
    tankWall->SetFixedInWorld(true);
    makeItBox(tankWall, tankLength, 0.1*tankWidth, 1.25*tankDepth, mass);

    Position tankWallPosition = shipNode->GetPositionInWorld(fc); tankWallPosition.GetZ() = 0.;
    tankWallPosition -= Position(-0.45*tankLength,0.5*tankWidth,0.375*tankDepth);
    tankWall->SetPosition(tankWallPosition,fc);

    auto wallNode = tankWall->NewNode();
    wallNode->SetPositionInBody(Position(-0.45*tankLength,0.,0.75*tankDepth), fc);
    wallNode->RotateAroundYInBody(-90*DEG2RAD,fc);

    // --------------------------------------------------
    // Carriage definition
    // --------------------------------------------------

    auto carriage = system->NewBody();
    carriage->SetName("Carriage");
    carriage->SetPositionOfBodyPoint(Position(0.,-0.5*tankWidth,0.), wallNode->GetPositionInWorld(fc), fc);

    double xSize = 0.1*tankWidth;
    double ySize = 1.1*tankWidth;
    double zSize = 0.1*tankWidth;

    // Properties of the box
    double xSize2 = xSize * xSize;
    double ySize2 = ySize * ySize;
    double zSize2 = zSize * zSize;

    // inertia
    double Ixx = (1./12.) * mass * (ySize2 + zSize2);
    double Iyy = (1./12.) * mass * (xSize2 + zSize2);
    double Izz = (1./12.) * mass * (xSize2 + ySize2);

    carriage->SetInertiaTensor(FrInertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., Position(), NWU));

    auto carriageAsset = std::make_shared<FrTriangleMeshConnected>();
    carriageAsset->LoadWavefrontMesh(resources_path.resolve("carriage.obj").path());
    carriageAsset->Scale(tankWidth);
    carriage->AddMeshAsset(carriageAsset);

    auto carriageToWallNode = carriage->NewNode();
    carriageToWallNode->SetPositionInBody(Position(0.,-0.5*tankWidth,0.), fc);
    carriageToWallNode->RotateAroundYInBody(-90*DEG2RAD,fc);

    auto carriageToShipNode = carriage->NewNode();
    carriageToShipNode->SetPositionInWorld(shipNode->GetPositionInWorld(fc), fc);
    carriageToShipNode->RotateAroundYInBody(90*DEG2RAD,fc);
    carriageToShipNode->RotateAroundXInBody(90*DEG2RAD,fc);

    // --------------------------------------------------
    // Link definitions
    // --------------------------------------------------

    if (is_captive)
        auto linkToShip = make_fixed_link(carriageToShipNode, shipNode, system);
    else
        auto linkToShip = make_prismatic_revolute_link(carriageToShipNode, shipNode, system);

    auto rail = make_prismatic_link(wallNode, carriageToWallNode, system);

    return rail->Motorize(VELOCITY);
}

// ----------------------------------------------------------
// Steady Pitch Torque
// ----------------------------------------------------------

class SteadyPitchTorque : public FrForce {

    void Compute(double time) override {

        auto speed = m_body->GetVelocityInWorld(NWU).GetVx();
        auto torque = 4.332 * std::pow(speed, 6)
                    - 9.1135 * std::pow(speed, 5)
                    - 9.7756 * std::pow(speed, 4)
                    + 34.232 * std::pow(speed, 3)
                    - 22.7359 * std::pow(speed, 2);

        SetTorqueInBodyAtCOG(Torque(0., -torque, 0.), NWU);

    }
};

// ----------------------------------------------------------
// Steady Heave Force
// ----------------------------------------------------------

class SteadyHeaveForce : public FrForce {

    void Compute(double time) override {

        auto speed = m_body->GetVelocityInWorld(NWU).GetVx();

        auto force = -12.32426 * std::pow(speed, 3)
                   - 2.8696 * std::pow(speed, 2);

        SetForceInWorldAtCOG(Force(0., 0., force), NWU);

    }
};

// -----------------------------------------------------------
// ITTC57 : residual coefficient
// -----------------------------------------------------------

double ResidualITTC(double speed) {

    if (std::abs(speed - 1.04) < 1E-2) {
        return 5.3696e-4;
    } else if (std::abs(speed - 1.532) < 1E-2) {
        return 9.0677e-4;
    } else if (std::abs(speed - 1.86) < 1E-2) {
        return 1.6812e-3;
    } else if (std::abs(speed - 2.243) < 1E-2) {
        return 4.02529e-3;
    } else {
        std::cout << "warning : no residual coefficient for this speed value" << std::endl;
        std::cout << "        : residual set to 0. " << std::endl;
    }
    return 0.;
};


// ------------------------------------------------------------
// Benchmark : main
// ------------------------------------------------------------

int main(int argc, char* argv[]) {

    std::cout << " ======================================================= \n"
                 " Benchmark test : DTMB 5512 captive motion \n"
                 " =======================================================" << std::endl;

    // -- Input

    double speed = atof(argv[1]);   // Ship forward speed
    double ak = 0.5*atof(argv[2]);  // Wave amplitude (m)
    double Tk = atof(argv[3]);      // Wave period (s)
    char* name = argv[4];     // Output director prefix name

    bool captive_test = true;      // fixed heave and pitch motions

    // -- System

    FrOffshoreSystem system;
    system.SetName(name);
    system.GetPathManager()->SetResourcesPath(std::string(RESOURCES_PATH));

    // -- Ocean
    auto ocean = system.GetEnvironment()->GetOcean();
    ocean->SetDensity(1000.);

    auto waveField = ocean->GetFreeSurface()->SetAiryRegularWaveField();
    waveField->SetWaveHeight(ak);
    waveField->SetWavePeriod(Tk);
    waveField->SetDirection(180., DEG, NWU, GOTO);

    system.GetEnvironment()->GetTimeRamp()->SetByTwoPoints(5., 0., 20., 1.);

    // -- Body

    auto body = system.NewBody();
    Position COGPosition(0., 0., 0.03); // 0.03
    body->SetPosition(Position(0., 0., 0.), NWU);
    body->AddMeshAsset(system.GetDataPath("DTMB5512_close.obj"));
    body->SetColor(Yellow);

    // -- Inertia

    double mass = 86.0;

    double Ixx = 1.98;
    double Iyy = 53.88;
    double Izz = 49.99;

    FrInertiaTensor InertiaTensor(mass, Ixx, Iyy, Izz, 0., 0., 0., COGPosition, NWU);
    body->SetInertiaTensor(InertiaTensor);

    // -- Hydrodynamics

    auto hdb = make_hydrodynamic_database(system.GetDataPath("DTMB5512.hdb5"));

    auto eqFrame = std::make_shared<FrEquilibriumFrame>(body.get(), false);
    eqFrame->SetPosition(Position(0., 0., 0.03), NWU);
    eqFrame->SetVelocityInWorld(Velocity(speed, 0., 0.), NWU);

    system.AddPhysicsItem(eqFrame);

    hdb->Map(0, body.get(), eqFrame);

    // -- Hydrostatic

    auto forceHst = make_linear_hydrostatic_force(hdb, body);

    // -- Radiation

    auto radiationModel = make_radiation_convolution_model(hdb, &system);
    radiationModel->SetImpulseResponseSize(body.get(), 50., 0.008);

    // -- Excitation

    auto excitationForce = make_linear_excitation_force(hdb, body);

    // -- Wave Drift force

    auto waveDriftForce = std::make_shared<FrWaveDriftForce>(hdb);
    body->AddExternalForce(waveDriftForce);

    // -- ITTC57

    auto lpp = 3.048;
    auto wettedSurfaceArea = 1.371;

    auto ct = ResidualITTC(speed);
    auto forceResistance = std::make_shared<FrITTCResistance>(lpp, wettedSurfaceArea, ct, 0.03);
    body->AddExternalForce(forceResistance);

    // -- Steady force

    auto forcePitch = std::make_shared<SteadyPitchTorque>();
    body->AddExternalForce(forcePitch);

    auto forceHeave = std::make_shared<SteadyHeaveForce>();
    body->AddExternalForce(forceHeave);

    // -- Carriage and fixation point

    auto shipNode = body->NewNode();
    shipNode->SetPositionInBody(body->GetCOG(NWU),NWU);
    shipNode->RotateAroundYInBody(90*DEG2RAD,NWU);
    shipNode->RotateAroundXInBody(90*DEG2RAD,NWU);

    auto carriage = make_carriage(&system, shipNode, captive_test);
    carriage->SetMotorFunction(FrConstantFunction(speed));

    auto dt = 0.008;

    system.SetTimeStep(dt);
    system.Initialize();
    system.DoAssembly();

    bool is_irrlicht = true;

    if (is_irrlicht) {
        system.RunInViewer(50., 10., false);
    } else {
        double time = 0.;
        while (time < 50.) {
            time += dt;
            system.AdvanceTo(time);
            std::cout << "time : " << time << std::endl;
        }
    }
    std::cout << "=============================== End ========================" << std::endl;
}
