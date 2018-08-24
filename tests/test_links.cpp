//
// Created by frongere on 22/08/18.
//
#include <chrono/physics/ChLinkMotorRotationSpeed.h>

#include "frydom/frydom.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include <irrlicht.h>



using namespace frydom;

int main() {


    FrOffshoreSystem system;


    double density = 100.;

    // Platform
    // =========
    double platform_mass = 1e12; // Attention, si la masse est faible, le contact est tres elastique avec le colis...
    auto platform = std::make_shared<FrBox>(100, 50, 15, platform_mass, true, true);
    platform->SetBodyFixed(true);
//    platform->SetCollide(false);
    platform->SetPos(chrono::ChVector<>(0, 0, 0));


    system.AddBody(platform);

    double steelYoungModulus = 1e8;
    double steelNormalDamping = 1e10;

    auto platform_material_props = platform->GetMaterialSurfaceSMC();
//    std::cout << platform_material_props->GetKn() << std::endl;
    platform_material_props->SetKn(steelYoungModulus);  // Acier de construction
    platform_material_props->SetGn(steelNormalDamping);
    platform_material_props->young_modulus = steelYoungModulus;
    platform_material_props->restitution = 0.;



    // Crane turet
    // ===========
    double turet_mass = 1e3;
    auto turet = std::make_shared<FrBox>(10, 10, 5, turet_mass, false, true);
    system.AddBody(turet);

    turet->SetPos(chrono::ChVector<double>(-40, 0, 15));


    // On recupere des noeuds frydom
    auto platformNode = platform->CreateNode(-40, 0, 7.5);
    auto turetNode = turet->CreateNode(0, 0, -2.5);


    auto motor = make_motorRotationSpeed(platformNode, turetNode);
    system.AddLink(motor);

    double w = convert_frequency(3., RPM, RADS);
    motor->SetConstantSpeed(w);


    // colis box
    auto colis = std::make_shared<chrono::ChBodyEasyBox>(10, 10, 5, density, true, true, chrono::ChMaterialSurface::SMC);
    colis->SetPos(chrono::ChVector<double>(20, 0, 13));
    system.AddBody(colis);
//    colis->SetCollide(true);
    colis->SetMass(100e3);


    auto colis_material_props = colis->GetMaterialSurfaceSMC();
//    std::cout << platform_material_props->GetKn() << std::endl;
    colis_material_props->SetKn(steelYoungModulus);  // Acier de construction
    colis_material_props->SetGn(steelNormalDamping);
    colis_material_props->young_modulus = steelYoungModulus;
    colis_material_props->restitution = 0.;


    //
    system.SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);

    system.SetMinBounceSpeed(10.);
    system.SetMaxPenetrationRecoverySpeed(0.1);
//    system.UseMaterialProperties(false);
//    system.KRMmatricesLoad(1e10, 0., 1e9);

    // TODO: tester une mise en place de procedure de mise en equilibre statique (equilibre global et resolution des contraintes de liaison)


    auto app = FrIrrApp(system, 70.);

    app.SetTimestep(1e-2);
    app.Run();


    return 0;

}
