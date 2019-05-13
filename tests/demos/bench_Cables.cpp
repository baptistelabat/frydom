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


bool ValidationResults (const std::shared_ptr<FrCatenaryLine> CatenaryLine, const std::shared_ptr<FrDynamicCable> DynamicCable,
        double unstrainedLength, unsigned int nbElements, double TargetLenghtRelError, double TargetTensionRelError) {

    //Analyse results
    double LengthRelError = 100.* abs(DynamicCable->GetStrainedLength() - CatenaryLine->GetStrainedLength()) / CatenaryLine->GetStrainedLength();

    double TensionRelError = 0;
    double ds = unstrainedLength/nbElements;
    double s = 0;
    for (int i=0; i<nbElements+1; i++) {

        Force diffTension = DynamicCable->GetTension(s,NWU) - CatenaryLine->GetTension(s,NWU);

        TensionRelError += 100.* diffTension.norm() / CatenaryLine->GetTension(s,NWU).norm();

        s += ds;

    }
    TensionRelError /= (nbElements+1);

    std::cout<<" Relative error on the strained length = " << LengthRelError << "%, should be < " << TargetLenghtRelError << "%" << std::endl;
    std::cout<<" Relative error on the tension = " << TensionRelError << "%, should be < " << TargetTensionRelError << "%" << std::endl;

    return (LengthRelError < TargetLenghtRelError) && (TensionRelError < TargetTensionRelError);

}


int main(int argc, char* argv[]) {

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
    Node1->SetPositionInBody(Position(-10., 0., -10.), NWU);
//    Node1->RotateAroundZInBody(MU_PI, NWU);
    auto Node2 = system.GetWorldBody()->NewNode();
    Node2->SetPositionInBody(Position(10., 0., 10.), NWU);
//    Node2->RotateAroundZInBody(MU_PI, NWU);

    Position temp = Node2->GetPositionInWorld(NWU) - Node1->GetPositionInWorld(NWU);
    double distanceBetweenNodes = temp.norm();


    // Line properties :
    bool elastic = true;                            //  non elastic catenary lines are only available for non strained lines
    auto cableProp = make_cable_properties();
    cableProp->SetDiameter(0.1);
    cableProp->SetEA(5e6);
    cableProp->SetLinearDensity(616.538);
    
    double unstrainedLength = distanceBetweenNodes*1.5;    //  unstrained length
    double rayleighDamping = 0.;                            //  Rayleigh damping
    unsigned int nbElements = 50;                           //  number of elements

    // Slack lines
    // ANCF cable
    auto DynamicCable = make_dynamic_cable(Node1, Node2, &system, cableProp, unstrainedLength, rayleighDamping, nbElements);

    // Catenary line

    auto CatenaryLine = make_catenary_line(Node1,Node2,&system, cableProp, elastic, unstrainedLength, AIR);


//    system.Visualize(20.,false);

    system.SetSolverWarmStarting(true);
    system.SetSolverMaxIterSpeed(200);
    system.SetSolverMaxIterStab(200);
    system.SetSolverForceTolerance(1e-13);

    system.SetTimeStep(0.01);

    system.GetStaticAnalysis()->SetNbIteration(20);
    system.GetStaticAnalysis()->SetNbSteps(50);

    std::cout<<"Solve statics for slack case"<<std::endl;
    system.SolveStaticWithRelaxation();
//    system.Visualize(20.,false);

    //Analyse results
    auto slack_valid = ValidationResults(CatenaryLine, DynamicCable, unstrainedLength, nbElements, 1.e-2, 4.25);

    // Clear the system
    system.Clear();


    // Taut lines
    unstrainedLength = distanceBetweenNodes*0.85;

    // ANCF cable
    auto DynamicCable2 = make_dynamic_cable(Node1, Node2, &system, cableProp, unstrainedLength, rayleighDamping, nbElements);

    // Catenary line

    auto CatenaryLine2 = make_catenary_line(Node1, Node2, &system, cableProp ,elastic, unstrainedLength, AIR);


    std::cout<<"Solve statics for taut case"<<std::endl;
//    system.GetStaticAnalysis()->SetNbIteration(50);
    system.SolveStaticWithRelaxation();
//    system.Visualize(20.,false);

    //Analyse results
    auto taut_valid = ValidationResults(CatenaryLine2, DynamicCable2, unstrainedLength, nbElements, 1.e-2, 0.25);


    return !(slack_valid && taut_valid);

}