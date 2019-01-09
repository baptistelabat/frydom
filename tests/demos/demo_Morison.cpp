//
// Created by Lucas Letournel on 04/01/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    /**
     *
     */

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION
    fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem_ system;

    // ------------------ Wave Field ------------------ //

    auto FreeSurface = system.GetEnvironment()->GetOcean()->GetFreeSurface();

    // To manipulate the free surface grid asset, you first need to access it, through the free surface object.
    auto FSAsset = FreeSurface->GetFreeSurfaceGridAsset();

    // The free surface grid is defined here as a squared one ranging from -100m to 100m (in north and west
    // directions) with a 2m steps.
    FSAsset->SetGrid(-100., 100, 2, -100, 100, 2);

    // You have to specify if you want the free surface asset to be updated during the simulation. By default, the
    // update is not activated.
    FSAsset->SetUpdateStep(10);

    // The Airy regular wave parameters are its height, period and direction.
    double waveHeight = 2.;    double wavePeriod = 2.*M_PI;
    Direction waveDirection = Direction(SOUTH(fc));

    auto waveField = FreeSurface->SetAiryRegularOptimWaveField(waveHeight, wavePeriod, waveDirection, fc, dc);

    // ------------------ Cylinder ------------------ //

    // Create the cylinder
    auto cylinder = system.NewBody();
    cylinder->SetName("Cylinder");
    double radius = 3, height = 30, mass = 1000;
    makeItCylinder(cylinder,radius, height, mass);
    cylinder->SetColor(LightGoldenRodYellow);
    cylinder->SetPosition(Position(0.,0.,-0.5*height),fc);

    cylinder->SetBodyFixed(true);

    // ------------------ Morison Model ------------------ //

    // Several ways exist to add a Morison model to a body. Remember that a Morison model is a composition of Morison
    // elements (using a composition pattern). Morison elements can be added simply to a Morison model. Note that you
    // can also provide a Morison Force with a Morison element, if you got only one element.
    auto MorisonModel = make_MorisonModel(cylinder.get());

    // Define the added mass and drag Morison Coefficients.
    MorisonCoeff AddedMassCoeff(0.5,0.7);
    MorisonCoeff DragCoeff(1.5,1.7);
    double frictionCoeff = 0.1;

    // Add an element, with parameters corresponding to the cylinder.
    MorisonModel->AddElement(FrFrame_(), height, 2.*radius, AddedMassCoeff, DragCoeff, frictionCoeff);

    // Instantiate a Morison Force, using a Morison model
    auto MorisonForce = std::make_shared<FrMorisonForce_>(MorisonModel);

    MorisonForce->SetIsForceAsset(true);

    // Don't forget to add the Morison force to the body !
    cylinder->AddExternalForce(MorisonForce);



    // ------------------ Run ------------------ //

    // You can change the dynamical simulation time step using.
    system.SetTimeStep(0.04);

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 15) and the distance from the camera to the objectif (75m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(150, 155, false);


}