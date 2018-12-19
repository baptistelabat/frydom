//
// Created by Lucas Letournel on 19/12/18.
//
#include "frydom/frydom.h"
#include <ctime>

using namespace frydom;


int main(int argc, char* argv[]) {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;
    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem_ system;

    {
        //
        // EXAMPLE 1: Atmosphere physical properties and wind field model definition
        //

        // The atmosphere object contains mainly the wind model along with the air physical properties
        auto Atmosphere = system.GetEnvironment()->GetAtmosphere();

        // Several physical properties of the Atmosphere can be adjusted: static pressure, density, temperature, salinity
        // and kinematic and dynamic viscosity. The following values are those set by default.
        Atmosphere->SetDensity(1.204);
        Atmosphere->SetTemperature(20.);
        Atmosphere->SetSalinity(0.);
        Atmosphere->SetPressure(0.);
        Atmosphere->SetKinematicViscosity(0.);
        Atmosphere->SetDynamicViscosity(0.);

        // A uniform field is set by default for the wind model. In order to set the wind characteristics, you need to
        // get first this uniform field.
        auto wind = Atmosphere->GetWind()->GetFieldUniform();

        // Several methods are available to set the wind characteristics : velocity amplitude, direction, etc.. In all
        // these methods, you must specify the direction convention used (GOT/COMEFROM). The frame convention is implicit
        // for those with a cardinal direction in their name. Otherwise, you have to specify the frame convention (NED/NWU)
        // as well.
        double windAmplitude = 6.; // in knots (KNOT), but can be in m/s (MS) or in km/h (KMH)
        Velocity windVelocity = WEST(fc) * windAmplitude;

        wind->Set(windVelocity, fc, dc);

        wind->Set(90, windAmplitude, DEG, KNOT, NWU, dc);

        wind->Set(WEST, windAmplitude, KNOT, dc);

        wind->SetWest(windAmplitude, KNOT, dc);

    }

    {
        //
        // EXAMPLE 2: Ocean physical properties, current model definition
        //

        // The ocean object contains more components than the atmosphere :
        // * sea water properties : static pressure, density, temperature, salinity, kinematic and dynamic viscosity,
        // * current field model,
        // * seabed object, with the bathymetry model,
        // * free surface, contains the tidal and the wave field models : for more details refer to demo_FreeSurface

        auto Ocean = system.GetEnvironment()->GetOcean();

        //--------------------------------------------------------------------------------------------------------------
        // The physical properties of the sea water can be adjusted in the same manner as those of the Atmosphere.
        // The following values are those set by default.
        Ocean->SetDensity(1027);
        Ocean->SetTemperature(10.);
        Ocean->SetSalinity(35.);
        Ocean->SetPressure(1.2030E-03);
        Ocean->SetKinematicViscosity(1.3604E-06);
        Ocean->SetDynamicViscosity(0.001397);

        //--------------------------------------------------------------------------------------------------------------
        // A uniform field is also set by default for the current model. In order to set the current characteristics,
        // you need to get first this uniform field.
        auto current = Ocean->GetCurrent()->GetFieldUniform();

        // Several methods are available to set the current characteristics : velocity amplitude, direction, etc.. In all
        // these methods, you must specify the direction convention used (GOT/COMEFROM). The frame convention is implicit
        // for those with a cardinal direction in their name. Otherwise, you have to specify the frame convention (NED/NWU)
        // as well.
        double currentAmplitude = 6.; // in knots (KNOT), but can be in m/s (MS) or in km/h (KMH)
        Velocity currentVelocity = EAST(fc) * currentAmplitude;

        current->Set(currentVelocity, fc, dc);

        current->Set(-90, currentAmplitude, DEG, KNOT, NWU, dc);

        current->Set(EAST, currentAmplitude, KNOT, dc);

        current->SetEast(currentAmplitude, KNOT, dc);

        //--------------------------------------------------------------------------------------------------------------
        // You can access to the seabed component, to set or get the mean bathymetry or manipulate the seabed asset.
        auto Seabed = Ocean->GetSeabed();

        // Be careful of the sign of the mean bathymetry, which is dependant of the frame convention specified:
        // * vertical axis up for NWU means a negative mean bathymetry,
        // * vertical axis down for NED means a positive man bathymetry.
        Seabed->SetBathymetry(-30,NWU);

        // You can hide the seabed asset with the following method. It however means you cannot access some methods
        // related to the bathymetry (Set, Get, etc.). The infinite depth condition is applied in this case for the wave
        // field computations (wave elevation, velocity, etc.)
        // Ocean->ShowSeabed(false);

        // An alternative method to run a simulation without a seabed is to enforce the infinite depth condition.
        // system.GetEnvironment()->GetOcean()->SetInfiniteDepth();

        // The following GetBathymetry() method will then return an error.
        // auto bathy = system.GetEnvironment()->GetOcean()->GetSeabed()->GetBathymetry(fc)

        // To manipulate the seabed grid asset, you first need to access it, through the seabed object.
        auto SeabedAsset = Seabed->GetSeabedGridAsset();

        // The seabed grid is defined here as a squared one ranging from -100m to 100m (in north and west
        // directions) with a 2m steps.
        SeabedAsset->SetGrid(-100., 100, 2, -100, 100, 2);

    }


    {
        //
        // EXAMPLE 3: Geographic services - conversion of cartesian coordinates to geographic coordinates and vice versa
        //

    }


    {
        //
        // EXAMPLE 4: Time ramp function, applied to the current, wind and wave field models
        //
        // This function is for now a linear function.
        // ^ value
        // |
        // + y1                  _________
        // |                    /
        // |                   /
        // |                  /
        // |                 /
        // |                /
        // |               /
        // |              /
        // + y0 _________/
        // |
        //  -------------+-------+--------
        //              t0       t1

        auto TimeRamp = system.GetEnvironment()->GetTimeRamp();

        // By default this function is deactivated.
        TimeRamp->Activate();

        // The function can be set increasing or decreasing. It has no effect on the min and max values : in both cases,
        // be sure to set y0<=y1;
        TimeRamp->SetDecrease();
        TimeRamp->SetIncrease();

        // You can set its starting (t0) and ending (t1) times as well as its initial (y0) and final (y1) values.
        double t0=0, t1=10, y0=0, y1=1;
        TimeRamp->SetMinTime(t0);
        TimeRamp->SetMaxTime(t1);
        TimeRamp->SetMinVal(y0);
        TimeRamp->SetMaxVal(y1);
        // The SetDuration() method modifies only the ending time, taking for reference the starting time : t1 = t0 + d
        TimeRamp->SetDuration(t1-t0);

        // To add this function to custom objects, you just need to be sure to call the Update() method, which stores
        // a cached value of the simulation time, by your object.
        // TimeRamp->Update(time);
        // The GetFunctionValue() method then returns the value of the time ramp function for the cached time.
    }

    // ------------------ Run ------------------ //

    // Justfor the sake of visualizing something
    system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularOptimWaveField(3.,6.,0.,RAD,fc,dc);
    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetGrid(-100., 100, 2, -100, 100, 2);
    system.GetEnvironment()->GetOcean()->GetFreeSurface()->GetFreeSurfaceGridAsset()->SetUpdateStep(1);

    // Don't forget to initialize the offshore system : it will initialize every physical objects and environmental
    // components it contains.
    system.Initialize();

    // Now you are ready to perform the simulation and you can watch its progression in the viewer. You can adjust
    // the time length of the simulation (here 30s) and the distance from the camera to the objectif (100m).
    // For saving snapshots of the simulation, just turn the boolean to true.
    system.RunInViewer(30, 100, false);

}

