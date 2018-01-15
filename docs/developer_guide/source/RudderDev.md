Rudder {#rudder}
======

# Presentation {#presentation}

The [FrRudder](@ref frydom::FrRudder) is part of the [FrShip](@ref frydom::FrShip) model. It is subdived into two main object :

- [FrRudder](@ref frydom::FrRudder)
- [FrRudderForce](@ref frydom::FrRudderForce)

Additional model can be used to take into account the perturbation due to the presence of the propeller on the rudder.


# Instanciation {#instanciation}

In the following an example of instanciation of a new rudder system is presented

~~~{.cpp}
    // Define a new rudder model with:
    // - side area
    // - height
    // - stock position
    auto rudder = std::make_shared<FrRudder>(5., 1., 0.3);
    
    // Set parameters 
    rudder->SetMasAngleSpeed(0.01);
    rudder->SetMaxAngle(1.4);
    
    
    // Define the force from lift and drag coefficients
    rudder->MakeForce("PolarRudderCoeff.yml")
    
    // Define position and rotation according to the body coordinate system
    auto position = ChVector<>(10., 0., 0.);
    rudder->SetPos(position);
    auto rotation = euler_to_quat(0., 0., 0., CARDAN, DEG);
    rudder->SetRot(rotation);
    
    // Add the rudder to the ship model
    ship->AddRudder(rudder);

~~~

# Control commands {#control_cmd}

For the rudder, only the azimuth angle can be control. The command of the angle is set up by the following command

~~~{.cpp}
    // Setup of the new control angle command
    ship->GetRudder(0)->SetCtrlAngle(0.1);
~~~

The rudder model will evolve to this new state following the rudder dynamic.

To force the angle of the rudder, the following command can be used.

~~~{.cpp}
    ship->GetRudder(0)->SetAngle(0.1);
~~~

# Update procedure {#update}

As part of the ship model, the rudder integrates an [UpdateState](@ref frydom::FrRudder::UpdateState) and an [UpdateTime](@ref frydom::FrRudder::UpdateTime) methods to update the object in time.

- the [UpdateTime](@ref frydom::FrRudder::UpdateTime) method updates @ref chrono::ChTime and time dependant functions.

- the [UpdateState](@ref frydom::FrRudder::UpdatetState) method updates all state of the rudder. The new position angle is updated during this step such as the water inflow velocity and lift and drag coefficients are reevaluate from polar coefficient table.

The [FrRudderForce](@ref frydom::FrRudderForce) model is not updated furing this procedure since force are part of an external update procedure linked to chrono.

# Destruction {#destruction}

Lik between the different part of the rudder are based on smart pointer, which integrates automatic deallocation if the object is no longer referenced. A ruddeer is deleted from the ship model from the [RemoveRudder](@ref frydom::FrShip::RemoveRudder) method.
