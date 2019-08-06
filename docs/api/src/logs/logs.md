Logs architecture and philosophy {#logs}
================================

Logs philosophy
---------------

Logs in FRyDoM are possible for all classes derived from FrObject (almost all classes in FRyDoM). However the logs
architecture and data are defined only for the main classes : FrOffshoreSystem, FrBody, FrPhysicsItem, FrLinks, FrForce, etc.
No logs have been defined for the Environment components yet, since they are all stationary.

Logs outputs are .csv file format, with a two lines header (name and unit of the variables). Logs are organised in the
same manner as in-code. Objects have their logs in their own folder, except for forces and nodes which logs are all 
grouped in their body log folder.

Logs architecture
-----------------

Below is the example of the log architecture for the demo_Hub_Installation dynamic simulation.


![Log Architecture](LogTree.png "Log Architecture")

A similar architecture can be found for the static analysis.

Config file
-----------

A .frydom.json config file, in JSON, to be placed in your home, is used to specify the location of the logs root folder. The 
frame convention to be used in the logs (NED or NWU) is also defined in this config file. If no config file is 
provided, the logs are in NED and located in the execution folder.


    {
        "logs": {
            "output_path": "/path/to/logs/",
            "frame_convention": "NED"
        }
    }

Adding logs
-----------

To add logs to new classes, you need to implement the following steps:

- derive from FrObject
    
    
    class FrDummy : public FrObject {}

- activate the logs either at the class scope or the instance scope, using the method

        
    FrObject::SetLogged() 

- initialize the logs : path, data, etc. A predefined FrObject::InitializeLog() can be used, but it must be called
during the initialize log loop (started by FrOffshoreSystem::InitializeLog(), called by FrOffshoreSystem:Initialize())
if your class composes with other ones which needs to be logged, just override FrObject::InitializeLogDependencies() and
implement it (see FrBody::InitializeLogDependencies() for example). You can also override FrObject::BuildPath() method
if you have specific needs for the log path. Finally override the FrObject::AddFields() method to add data to the logs, 
using the following syntax (taken from the body AddField() method).
    

    m_message->AddField<double>("time", "s", "Current time of the simulation",[this]() { return m_system->GetTime(); });
    m_message->AddField<Eigen::Matrix<double, 3, 1>>("Position","m", fmt::format("body position in the world reference frame in {}", GetLogFrameConvention()),
                                                     [this]() {return GetPosition(GetLogFrameConvention());});
                                                     
You can find in this example the reference to the frame convention for the logs, as mentioned in the config file entry,
given by

    
    GetLogFrameConvention()