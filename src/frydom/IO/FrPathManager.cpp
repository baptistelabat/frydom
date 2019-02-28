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
#include <cppfs/system.h>

#include <fmt/format.h>

#include "yaml-cpp/yaml.h"

#include "FrPathManager.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/force/FrForce.h"


namespace frydom{


    FrPathManager::FrPathManager() {

        ReadConfig();

    }

    void FrPathManager::ReadConfig() {

        cppfs::FilePath homePath = cppfs::system::homeDir();
        cppfs::FilePath configPath = homePath.resolve(".frydom");

        // Get the workspace directory path, located in the output_path in the FRyDOM config file.
        // TODO: change to JSON format
        try {
            YAML::Node data = YAML::LoadFile(configPath.path());  // TODO: throw exception if not found !

            m_outputPath = data["output_path"].as<std::string>();

            try {
                m_logFrameConvention = STRING2FRAME(data["frame_convention"].as<std::string>());
            } catch (YAML::BadConversion& err) {
                std::cout << " warning : frame convention must be NED or NWU" << std::endl;
            }
        }
        catch (YAML::BadFile& err){
            std::cout << " warning : no .frydom found in your home. Please add one with:" << std::endl;
            std::cout << " output_path: \"path/wanted\" \n frame_convention: NED " << std::endl;
            m_outputPath = ".";
            m_logFrameConvention = NED;
        }

    }

    void FrPathManager::Initialize() {

    }

    void FrPathManager::SetLogFrameConvention(FRAME_CONVENTION fc) {
        m_logFrameConvention = fc;
    }

    FRAME_CONVENTION FrPathManager::GetLogFrameConvention() const {
        return m_logFrameConvention;
    }

    void FrPathManager::SetLogOutputPath(std::string path) {
        m_outputPath = path;
    }

    std::string FrPathManager::GetLogOutputPath() const {
        return m_outputPath.path();
    }


    std::string FrPathManager::BuildSystemPath(FrOffshoreSystem *system){

        // Open the FRyDom workspace directory
        cppfs::FileHandle workspaceDir = cppfs::fs::open(m_outputPath.path());
        // Create directory if it does not yet exist
        if (!workspaceDir.isDirectory()) workspaceDir.createDirectory();

        // Create the run directory
        // TODO add the date to the run directory name, once TimeZone is refactored (need TimeZone to be brought back from Environment to the system?)
        m_runPath = m_outputPath.resolve(fmt::format("run_{}/",system->GetShortenUUID()));
        cppfs::FileHandle runDir = cppfs::fs::open(m_runPath.path());
        runDir.createDirectory();

        // Create the system directory
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));
        cppfs::FileHandle systemDir = cppfs::fs::open(systemPath.path());
        systemDir.createDirectory();

        // System log file path
        cppfs::FilePath systemLogPath = systemPath.resolve("system.csv");

        return systemLogPath.path();

    }

    std::string FrPathManager::BuildPhysicsItemPath(FrPhysicsItem *pi) {
        pi->SetLogFrameConvention(m_logFrameConvention);
        // Create the system directory
        auto system = pi->GetSystem();
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));

        // Create the path for the PI log
        auto relPILogPath = fmt::format("{0}_{1}_{2}/{0}.csv",pi->GetTypeName(),pi->GetName(),pi->GetShortenUUID());
        cppfs::FilePath PILogPath = systemPath.resolve(relPILogPath);

        // Create the directory for the PI logs
        cppfs::FilePath PIDirPath = PILogPath.directoryPath();
        auto PILogDir = cppfs::fs::open(PIDirPath.path());
        PILogDir.createDirectory();

        return PILogPath.path();
    }



    std::string FrPathManager::BuildBodyPath(FrBody *body){

        body->SetLogFrameConvention(m_logFrameConvention);
        // Create the system directory
        auto system = body->GetSystem();
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));

        // Create the path for the body log
        auto relBodyLogPath = fmt::format("{}_{}_{}/body.csv",body->GetTypeName(),body->GetName(),body->GetShortenUUID());
        cppfs::FilePath bodyLogPath = systemPath.resolve(relBodyLogPath);

        // Create the directory for the body logs
        cppfs::FilePath bodyDirPath = bodyLogPath.directoryPath();
        auto bodyLogDir = cppfs::fs::open(bodyDirPath.path());
        bodyLogDir.createDirectory();

        // Create the directory for the forces logs
        cppfs::FilePath forcesLogPath = bodyDirPath.resolve("Forces/");
        auto forcesLogDir = cppfs::fs::open(forcesLogPath.path());
        forcesLogDir.createDirectory();

        // Create the directory for the nodes logs
        cppfs::FilePath nodesLogPath = bodyDirPath.resolve("Nodes/");
        auto nodesLogDir = cppfs::fs::open(nodesLogPath.path());
        nodesLogDir.createDirectory();

        return bodyLogPath.path();
    }

    std::string FrPathManager::BuildForcePath(FrForce *force) {

        force->SetLogFrameConvention(m_logFrameConvention);

        auto body = force->GetBody();
        auto system = body->GetSystem();

        // Create the system directory
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));

        // Create the path for the body log
        auto relBodyLogPath = fmt::format("{}_{}_{}/",body->GetTypeName(),body->GetName(),body->GetShortenUUID());
        cppfs::FilePath bodyPath = systemPath.resolve(relBodyLogPath);

        // Create the directory for the body logs
        auto relForceLogPath = fmt::format("Forces/{}_{}.csv",force->GetTypeName(),force->GetShortenUUID());
        cppfs::FilePath forceLogPath = bodyPath.resolve(relForceLogPath);

        return forceLogPath.path();
    }

    std::string FrPathManager::BuildNodePath(FrNode *node) {

        node->SetLogFrameConvention(m_logFrameConvention);

        auto body = node->GetBody();
        auto system = body->GetSystem();

        // Create the system directory
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));

        // Create the path for the body log
        auto relBodyLogPath = fmt::format("{}_{}_{}/",body->GetTypeName(),body->GetName(),body->GetShortenUUID());
        cppfs::FilePath bodyPath = systemPath.resolve(relBodyLogPath);

        // Create the directory for the body logs
        auto relNodeLogPath = fmt::format("Nodes/{}_{}.csv",node->GetTypeName(),node->GetShortenUUID());
        cppfs::FilePath nodeLogPath = bodyPath.resolve(relNodeLogPath);

        return nodeLogPath.path();
    }


}// end namespace frydom
