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
#include <iomanip>

#include "yaml-cpp/yaml.h"

#include "FrPathManager.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrObject.h"
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

    void FrPathManager::Initialize(FrOffshoreSystem* system) {

        // Open the FRyDom workspace directory
        cppfs::FileHandle workspaceDir = cppfs::fs::open(m_outputPath.path());
        // Create directory if it does not yet exist
        if (!workspaceDir.isDirectory()) workspaceDir.createDirectory();

        // Create the run directory
        auto tt = std::chrono::system_clock::to_time_t ( std::chrono::system_clock::now() );
        std::stringstream ss;
        ss << std::put_time(std::localtime(&tt), "%Y-%m-%d_%HH%M");

        m_runPath = m_outputPath.resolve(fmt::format("{}_{}",system->GetName(),ss.str()));
        cppfs::FileHandle runDir = cppfs::fs::open(m_runPath.path());
        runDir.createDirectory();

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


    std::string FrPathManager::BuildPath(FrOffshoreSystem *system, std::string relPath){

        system->SetLogFrameConvention(m_logFrameConvention);

        // Create the system directory
        cppfs::FilePath systemPath = m_runPath.resolve(fmt::format("{}_{}/{}",
                system->GetTypeName(), system->GetShortenUUID(), relPath));

        cppfs::FileHandle systemDir = cppfs::fs::open(systemPath.directoryPath());
        systemDir.createDirectory();

        return systemPath.path();

    }

    std::string FrPathManager::BuildPath(FrPhysicsItem *pi, std::string relPath) {

        pi->SetLogFrameConvention(m_logFrameConvention);

        auto system = pi->GetSystem();

        // Create the path for the PI log
        auto relPILogPath = fmt::format("{}_{}/{}_{}_{}/{}",
                system->GetTypeName(),system->GetShortenUUID(),
                pi->GetTypeName(),pi->GetName(),pi->GetShortenUUID(),
                relPath);
        cppfs::FilePath PILogPath = m_runPath.resolve(relPILogPath);

        // Create the directory for the PI logs
        auto PILogDir = cppfs::fs::open(PILogPath.directoryPath());
        PILogDir.createDirectory();

        return PILogPath.path();

    }

    std::string FrPathManager::BuildPath(FrBody *body, std::string relPath){

        body->SetLogFrameConvention(m_logFrameConvention);

        auto system = body->GetSystem();

        // Create the path for the body log
        auto relBodyLogPath = fmt::format("{}_{}/{}_{}_{}/{}",
                system->GetTypeName(),system->GetShortenUUID(),
                body->GetTypeName(),body->GetName(),body->GetShortenUUID(),
                relPath);
        cppfs::FilePath bodyLogPath = m_runPath.resolve(relBodyLogPath);

        // Create the directory for the body logs
        auto bodyDir = cppfs::fs::open(bodyLogPath.directoryPath());
        bodyDir.createDirectory();

        return bodyLogPath.path();

    }

    std::string FrPathManager::BuildPath(FrForce *force, std::string relPath) {

        force->SetLogFrameConvention(m_logFrameConvention);

        auto body = force->GetBody();
        auto system = body->GetSystem();

        // Build the path for the force log
        auto relForceDirPath = fmt::format("{}_{}/{}_{}_{}/Forces/{}",
                system->GetTypeName(), system->GetShortenUUID(),
                body->GetTypeName(), body->GetName(), body->GetShortenUUID(),
                relPath);
        cppfs::FilePath forceLogPath = m_runPath.resolve(relForceDirPath);

        // Create the directory for the force logs, if not already created
        auto forceDir = cppfs::fs::open(forceLogPath.directoryPath());
        forceDir.createDirectory();

        return forceLogPath.path();

    }

    std::string FrPathManager::BuildPath(FrNode *node, std::string relPath) {

        node->SetLogFrameConvention(m_logFrameConvention);

        auto body = node->GetBody();
        auto system = body->GetSystem();

        // Build the path for the node log
        auto relNodeDirPath = fmt::format("{}_{}/{}_{}_{}/Nodes/{}",
                system->GetTypeName(), system->GetShortenUUID(),
                body->GetTypeName(), body->GetName(), body->GetShortenUUID(),
                relPath);
        cppfs::FilePath nodeLogPath = m_runPath.resolve(relNodeDirPath);

        // Create the directory for the force logs, if not already created
        auto nodeDir = cppfs::fs::open(nodeLogPath.directoryPath());
        nodeDir.createDirectory();

        return nodeLogPath.path();

    }


}// end namespace frydom
