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

#include "FrLogManager.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/force/FrForce.h"


namespace frydom{


    FrLogManager::FrLogManager() {

        ReadConfig();

    }

    void FrLogManager::ReadConfig() {

        cppfs::FilePath homePath = cppfs::system::homeDir();
        cppfs::FilePath configPath = homePath.resolve(".frydom/config");

        // Get the workspace directory path, located in the output_path in the FRyDOM config file.
        // TODO: change to JSON format
        YAML::Node data = YAML::LoadFile(configPath.path());  // TODO: throw exception if not found !

        m_outputPath = data["output_path"].as<std::string>();

        try {
            m_logFrameConvention = STRING2FRAME(data["frame_convention"].as<std::string>());
        } catch (YAML::BadConversion& err) {
            std::cout << " warning : frame convention must be NED or NWU" << std::endl;
        }

    }

    void FrLogManager::Initialize() {

    }


    std::string FrLogManager::NewSystemLog(FrOffshoreSystem* system){

        cppfs::FilePath workspacePath = m_outputPath;

        // Open the FRyDom workspace directory
        cppfs::FileHandle workspaceDir = cppfs::fs::open(workspacePath.path());
        // Create directory if it does not yet exist
        if (!workspaceDir.isDirectory()) workspaceDir.createDirectory();

        // Create the run directory
        // TODO add the date to the run directory name, once TimeZone is refactored (need TimeZone to be brought back from Environment to the system?)
        cppfs::FilePath runPath = workspacePath.resolve(fmt::format("run_{}/",system->GetShortenUUID()));
        cppfs::FileHandle runDir = cppfs::fs::open(runPath.path());
        runDir.createDirectory();

        // Create the system directory
        cppfs::FilePath systemPath = runPath.resolve(fmt::format("{}_{}/",system->GetTypeName(),system->GetShortenUUID()));
        cppfs::FileHandle systemDir = cppfs::fs::open(systemPath.path());
        systemDir.createDirectory();

        // System log file path
        cppfs::FilePath systemLogPath = systemPath.resolve("system.csv");

        // Set the path to system log file
        system->SetFilePath(systemLogPath.path());

        return systemLogPath.path();

    }



    std::string FrLogManager::NewBodyLog(FrBody* body){

        body->SetLogFrameConvention(m_logFrameConvention);
        // Get the path to the system log path
        cppfs::FilePath systemPath = body->GetSystem()->GetFilePath();
        // just keep the directory path, not the path to the system log file
        systemPath = systemPath.directoryPath();

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

        body->SetFilePath(bodyLogPath.path());

        return bodyLogPath.path();
    }

    std::string FrLogManager::NewForceLog(FrForce *force) {

        force->SetLogFrameConvention(m_logFrameConvention);
        // Get the path to the body log path
        cppfs::FilePath bodyPath = force->GetBody()->GetFilePath();
        // just keep the directory path, not the path to the system log file
        bodyPath = bodyPath.directoryPath();

        // Create the directory for the body logs
        auto relForceLogPath = fmt::format("Forces/{}_{}.csv",force->GetTypeName(),force->GetShortenUUID());
        cppfs::FilePath forceLogPath = bodyPath.resolve(relForceLogPath);

        force->SetFilePath(forceLogPath.path());

        return forceLogPath.path();
    }

    void FrLogManager::SetLogFrameConvention(FRAME_CONVENTION fc) {
        m_logFrameConvention = fc;
    }

    FRAME_CONVENTION FrLogManager::GetLogFrameConvention() const {
        return m_logFrameConvention;
    }

    void FrLogManager::SetLogOutputPath(std::string path) {
        m_outputPath = path;
    }

    std::string FrLogManager::GetLogOutputPath() const {
        return m_outputPath.path();
    }


}// end namespace frydom
