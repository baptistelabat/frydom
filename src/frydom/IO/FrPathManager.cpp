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

#include <nlohmann/json.hpp>
using json = nlohmann::json; // for convenience

#include "FrPathManager.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/force/FrForce.h"

namespace frydom{


    FrPathManager::FrPathManager() :
      m_logFrameConvention(NWU) {

        ReadConfig();

    }

    void FrPathManager::ReadConfig() {

        cppfs::FilePath homePath = cppfs::system::homeDir();
        cppfs::FilePath configPath = homePath.resolve(".frydom.json");

        // Get the workspace directory path, located in the output_path in the FRyDOM config file.
        try {

            // Reading the json input file.
            std::ifstream ifs(configPath.path());
            json data = json::parse(ifs);

            auto log = data["logs"];

            m_outputPath = log["output_path"].get<json::string_t>();

            try {
                m_logFrameConvention = STRING2FRAME(log["frame_convention"].get<json::string_t>());

            } catch (json::parse_error& err) {
                std::cout << " warning : frame convention must be NED or NWU" << std::endl;
            }
        }
        catch (json::parse_error& err){
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

        m_projectPath = m_outputPath.resolve(fmt::format("{}_{}",system->GetName(),ss.str()));
        cppfs::FileHandle runDir = cppfs::fs::open(m_projectPath.path());
        runDir.createDirectory();

    }

    void FrPathManager::SetRunPath(std::string relPath){

        m_runPath = m_projectPath.resolve(relPath);
        cppfs::FileHandle runDir = cppfs::fs::open(m_runPath.path());
        runDir.createDirectory();

    }

    std::string FrPathManager::GetRunPath() const {
        return m_runPath.path();
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

    std::string FrPathManager::BuildPath(const std::string& rootPath, const std::string& relPath) const {
        cppfs::FilePath filePath = m_runPath.resolve(fmt::format("{}/{}", rootPath, relPath));
        cppfs::FileHandle fileDir = cppfs::fs::open(filePath.directoryPath());
        fileDir.createDirectory();
        return filePath.path();
    }

    std::string FrPathManager::BuildPath(const std::string& absPath) const {
        cppfs::FilePath filePath = m_runPath.resolve(fmt::format("{}", absPath));
        cppfs::FileHandle fileDir = cppfs::fs::open(filePath.directoryPath());
        fileDir.createDirectory();
        return filePath.path();
    }

    void FrPathManager::SetResourcesPath(std::string absPath) {
        m_resourcesPath.setPath(absPath);
    }

    std::string FrPathManager::GetDataPath(const std::string& relPath) const {
        //cppfs::FilePath resources_path(std::string(RESOURCES_PATH));
        cppfs::FilePath filePath = m_resourcesPath.resolve(fmt::format("{}", relPath));
        return filePath.path();
    }

}// end namespace frydom
