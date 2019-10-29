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

#include <algorithm>
#include <nlohmann/json.hpp>
#include <ctime>
#include <chrono>

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/FrPlatform.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/version.h"

#include "FrPathManager.h"
#include "FrLogManager.h"
#include "FrLoggable.h"
#include "FrEventLogger.h"


#define META_FILE_NAME "meta.json"
#define DATE_FOLDER_FORMAT "%Y-%m-%d_%Hh%Mm%Ss"
#define CONFIG_FILE_NAME ".frydom_config"


using json = nlohmann::json;

namespace frydom {

  FrLogManager::FrLogManager(FrOffshoreSystem *system) :
      m_log_CSV(true),
      m_system(system) { // FIXME : Du coup LogManager devrait etre un TreeNode...

    Add(system);
    m_log_folder = InitializeLogFolder();

    // Event Logger initialization
    event_logger::init(system, "FRYDOM", "frydom_event.log");

    event_logger::info("LogManager", "log manager", "Results will be logged into {}", m_log_folder);

  }

  FrLogManager::~FrLogManager() = default;

  FrOffshoreSystem *FrLogManager::GetSystem() const {
    return m_system;
  }

  void FrLogManager::Add(const std::shared_ptr<FrLoggableBase> &obj) {
    Add(obj.get());
  }

  void FrLogManager::Add(FrLoggableBase *obj) {
    if (!Has(obj)) m_loggable_list.push_back(obj);
  }

  void FrLogManager::Remove(const std::shared_ptr<FrLoggableBase> &obj) {
    Remove(obj.get());
  }

  void FrLogManager::Remove(FrLoggableBase *obj) {
    auto it = std::find(m_loggable_list.begin(), m_loggable_list.end(), obj);

    // Remove if present
    if (it != m_loggable_list.end()) {
      m_loggable_list.erase(it);
    }
  }

  bool FrLogManager::Has(FrLoggableBase *obj) const {
    return (std::find(m_loggable_list.begin(), m_loggable_list.end(), obj) != m_loggable_list.end());
  }

  std::string FrLogManager::InitializeLogFolder() {

    /*
     * 1 - looking for a local CONFIG_FILE_NAME file
     * 2 - looking for a CONFIG_FILE_NAME file into home
     * 3 - taking the current directory;
     */

    std::string log_folder;

    // Looking for a file CONFIG_FILE_NAME into the CURRENT directory (local project config)
    std::string local_config_file = FrFileSystem::join({FrFileSystem::cwd(), CONFIG_FILE_NAME});
    if (FrFileSystem::exists(local_config_file)) {
      log_folder = LogFolderFromFrydomConfigFile(local_config_file);
    }

    // Looking for a file CONFIG_FILE_NAME into the HOME directory (session config))
    std::string session_file = FrFileSystem::join({FrFileSystem::get_home(), CONFIG_FILE_NAME});
    if (FrFileSystem::exists(session_file) && log_folder.empty()) {
      log_folder = LogFolderFromFrydomConfigFile(session_file);
    }

    if (log_folder.empty()) {
      log_folder = FrFileSystem::cwd();
    }

    log_folder = FrFileSystem::join({log_folder, GetDateFolder()});
    FrFileSystem::mkdir(log_folder);

//    std::cout << "Logging into: " << log_folder << std::endl;

    CreateMetaDataFile(log_folder);

    return log_folder;
  }

  void FrLogManager::CreateMetaDataFile(const std::string &log_folder) {

    json j;

    j["date"] = now();
    j["username@hostname"] = FrFileSystem::get_login() + "@" + FrFileSystem::get_hostname();
    j["project_name"] = GetSystem()->GetName();
    j["frydom_git_revision"] = GetGitSHA1();
    j["platform"] = GetPlatformName();
    j["frydom_flavor"] = GetFrydomFlavor();


    std::ofstream file;
    file.open(FrFileSystem::join({log_folder, META_FILE_NAME}), std::ios::trunc);
    file << j.dump(2);
    file.close();

  }

  std::string FrLogManager::now() { // TODO : voir a faire avec fmt...
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string now_str = std::ctime(&now);
    return now_str.substr(0, now_str.size() - 1); // Removing last char that is an end line...
  }

  std::string FrLogManager::LogFolderFromFrydomConfigFile(const std::string &path_to_config_file) {

    if (!FrFileSystem::exists(path_to_config_file)) exit(EXIT_FAILURE);

    std::ifstream ifs(path_to_config_file);

    json json_obj = json::parse(ifs);

    // FIXME : si la cle n'est pas trouve, cela ne doit pas crasher !!!
    std::string log_folder = json_obj["log_folder"].get<std::string>();

    // FIXME : on autorise pas les chemins relatif mais on devrait. Demande reflexion...
    if (!FrFileSystem::isabs(log_folder)) return "";

    return log_folder;

  }

  std::string FrLogManager::GetDateFolder() {

    // TODO : utiliser fmt pour ce formatage

    time_t temps;
    struct tm datetime;
    char format[32];

    time(&temps);
    datetime = *localtime(&temps);

    strftime(format, 32, DATE_FOLDER_FORMAT, &datetime);

    return format;
  }

  void FrLogManager::Initialize() { // TODO : retirer la necessite d'avoir cette methode friend de FrLoggableBase

    auto path_manager = GetSystem()->GetPathManager();

    for (auto &obj : m_loggable_list) {

      if (!obj->IsLogged()) continue;


//      std::string type_name(GetTypeNameId(*obj));


      obj->DefineLogMessages();

      if (m_log_CSV) {

        // Adding a CSV serializer to messages
        for (auto &message : obj->m_messages) { // TODO : ajouter un iterateur de message sur FrLoggableBase

          // Building the message folder
          std::string message_folder = FrFileSystem::join({m_log_folder, obj->GetTreePath()});
          FrFileSystem::mkdir(message_folder);

          std::string csv_file = FrFileSystem::join({message_folder, message->GetName() + ".csv"}); // FIXME : ajouter les infos de type...

          message->AddSerializer(new hermes::CSVSerializer(csv_file));
        }

      }

      obj->InitializeLogMessages();

      obj->SendLogMessages();

    }
  }

  void FrLogManager::StepFinalize() {
    for (auto &obj : m_loggable_list) {
      if (!obj->IsLogged()) continue;
      obj->StepFinalizeLog();
    }
  }

  void FrLogManager::SetLogFrameConvention(FRAME_CONVENTION fc) {
    for (auto &obj : m_loggable_list) {
      obj->SetLogFrameConvention(fc);
    }
  }

  void FrLogManager::NoCSVLlog() { m_log_CSV = false; }

  unsigned int FrLogManager::GetNumberOfLoggables() const {
    return m_loggable_list.size();
  }

}  // end namespace frydom
