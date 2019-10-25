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

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/utils/FrFileSystem.h"
#include "frydom/utils/GitSHA1.h"

#include "FrPathManager.h"
#include "FrLogManager.h"
#include "FrLoggable.h"


#include <nlohmann/json.hpp>
#include <ctime>
#include <chrono>


#define META_FILE_NAME "meta.json"
#define DATE_FOLDER_FORMAT "%Y-%m-%d_%Hh%Mm%Ss"


using json = nlohmann::json;

namespace frydom {

  FrLogManager::FrLogManager(FrOffshoreSystem *system) :
      m_log_CSV(true),
      m_system(system) {

    Add(system);
    m_log_folder = InitializeLogFolder();

//    m_serializers.push_back(std::make_unique<hermes::CSVSerializer>(m_log_folder + "file.csv"));
    // FIXME : lorsqu'on change le log_folder, il faut que ce soit vu par le serializer...
    // FIXME : on a devoir passer en shared dans hermes non ?
  }

  FrLogManager::FrLogManager(const std::string &log_folder, FrOffshoreSystem *system) :
      m_log_folder(log_folder) {
    Add(system);
  }

  FrOffshoreSystem *FrLogManager::GetSystem() const {
    return m_system;
  }

  const std::string &FrLogManager::GetLogFolder() const {
    return m_log_folder;
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
     * 1 - looking for a local frydom_config file
     * 2 - looking for a frydom_config file into home
     * 3 - taking the current directory;
     */

    std::string log_folder;

    // Looking for a file .frydom_config into the CURRENT directory (local project config)
    std::string local_config_file = FrFileSystem::join({FrFileSystem::cwd(), ".frydom_config"});
    if (FrFileSystem::exists(local_config_file)) {
      log_folder = LogFolderFromFrydomConfigFile(local_config_file);
    }

    // Looking for a file .frydom_config into the HOME directory (session config))
    std::string session_file = FrFileSystem::join({FrFileSystem::get_home(), ".frydom_config"});
    if (FrFileSystem::exists(session_file) && log_folder.empty()) {
      log_folder = LogFolderFromFrydomConfigFile(session_file);
    }

    if (log_folder.empty()) {
      log_folder = FrFileSystem::cwd();
    }

    log_folder = FrFileSystem::join({log_folder, GetDateFolder()});
    FrFileSystem::mkdir(log_folder);

    std::cout << "Logging into: " << log_folder << std::endl;

    CreateMetadataFile(log_folder);

    exit(EXIT_SUCCESS);

    return log_folder;
  }

  void FrLogManager::CreateMetadataFile(const std::string &log_folder) {

    json j;

    j["date"] = now();
    j["username"] = FrFileSystem::get_login();
    j["hostname"] = FrFileSystem::get_hotname();
    j["project_name"] = GetSystem()->GetName();
    j["frydom_git_revision"] = GetGitSHA1();

    // version frydom -> mettre le hash de commit



    std::ofstream file;
    file.open(FrFileSystem::join({log_folder, META_FILE_NAME}), std::ios::trunc);
    file << j.dump(2);
    file.close();

  }

  std::string FrLogManager::now() {
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string now_str = std::ctime(&now);
    return now_str.substr(0, now_str.size() - 1); // Removing last char that is an end line...
  }

  std::string FrLogManager::LogFolderFromFrydomConfigFile(const std::string &path_to_config_file) {

    if (!FrFileSystem::exists(path_to_config_file)) exit(EXIT_FAILURE);

    std::ifstream ifs(path_to_config_file);

    json json_obj = json::parse(ifs);

    std::string log_folder = json_obj["log_folder"].get<std::string>();

    if (!FrFileSystem::isabs(log_folder)) return "";

    return log_folder;

  }

  std::string FrLogManager::GetDateFolder() {

    time_t temps;
    struct tm datetime;
    char format[32];

    time(&temps);
    datetime = *localtime(&temps);

    strftime(format, 32, DATE_FOLDER_FORMAT, &datetime);

    return format;
  }


  void FrLogManager::Initialize() { // TODO : retirer la necessite d'avoir cette methode friend de FrLoggableBase

    for (auto &obj : m_loggable_list) {

      if (!obj->IsLogged()) continue;

      obj->DefineLogMessages();

      if (m_log_CSV) {

        // Adding run information
        std::string system_folder = m_log_folder + GetSystem()->GetTreePath();
        FrFileSystem::mkdir(system_folder);

        // Adding a CSV serializer to messages
        for (auto &message : obj->m_messages) { // TODO : ajouter un iterateur de message sur FrLoggableBase

          // Building the message folder
          std::string message_folder = m_log_folder + obj->GetTreePath();
          FrFileSystem::mkdir(message_folder);

          std::string csv_file = message_folder + message->GetName() + ".csv";

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

  void FrLogManager::LogCSV(bool val) {
    m_log_CSV = val;
  }

  unsigned int FrLogManager::GetNumberOfLoggables() const {
    return m_loggable_list.size();
  }

}  // end namespace frydom
