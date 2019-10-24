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
#include "FrPathManager.h"
#include "FrLogManager.h"
#include "FrLoggable.h"


namespace frydom {

  FrLogManager::FrLogManager(FrOffshoreSystem *system) :
      m_log_folder(InitializeLogFolder()),
      m_log_CSV(true) {
    Add(system);

//    m_serializers.push_back(std::make_unique<hermes::CSVSerializer>(m_log_folder + "file.csv"));
    // FIXME : lorsqu'on change le log_folder, il faut que ce soit vu par le serializer...
    // FIXME : on a devoir passer en shared dans hermes non ?
  }

  FrLogManager::FrLogManager(const std::string &log_folder, FrOffshoreSystem *system) :
      m_log_folder(log_folder) {
    Add(system);
  }

  FrOffshoreSystem *FrLogManager::GetSystem() const {
    return dynamic_cast<FrOffshoreSystem *>(m_loggable_list.front());
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
    // Looking for frydom configuration file
    /*
     * TODO :
     * On regarde en premier si on a un .frydom dans le repertoire courant --> avoir du frydom init a la git ?
     * On devrait pouvoir faire un frydom init.
     *
     * Si on a un .frydom dans le home, on voit si on peut charger un workspace
     *
     * Tout cela sera vraiment utile quand FRyDoM sera installable...
     * On fournira un utilitaire frydom_update qui permettra de mettre a jour l'install de frydom avec la dernière version...
     *
     */

    std::cerr << "Mettre en place la determination du dossier de log" << std::endl;

    return "./";

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
