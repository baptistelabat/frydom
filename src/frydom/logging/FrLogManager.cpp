//
// Created by frongere on 25/09/19.
//

#include <algorithm>

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrConvention.h"
#include "FrLogManager.h"
#include "FrLoggable.h"


namespace frydom {


  FrLogManager::FrLogManager(FrOffshoreSystem *system) :
      m_log_folder(InitializeLogFolder()) {
    Add(system);
  }

  FrLogManager::FrLogManager(const std::string &log_folder, FrOffshoreSystem *system) :
      m_log_folder(log_folder) {
    Add(system);
  }

  const std::string FrLogManager::GetLogFolder() const {
    return m_log_folder;
  }

  void FrLogManager::Add(const std::shared_ptr<FrLoggableBase>& obj) {
    Add(obj.get());
  }

  void FrLogManager::Add(FrLoggableBase* obj) {
    if (!Has(obj)) m_loggable_list.push_back(obj);
  }

  void FrLogManager::Remove(const std::shared_ptr<FrLoggableBase>& obj) {
    Remove(obj.get());
  }

  void FrLogManager::Remove(FrLoggableBase* obj) {
    auto it = std::find(m_loggable_list.begin(), m_loggable_list.end(), obj);

    // Remove if present
    if (it != m_loggable_list.end()) {
      m_loggable_list.erase(it);
    }
  }

  bool FrLogManager::Has(FrLoggableBase* obj) const {
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
     *
     *
     *
     */


    return ".";

  }

  void FrLogManager::Initialize() {
    for (auto &obj : m_loggable_list) {
      obj->InitializeLog();
    }
  }

  void FrLogManager::StepFinalize() {
    for (auto &obj : m_loggable_list) {
      obj->StepFinalizeLog();
    }
  }

  void FrLogManager::SetLogFrameConvention(FRAME_CONVENTION fc) {
    for (auto &obj : m_loggable_list) {
      obj->SetLogFrameConvention(fc);
    }
  }

  unsigned int FrLogManager::GetNumberOfLoggables() const {
    return m_loggable_list.size();
  }

}  // end namespace frydom
