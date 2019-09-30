//
// Created by frongere on 25/09/19.
//

#include "FrLogManager.h"


namespace frydom {


    FrLogManager::FrLogManager() : m_log_folder(InitializeLogFolder()) {
    }

    FrLogManager::FrLogManager(const std::string &log_folder) : FrLogManager() {
      m_log_folder = log_folder;
    }

    const std::string FrLogManager::GetLogFolder() const {
      return m_log_folder;
    }

    void FrLogManager::Add(FrLoggableBase* obj){
      auto it = std::find(m_loggable_list.begin(), m_loggable_list.end(), obj);

      // Don't add if already present !
      if (it == m_loggable_list.end()) {
        m_loggable_list.push_back(obj);
      }
    }

    void FrLogManager::Remove(FrLoggableBase* obj) {
      auto it = std::find(m_loggable_list.begin(), m_loggable_list.end(), obj);

      // Remove if present
      if (it != m_loggable_list.end()) {
        m_loggable_list.erase(it);
      }
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
      for (auto& obj : m_loggable_list) {
        obj->InitializeLog();
      }
    }

    void FrLogManager::Update() {
      for (auto& obj : m_loggable_list) {
        obj->UpdateLog();
      }
    }

    void FrLogManager::Finalize() {
      for (auto& obj : m_loggable_list) {
        obj->FinalizeLog();
      }
    }

}  // end namespace frydom
