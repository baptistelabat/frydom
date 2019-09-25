//
// Created by frongere on 25/09/19.
//

#include "FrLogManager.h"


namespace frydom {


    FrLogManager::FrLogManager() : m_log_folder(InitializeLogFolder()){
    }

    FrLogManager::FrLogManager(const std::string&& log_folder) : FrLogManager() {
      m_log_folder = log_folder;
    }

    const std::string FrLogManager::GetLogFolder() const {
      return m_log_folder;
    }

    std::string FrLogManager::InitializeLogFolder() {
      // Looking for frydom configuration file



    }

}  // end namespace frydom
