//
// Created by frongere on 25/09/19.
//

#include "FrLogManager.h"


namespace frydom {


    FrLogManager::FrLogManager() : m_log_folder(InitializeLogFolder()) {
    }

    FrLogManager::FrLogManager(const std::string&& log_folder) : FrLogManager() {
      m_log_folder = log_folder;
    }

    const std::string FrLogManager::GetLogFolder() const {
      return m_log_folder;
    }

    std::string FrLogManager::InitializeLogFolder() {
      // Looking for frydom configuration file
      /*
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

}  // end namespace frydom
