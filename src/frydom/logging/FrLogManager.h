//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H


#include <string>

namespace frydom {

    class FrLogManager {

     public:

      FrLogManager();

      explicit FrLogManager(const std::string&& log_folder);

      const std::string GetLogFolder() const;


     private:
      std::string InitializeLogFolder();


     private:
      std::string m_log_folder;


    };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
