//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H


#include <string>
#include <vector>

//#include "frydom/core/common/FrObject.h"
#include "FrLoggable.h"


namespace frydom {

    class FrLogManager {

     public:

      FrLogManager();

      explicit FrLogManager(const std::string &log_folder);

      const std::string GetLogFolder() const;


     private:
      std::string InitializeLogFolder();


     private:
      std::string m_log_folder;

      std::vector<std::shared_ptr<FrLoggable>> m_loggable_list;


    };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
