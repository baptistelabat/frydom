//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H


#include <string>
#include <list>

//#include "frydom/core/common/FrObject.h"
#include "FrLoggable.h"


namespace frydom {

    class FrLogManager {

     public:

      FrLogManager();

      explicit FrLogManager(const std::string &log_folder);

      const std::string GetLogFolder() const;

      void Add(FrLoggable* obj);

      void Remove(FrLoggable* obj);

      void Initialize();

      void Update();

      void Finalize();


     private:
      std::string InitializeLogFolder();


     private:
      std::string m_log_folder;

      std::list<FrLoggable*> m_loggable_list;




    };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
