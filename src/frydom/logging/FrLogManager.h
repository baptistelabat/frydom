//
// Created by frongere on 25/09/19.
//

#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H

#include <memory>

#include <string>
#include <list>


namespace frydom {

  // Forward declaration
  class FrLoggableBase;


  class FrLogManager {

   public:

    FrLogManager();

    explicit FrLogManager(const std::string &log_folder);

    const std::string GetLogFolder() const;

    void Add(std::shared_ptr<FrLoggableBase> obj);

    void Remove(std::shared_ptr<FrLoggableBase> obj);

    void Initialize();

    void Update();

    void Finalize();


   private:
    bool Has(std::shared_ptr<FrLoggableBase> obj) const;

    std::string InitializeLogFolder();


   private:
    std::string m_log_folder;

    std::list<std::shared_ptr<FrLoggableBase>> m_loggable_list;


  };

}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
