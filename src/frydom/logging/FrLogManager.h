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


#ifndef FRYDOM_FRLOGMANAGER_H
#define FRYDOM_FRLOGMANAGER_H

#include <memory>

#include <string>
#include <list>


namespace frydom {

  // Forward declaration
  class FrLoggableBase;

  class FrOffshoreSystem;


  class FrLogManager {

   public:

    explicit FrLogManager(FrOffshoreSystem *system);

    ~FrLogManager();

    FrOffshoreSystem *GetSystem() const;

    void Add(const std::shared_ptr<FrLoggableBase> &obj);

    void Add(FrLoggableBase *obj);

    void Remove(const std::shared_ptr<FrLoggableBase> &obj);

    void Remove(FrLoggableBase *obj);

    unsigned int GetNumberOfLoggables() const;

    virtual void Initialize();

    void StepFinalize();

    void SetLogFrameConvention(FRAME_CONVENTION fc);

    void NoCSVLlog();  // TODO: permettre de ne pas logger en CSV... -> perf !

    using LoggableList = std::list<FrLoggableBase *>;
    using LoggableIter = LoggableList::iterator;

    LoggableIter begin() {
      return m_loggable_list.begin();
    };

    LoggableIter end() {
      return m_loggable_list.end();
    }

   private:
    bool Has(FrLoggableBase *obj) const;

//    std::string InitializeLogFolder();

    static std::string LogFolderFromFrydomConfigFile(const std::string &path_to_config_file);

    static std::string GetDateFolder();

    void WriteMetaDataFile(const std::string &log_folder);

    static std::string now();

   private:
    std::string m_log_folder;

    LoggableList m_loggable_list;

    FrOffshoreSystem *m_system;

    bool m_log_CSV;

  };


}  // end namespace frydom



#endif //FRYDOM_FRLOGMANAGER_H
