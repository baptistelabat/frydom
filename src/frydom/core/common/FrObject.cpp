#include <utility>

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


#include "FrObject.h"
#include "frydom/utils/FrSerializerFactory.h"
#include "frydom/IO/FrPathManager.h"

namespace frydom {

    template<typename OffshoreSystemType>
    FrObject<OffshoreSystemType>::FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {

      m_message = std::make_unique<hermes::Message>();
    }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::InitializeLog(const std::string &path) {

      if (IsLogged()) {

        // Build the log path, create the directory and add a csv serializer to the hermes message
        auto objPath = BuildPath(path);

        // Add the fields to the hermes message
        AddFields();

        // Initializing message name and description
        if (m_message->GetName().empty()) {
          m_message->SetNameAndDescription(
              fmt::format("{}_{}", GetTypeName(), GetShortenUUID()),
              fmt::format("\"Message of a {}", GetTypeName()));
        }

        // Init the message
        m_message->Initialize();
        m_message->Send();

        // Initialize the logs of the dependencies
        InitializeLog_Dependencies(objPath);

      }

    }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::SendLog() {

      if (IsLogged()) {
        m_message->Serialize();
        m_message->Send();
      }

    }

    template<typename OffshoreSystemType>
    std::string FrObject<OffshoreSystemType>::BuildPath(const std::string &rootPath) {

      auto objPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());

      auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

      // Add a serializer
      m_message->AddSerializer(FrSerializerFactory<OffshoreSystemType>::instance().Create(this, logPath));

      return objPath;
    }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::StepFinalize() {
      SendLog();
    }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::SetPathManager(const std::shared_ptr<FrPathManager<OffshoreSystemType>> &manager) {
      m_pathManager = manager;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrPathManager<OffshoreSystemType>> FrObject<OffshoreSystemType>::GetPathManager() const {
      return m_pathManager;
    }

    template<typename OffshoreSystemType>
    bool FrObject<OffshoreSystemType>::IsLogged() { return m_isLogged; }

    template<typename OffshoreSystemType>
    FRAME_CONVENTION FrObject<OffshoreSystemType>::GetLogFrameConvention() const {
      return m_pathManager->GetLogFrameConvention();
    }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::SetLogged(bool isLogged) { m_isLogged = isLogged; }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::ClearMessage() { m_message = std::make_unique<hermes::Message>(); }

    template<typename OffshoreSystemType>
    std::string FrObject<OffshoreSystemType>::GetUUID() const { return m_UUID; }

    template<typename OffshoreSystemType>
    std::string FrObject<OffshoreSystemType>::GetShortenUUID() const { return m_UUID.substr(0, 5); }

    template<typename OffshoreSystemType>
    const char *FrObject<OffshoreSystemType>::GetName() const { return m_name.c_str(); }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::SetName(const char *myname) { m_name = myname; }

    template<typename OffshoreSystemType>
    std::string FrObject<OffshoreSystemType>::GetNameString() const { return m_name; }

    template<typename OffshoreSystemType>
    void FrObject<OffshoreSystemType>::SetNameString(const std::string &myname) { m_name = myname; }


}  // end namespace frydom
