//
// Created by frongere on 24/09/19.
//

#include "FrLoggable.h"


namespace frydom {

    FrLoggable::FrLoggable(const std::string&& name) : m_name(name) {  // TODO : Faire un FrNameManager::NewName(name)
      // Testing if the name is not already used !!! // TODO : l'unicite ne se fait pas sur les noms mais sur les path !!!!
    }

  const std::string& FrLoggable::GetName() const {
      return m_name;
    }

    void FrLoggable::InitializeLog(const std::string &path) {

//    if (IsLogged()) {
//
//      // Build the log path, create the directory and add a csv serializer to the hermes message
//      auto objPath = BuildPath(path);
//
//      // Add the fields to the hermes message
//      AddFields();
//
//      // Initializing message name and description
//      if (m_message->GetName().empty()) {
//        m_message->SetNameAndDescription(
//            fmt::format("{}_{}", GetTypeName(), GetShortenUUID()),
//            fmt::format("\"Message of a {}", GetTypeName()));
//      }
//
//      // Init the message
//      m_message->Initialize();
//      m_message->Send();
//
//      // Initialize the logs of the dependencies
//      InitializeLog_Dependencies(objPath);
//
//    }

    }

    void FrLoggable::SendLog() {

//    if (IsLogged()) {
//      m_message->Serialize();
//      m_message->Send();
//    }

    }

    std::string FrLoggable::BuildPath(const std::string &rootPath) {

//    auto objPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());
//
//    auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));
//
//    // Add a serializer
//    m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));
//
//    return objPath;
    }



//  void FrLoggable::SetPathManager(const std::shared_ptr<FrPathManager>& manager) {
//    m_pathManager = manager;
//  }
//
//  std::shared_ptr<FrPathManager> FrLoggable::GetPathManager() const {
//    return m_pathManager;
//  }

//  bool FrLoggable::IsLogged() {
////    return m_isLogged;
//  }

//  FRAME_CONVENTION FrLoggable::GetLogFrameConvention() const {
//    return m_pathManager->GetLogFrameConvention();
//  }

//  void FrLoggable::SetLogged(bool isLogged) { m_isLogged = isLogged; }

    void FrLoggable::ClearMessage() {
//    m_message = std::make_unique<hermes::Message>();
    }


}  // end namespace frydom
