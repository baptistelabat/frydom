//
// Created by frongere on 24/09/19.
//


//#include "FrLoggable.h"


namespace frydom {



//    FrLoggableBase::FrLoggableBase(const std::string &name) : m_name(name), m_log_this(true) {}
//
//    const std::string &FrLoggableBase::GetName() const {
//      return m_name;
//    }
//
//    void FrLoggableBase::LogThis(bool log) {
//      m_log_this = log;
//    }
//
//
//
//    template<class ParentType>
//    FrLoggable<ParentType>::FrLoggable(const std::string &name)
//        : FrTreeNode<ParentType>(),
//          FrLoggableBase(name) {}
//
//
//
//


//    template<class ParentType>
//    void FrLoggable<ParentType>::InitializeLog() {
//
////    if (IsLogged()) {
////
////      // Build the log path, create the directory and add a csv serializer to the hermes message
////      auto objPath = BuildPath(path);
////
////      // Add the fields to the hermes message
////      AddFields();
////
////      // Initializing message name and description
////      if (m_message->GetName().empty()) {
////        m_message->SetNameAndDescription(
////            fmt::format("{}_{}", GetTypeName(), GetShortenUUID()),
////            fmt::format("\"Message of a {}", GetTypeName()));
////      }
////
////      // Init the message
////      m_message->Initialize();
////      m_message->Send();
////
////      // Initialize the logs of the dependencies
////      InitializeLog_Dependencies(objPath);
////
////    }
//
//
//    }
//
//    template<class ParentType>
//    void FrLoggable<ParentType>::UpdateLog() {
//
////    if (IsLogged()) {
////      m_message->Serialize();
////      m_message->Send();
////    }
//
//    }
//
//    template<class ParentType>
//    void FrLoggable<ParentType>::FinalizeLog() {
//
//    }
//
//    template<class ParentType>
//    std::string FrLoggable<ParentType>::BuildPath(const std::string &rootPath) {
//
////    auto objPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());
////
////    auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));
////
////    // Add a serializer
////    m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));
////
////    return objPath;
//    }
//
//
//
////  void FrLoggable<ParentType>::SetPathManager(const std::shared_ptr<FrPathManager>& manager) {
////    m_pathManager = manager;
////  }
////
////  std::shared_ptr<FrPathManager> FrLoggable<ParentType>::GetPathManager() const {
////    return m_pathManager;
////  }
//
////  bool FrLoggable<ParentType>::IsLogged() {
//////    return m_isLogged;
////  }
//
////  FRAME_CONVENTION FrLoggable<ParentType>::GetLogFrameConvention() const {
////    return m_pathManager->GetLogFrameConvention();
////  }
//
////  void FrLoggable<ParentType>::SetLogged(bool isLogged) { m_isLogged = isLogged; }
//    template<class ParentType>
//    void FrLoggable<ParentType>::ClearMessage() {
////    m_message = std::make_unique<hermes::Message>();
//    }





}  // end namespace frydom
