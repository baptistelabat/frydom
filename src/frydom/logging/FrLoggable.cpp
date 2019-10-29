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


#include "FrLoggable.h"
#include "FrEventLogger.h"


namespace frydom {

  FrLoggableBase::FrLoggableBase(const std::string &type_name) :
      m_type_name(type_name),
      m_log_this(true),
      m_log_frame_convention(NWU) {}

  void FrLoggableBase::LogThis(bool log) {
    m_log_this = log;
  }

  void FrLoggableBase::StepFinalizeLog() {
    SerializeLogMessages();
    SendLogMessages();
  }

  void FrLoggableBase::SetLogFrameConvention(FRAME_CONVENTION fc) { m_log_frame_convention = fc; }

  bool FrLoggableBase::IsLogged() const {
    return m_log_this;
  }

  const std::string &FrLoggableBase::GetTypeName() const {
    return m_type_name;
  }

  hermes::Message *FrLoggableBase::NewMessage(const std::string &name, const std::string &description) {
    m_messages.emplace_back(std::make_unique<hermes::Message>(name, description));
    return m_messages.back().get();
  }

  void FrLoggableBase::InitializeLogMessages() {
    for (auto &message : m_messages) {
      message->Initialize();
    }
  }

  void FrLoggableBase::SerializeLogMessages() {
    for (auto &message : m_messages) {
      message->Serialize();
    }
  }

  void FrLoggableBase::SendLogMessages() {
    for (auto &message : m_messages) {
      message->Send();
    }
  }


}  // end namespace frydom
