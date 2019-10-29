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

#ifndef FRYDOM_FRLOGGABLE_H
#define FRYDOM_FRLOGGABLE_H


#include <string>
#include <memory>

#include "hermes/hermes.h"

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrTreeNode.h"
#include "frydom/logging/FrLogManager.h"


namespace frydom {


  class FrLoggableBase {

   public:
    FrLoggableBase();

    void LogThis(bool log);

    virtual void StepFinalizeLog();

    void SetLogFrameConvention(FRAME_CONVENTION fc);

    bool IsLogged() const;

   protected:
    hermes::Message *NewMessage(const std::string &name, const std::string &description);

    virtual void DefineLogMessages() = 0;

    virtual const std::string &GetTreePath() const = 0;

    inline FRAME_CONVENTION GetLogFC() const { return m_log_frame_convention; }

   private:
    void InitializeLogMessages();

    void SerializeLogMessages();

    void SendLogMessages();

   private:
    bool m_log_this;

    FRAME_CONVENTION m_log_frame_convention;

    std::vector<std::unique_ptr<hermes::Message>> m_messages;

    std::string m_type_name;

    friend void FrLogManager::Initialize();

  };

  template<class ParentType>
  class FrLoggable : public FrLoggableBase, public FrTreeNode<ParentType> {

   public:
    explicit FrLoggable(const std::string &name, ParentType *parent) :
        FrLoggableBase(),
        FrTreeNode<ParentType>(name, parent) {}

    const std::string &GetTreePath() const override {
      return FrTreeNode<ParentType>::GetTreePath();
    }

  };

}  // end namespace frydom

#endif //FRYDOM_FRLOGGABLE_H
