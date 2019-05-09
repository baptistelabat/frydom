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

    FrObject::FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {

        m_message = std::make_unique<hermes::Message>();
    }

    void FrObject::InitializeLog(const std::string& path) {

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

    void FrObject::SendLog() {

        if (IsLogged()) {
            m_message->Serialize();
            m_message->Send();
        }

    }

    std::string FrObject::BuildPath(const std::string &rootPath) {

        auto objPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());

        auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

        // Add a serializer
        m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));

        return objPath;
    }

    void FrObject::StepFinalize() {
            SendLog();
    }

    void FrObject::SetPathManager(const std::shared_ptr<FrPathManager> manager) {
            m_pathManager = manager;
        }

    std::shared_ptr<FrPathManager> FrObject::GetPathManager() const {
            return m_pathManager;
        }

    bool FrObject::IsLogged() { return m_isLogged; }

    FRAME_CONVENTION FrObject::GetLogFrameConvention() const {
            return m_pathManager->GetLogFrameConvention();
        }

    void FrObject::SetLogged(bool isLogged) { m_isLogged = isLogged; }

    void FrObject::ClearMessage() { m_message = std::make_unique<hermes::Message>(); }

    std::string FrObject::GetUUID() const { return m_UUID; }

    std::string FrObject::GetShortenUUID() const { return m_UUID.substr(0,5); }

    const char *FrObject::GetName() const { return m_name.c_str(); }

    void FrObject::SetName(const char *myname) { m_name = myname; }

    std::string FrObject::GetNameString() const { return m_name; }

    void FrObject::SetNameString(const std::string &myname) { m_name = myname; }


}  // end namespace frydom
