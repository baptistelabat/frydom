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

        std::string FrObject::InitializeLog(const std::string& path) {

            if (IsLogged()) {

                auto logPath = BuildPath(path);

                AddFields();


                // Initializing message
                if (m_message->GetName().empty()) {
                    m_message->SetNameAndDescription(
                            fmt::format("{}_{}", GetTypeName(), GetShortenUUID()),
                            fmt::format("\"Message of a {}", GetTypeName()));
                }

                // Init the message
                m_message->Initialize();
                m_message->Send();

                return logPath;
            }

        }

        void FrObject::SendLog() {

            if (IsLogged()) {
                m_message->Serialize();
                m_message->Send();
            }

        }

    std::string FrObject::BuildPath(const std::string &rootPath) {
        c_logFrameConvention = GetPathManager()->GetLogFrameConvention();

        std::string bodyPath;

        if (rootPath.empty()) {
            bodyPath= fmt::format("{}_{}_{}", GetTypeName(), GetName(), GetShortenUUID());
        }
        else {
            bodyPath = fmt::format("{}/{}_{}_{}", rootPath, GetTypeName(), GetName(), GetShortenUUID());
        }

        auto logPath = GetPathManager()->BuildPath(bodyPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

        // Add a serializer
        m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));

        return logPath;
    }

    void FrObject::StepFinalize() {
            SendLog();
    }


}  // end namespace frydom
