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

namespace frydom {

        void FrObject::InitializeLog(std::string path) {

            if (IsLogged()) {
                // Initializing message
                if (m_message->GetName().empty()) {
                    m_message->SetNameAndDescription(
                            fmt::format("{}_{}", GetTypeName(), GetShortenUUID()),
                            fmt::format("\"Message of a {}", GetTypeName()));
                }

                // Add a serializer
                m_message->AddSerializer(
                     FrSerializerFactory::instance().Create(this, path));

                // Init the message
                m_message->Initialize();
                m_message->Send();
            }

        }

        void FrObject::SendLog() {

            if (IsLogged()) {
                m_message->Serialize();
                m_message->Send();
            }

        }



}  // end namespace frydom
