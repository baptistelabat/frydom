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


#ifndef FRYDOM_FROBJECT_H
#define FRYDOM_FROBJECT_H

#include "hermes/hermes.h"

#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

#include "frydom/core/common/FrConvention.h"
#include "frydom/IO/FrPathManager.h"

namespace frydom {

    /**
     * \class FrObject
     * \brief Class for defining objects in FRyDoM.
     */
    class FrObject {

        // TODO : abandonner les uuid boost au profit d'un nameServer qui s'assure de l'unicite des noms donnes par l'utilisateur
        // La resolution de nom devra avoir lieu lors de l'initialisation des classes et le nameServer fera parti du OffshoreSystem...

    private:
        std::string m_UUID;
        std::string m_name;

    protected:

        // Logging
        bool m_isLogged = false;

        FRAME_CONVENTION c_logFrameConvention; // from LogManager

        std::unique_ptr<hermes::Message> m_message;

    public:

        FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {

            m_message = std::make_unique<hermes::Message>();
        }

        void SetLogFrameConvention(FRAME_CONVENTION fc) { c_logFrameConvention = fc; }

        bool IsLogged() { return m_isLogged; }

        void SetLogged(bool isLogged) { m_isLogged = isLogged; }

        std::string GetUUID() const { return m_UUID; }

        std::string GetShortenUUID() const { return m_UUID.substr(0,5); };

        virtual std::string GetTypeName() const = 0;

        /// Gets the name of the object as C Ascii null-terminated string -for reading only!
        const char* GetName() const { return m_name.c_str(); }
        /// Sets the name of this object, as ascii string
        void SetName(const char myname[]) { m_name = myname; }

        /// Gets the name of the object as C Ascii null-terminated string.
        std::string GetNameString() const { return m_name; }
        /// Sets the name of this object, as std::string
        void SetNameString(const std::string& myname) { m_name = myname; }

        /// Base method for Initialization of FryDoM objects
        ///
        /// This must be overrided in children classes in case of a need for special initialization at the beginning
        /// of a computation. Every Initialize() methods must be called indirectly when the call to
        /// FrOffshoreSystem::Initialize() is done.
        virtual void Initialize() = 0;

        /// This function is called at the end of the time step, after the last step of the integration scheme.
        virtual void StepFinalize() = 0;

        // Logging

        // Initialize the message log
        void InitializeLog(std::string path);

    protected:
        // Serialize and send the message
        void SendLog();
    };

}  // end namespace frydom

#endif //FRYDOM_FROBJECT_H
