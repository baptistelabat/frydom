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

//#include "hermes/hermes.h"

#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

namespace frydom {

    /**
     * \class FrObject
     * \brief Class for defining objects in FRyDoM.
     */
    class FrObject {

    private:
        std::string m_UUID;

    protected:
//        hermes::Message m_message;
        std::string m_logPath;

    public:
        FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {}

        std::string GetUUID() const{ return m_UUID; }

        void SetFilePath (std::string path) { m_logPath = path; }

        std::string GetFilePath() const { return m_logPath; }

//        void SetFilePath (std::string path) { m_message.SetFilePath(path); }
//
//        std::string GetFilePath() const { return m_message.GetFilePath(); }

        /// Base method for Initialization of FryDoM objects
        ///
        /// This must be overrided in children classes in case of a need for special initialization at the beginning
        /// of a computation. Every Initialize() methods must be called indirectly when the call to
        /// FrOffshoreSystem::Initialize() is done.
        virtual void Initialize() = 0;

        virtual void StepFinalize() = 0;

    };

}  // end namespace frydom

#endif //FRYDOM_FROBJECT_H
