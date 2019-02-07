// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FROBJECT_H
#define FRYDOM_FROBJECT_H

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

    public:
        FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {}

        std::string GetUUID() const{ return m_UUID; }

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
