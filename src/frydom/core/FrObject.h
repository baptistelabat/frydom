//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FROBJECT_H
#define FRYDOM_FROBJECT_H

#include "boost/lexical_cast.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

namespace frydom {

    class FrObject {

    private:
        std::string m_UUID;

    public:
        FrObject() :
                m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()()))
        {}

        std::string GetUUID() {
            return m_UUID;
        }

        // TODO: ajotuer une methode virtuelle Initialize()

    };

}  // end namespace frydom

#endif //FRYDOM_FROBJECT_H
