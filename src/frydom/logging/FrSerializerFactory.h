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


#ifndef FRYDOM_FRSERIALIZATIONFACTORY_H_
#define FRYDOM_FRSERIALIZATIONFACTORY_H_

#include "hermes/hermes.h"

#include <map>

namespace frydom {

// Forward declaration
  class FrObject;

  class FrSerializerFactory {
   public:
    static FrSerializerFactory &instance() {
      static FrSerializerFactory i;
      return i;
    }

    hermes::Serializer *Create(const FrObject *object, const std::string &path) {
      auto classID = ClassID(object);
      if (creators_.find(classID) == std::end(creators_)) {
        return DefaultCreator(object, path);
      }
      return creators_[classID](object, path);
    }

    template<typename T, typename SerializerType>
    void Register(std::function<SerializerType *(const FrObject *, const std::string &)> creator) {
      creators_[ClassID<T>(nullptr)] = creator;
    }

   private:
    template<typename T>
    std::string ClassID(T *instance) {
      // use rtti for class id
      if (instance) {  // use instance value to support polymorphism
        return typeid(*instance).name();
      }
      return typeid(T).name();
    }

    hermes::Serializer *DefaultCreator(const FrObject *, const std::string &path) {
      return new hermes::CSVSerializer(path);
    }

    using Creator = std::function<hermes::Serializer *(const FrObject *, const std::string &)>;
    std::map<std::string, Creator> creators_;
  };

}  // end namespace frydom

#endif  // FRYDOM_FRSERIALIZATIONFACTORY_H_
