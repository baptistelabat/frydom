#ifndef FRYDOM_FRSERIALIZATIONFACTORY_H_
#define FRYDOM_FRSERIALIZATIONFACTORY_H_

#include "hermes/hermes.h"

#include <map>

namespace frydom {

class FrSerializerFactory {
public:
    static FrSerializerFactory &instance() {
        static FrSerializerFactory i;
        return i;
    }

    hermes::Serializer *Create(FrObject *object, const std::string &path) {
        auto classID = ClassID(object);
        if (creators_.find(classID) == std::end(creators_)) {
            return new DefaultSerializer(path);
        }
        return creators_[classID](path);
    }

    template <typename T, typename SerializerType>
    void Register(std::function<SerializerType*(const std::string &)> creator) {
        creators_[ClassID<T>(nullptr)] = creator;
    }

private:
    template <typename T>
    std::string ClassID(T* instance) {
        // use rtti for class id
        if (instance) {  // use instance value to support polymorphism
            return typeid(*instance).name();
        }
        return typeid(T).name();
    }

    using DefaultSerializer = hermes::CSVSerializer;
    using Creator = std::function<hermes::Serializer *(const std::string &)>;
    std::map<std::string, Creator> creators_;
};

}  // end namespace frydom

#endif  // FRYDOM_FRSERIALIZATIONFACTORY_H_
