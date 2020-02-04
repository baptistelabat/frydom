//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FRTYPENAMES_H
#define FRYDOM_FRTYPENAMES_H

#include <string>


namespace frydom {

  template<typename T>
  std::string TypeToString();

  template<typename T>
  std::string TypeToString(T *) {
    return TypeToString<T>();
  }

  #define TYPE_TO_STRING(Type, name) template <> std::string TypeToString<Type>() { return name; }

  // Every classes names have to be declared in the CPP file...

}  // end namespace frydom

#endif //FRYDOM_FRTYPENAMES_H
