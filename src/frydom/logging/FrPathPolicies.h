//
// Created by frongere on 03/12/2019.
//

#ifndef FRYDOM_EE_FRPATHPOLICIES_H
#define FRYDOM_EE_FRPATHPOLICIES_H

#include <string>

namespace frydom {

  template<typename T>
  std::string TypeToNormalizedPathPrefix();

  template<typename T>
  std::string TypeToNormalizedPathPrefix(T *) { // TODO: voir si on garde le const...
    return TypeToNormalizedPathPrefix<T>();
  }

  #define TYPE_TO_NORMALIZED_PATH_PREFIX(Type, prefix) template <> std::string TypeToNormalizedPathPrefix<Type>() { return prefix; }

}  // end namespace frydom



#endif //FRYDOM_EE_FRPATHPOLICIES_H
