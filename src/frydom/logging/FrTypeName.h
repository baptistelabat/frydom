//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FRTYPENAME_H
#define FRYDOM_FRTYPENAME_H

#include <string>


namespace frydom {


  template <typename T>
  std::string TypeToString();

  template <typename T>
  std::string TypeToString(T*) {
    return TypeToString<T>();
  }


  #define TYPE_TO_STRING(Type, name) template <> std::string TypeToString<Type>() { return name; }

//  class FrOffshoreSystem;
//  TYPE_TO_STRING(FrOffshoreSystem, "OffshoreSystem")






//
//  class FrBody;
//  TYPE_TO_STRING(FrBody, "Body")


//  template <>
//  std::string TypeToString<FrOffshoreSystem>() { return "OffshoreSystem"; }



//  #define FRYDOM_DECLARE_CLASS_TYPE(T, type_name) \
//  template <> \
//  std::string GetTypeNameId<T>(const T& obj) { return type_name; }
//
//
//  template <class T>
//  std::string GetTypeNameId(const T& obj);




}  // end namespace frydom

#endif //FRYDOM_FRTYPENAME_H
