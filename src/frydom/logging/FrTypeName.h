//
// Created by frongere on 28/10/19.
//

#ifndef FRYDOM_FRTYPENAME_H
#define FRYDOM_FRTYPENAME_H

#include <string>


//#include <iostream>
//#include <typeinfo>



namespace frydom {


  #define FRYDOM_DECLARE_CLASS_TYPE(T, type_name) \
  template <> \
  std::string GetTypeNameId<T>(const T& obj) { return type_name; }


  template <class T>
  std::string GetTypeNameId(const T& obj);


}  // end namespace frydom

#endif //FRYDOM_FRTYPENAME_H
