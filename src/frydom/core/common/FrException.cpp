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


#include "FrException.h"


namespace frydom {


  FrException::FrException(const char *message) : msg_(message) {}

  FrException::FrException(const std::string &message) : msg_(message) {}

  FrException::~FrException() {}

  const char *FrException::what() const throw() {
    return msg_.c_str();
  }

} // end namespace frydom
