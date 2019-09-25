#include <utility>

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


#include "FrObject.h"

namespace frydom {

    FrObject::FrObject() : m_UUID(boost::lexical_cast<std::string>(boost::uuids::random_generator()())) {}

    std::string FrObject::GetUUID() const { return m_UUID; }

    std::string FrObject::GetShortenUUID() const { return m_UUID.substr(0, 5); }

//  const char *FrObject::GetName() const { return m_name.c_str(); }
//
//  void FrObject::SetName(const char *myname) { m_name = myname; }
//
//  std::string FrObject::GetNameString() const { return m_name; }
//
//  void FrObject::SetNameString(const std::string &myname) { m_name = myname; }

    void FrObject::StepFinalize() {}

}  // end namespace frydom
