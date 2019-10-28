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

#include "FrPathManager.h"

namespace frydom {


  bool frydom::FrPathManager::RegisterPath(const std::string &path) {
    if (HasPath(path)) return false;

    m_used_paths.insert(path);
    return true;
  }

  bool frydom::FrPathManager::HasPath(const std::string &path) {
    return (m_used_paths.find(path) != m_used_paths.end());
  }

}// end namespace frydom
